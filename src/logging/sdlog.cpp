#include "../common/size_opt.h"  // -Os on ESP32 (not flight critical)
#include "sdlog.h"

#include <stdio.h>
#include <string.h>

#include <atomic>

#include "../hal/hal.h"

namespace sdlog {

namespace {
// queue: monotonic byte counters (wrap at 2^32), size a power of two
uint8_t* q = nullptr;
uint32_t qSize = 0, qMask = 0;
std::atomic<uint32_t> head{0}, tail{0};
// flight markers (producer increments, consumer catches up)
std::atomic<uint32_t> starts{0}, ends{0}, closeAt{0};
uint32_t opened = 0, closed = 0;
std::atomic<bool> accept{false};
std::atomic<uint32_t> dropped{0};
// card ownership: 0 free, 1 log writer, 2 configurator
std::atomic<uint8_t> owner{0};

bool enabled_ = false, fileOpen = false, mountedOnce = false;
bool lastFailed = false;  // the last log ended with an error: keep the message until a log succeeds
std::atomic<bool> remountReq{false};
uint8_t mode = 0, mhz = 0, pins[6];
State state = State::OFF;
char err[48] = "";
char fileName[24] = "";
uint32_t fileBytes = 0, maxWriteMs = 0, lastMountMs = 0, lastSyncMs = 0, lastWriteMs = 0;
uint64_t total = 0, used = 0;
uint16_t files = 0;
uint32_t nextNum = 1;
uint8_t keep = 1;

bool lock(uint8_t who) {
  uint8_t f = 0;
  return owner.compare_exchange_strong(f, who) || f == who;
}
void unlock(uint8_t who) {
  uint8_t w = who;
  owner.compare_exchange_strong(w, 0);
}
void setErr(const char* e) { snprintf(err, sizeof(err), "%s", e); }

// log file number from its name, 0 = not ours
uint32_t numOf(const char* name) {
  if (!validName(name)) return 0;
  uint32_t v = 0;
  for (int i = 2; i < 7; i++) v = v * 10 + (uint32_t)(name[i] - '0');
  return v;
}

struct Scan { uint16_t count; uint32_t maxNum, minNum; };
void scanCb(const char* name, uint32_t, void* ctx) {
  Scan& s = *static_cast<Scan*>(ctx);
  uint32_t v = numOf(name);
  if (!v) return;
  s.count++;
  if (v > s.maxNum) s.maxNum = v;
  if (!s.minNum || v < s.minNum) s.minNum = v;
}
Scan scan() {
  Scan s{0, 0, 0};
  hal::sdList(DIR, scanCb, &s);
  return s;
}
void refreshSpace() {
  if (!hal::sdSpace(total, used)) total = used = 0;
}
void path(char* out, size_t n, const char* name) { snprintf(out, n, "%s/%s", DIR, name); }

// below MIN_FREE: delete the oldest logs (sd_keep 1), never the one just written. Disarmed only.
void makeRoom() {
  refreshSpace();
  for (int guard = 0; guard < 64 && keep && total && total - used < MIN_FREE; guard++) {
    Scan s = scan();
    if (s.count <= 1 || !s.minNum) break;
    char name[24], p[40];
    snprintf(name, sizeof(name), "AQ%05lu.BBL", (unsigned long)(s.minNum % 100000u));
    path(p, sizeof(p), name);
    if (!hal::sdRemove(p)) break;
    refreshSpace();
  }
  Scan s = scan();
  files = s.count;
  if (s.maxNum >= nextNum) nextNum = s.maxNum + 1;
}

void tryMount(uint32_t now) {
  lastMountMs = now;
  if (!lock(1)) return;
  char e[48] = "";
  if (hal::sdMount(mode, mhz, pins, e, sizeof(e))) {
    mountedOnce = true;
    if (!lastFailed) err[0] = 0;
    nextNum = 1;  // continue after the highest log on this card (makeRoom scans it)
    makeRoom();
    state = total && total - used < MIN_FREE ? State::FULL : State::READY;
    accept = state == State::READY;
  } else {
    setErr(e[0] ? e : "no card");
    state = State::NO_CARD;
    accept = false;
  }
  unlock(1);
}

void closeFile(bool ok) {
  hal::sdSync();
  hal::sdClose();
  fileOpen = false;
  closed = ends.load();
  if (ok) {
    files++;
    lastFailed = false;
    err[0] = 0;
    makeRoom();
    state = total && total - used < MIN_FREE ? State::FULL : State::READY;
  }
  accept = state == State::READY;
  unlock(1);
}

// a write failed (card pulled, full, broken): the card is released and mounted again by service()
void fail(const char* why) {
  setErr(why);
  lastFailed = true;
  hal::sdClose();
  hal::sdUnmount();
  fileOpen = false;
  accept = false;
  opened = starts.load();
  closed = ends.load();
  tail.store(head.load());
  state = State::ERROR;
  lastMountMs = hal::millis();  // next mount attempt in 3 s
  unlock(1);
}

// write the queue up to `limit`; small remainders wait unless forced (fewer, larger card writes)
bool drain(uint32_t limit, bool force, uint32_t now) {
  for (int k = 0; k < 8; k++) {
    uint32_t t = tail.load();
    uint32_t pend = limit - t;
    if (!pend || (!force && pend < 4096)) return true;
    uint32_t at = t & qMask;
    uint32_t chunk = pend;
    if (chunk > qSize - at) chunk = qSize - at;
    if (chunk > 16384) chunk = 16384;
    if (!force && chunk > 512) chunk &= ~511u;  // whole sectors
    uint32_t t0 = hal::millis();
    size_t w = hal::sdAppend(q + at, chunk);
    uint32_t dt = hal::millis() - t0;
    if (dt > maxWriteMs) maxWriteMs = dt;
    if (w != chunk) return false;
    tail.store(t + chunk);
    fileBytes += chunk;
    lastWriteMs = now;
  }
  return true;
}
}  // namespace

const char* stateName(State s) {
  static const char* n[] = {"off", "unsupported", "no card", "ready", "logging", "full", "error"};
  return n[(int)s];
}

bool validName(const char* n) {
  if (!n || strlen(n) != 11 || n[0] != 'A' || n[1] != 'Q' || strcmp(n + 7, ".BBL")) return false;
  for (int i = 2; i < 7; i++)
    if (n[i] < '0' || n[i] > '9') return false;
  return true;
}

void begin(const fc::Config& c) {
  mode = c.sd_mode;
  mhz = c.sd_mhz;
  hal::sdSetDma(c.sd_dma != 0);
  keep = c.sd_keep;
  enabled_ = false;
  state = State::OFF;
  err[0] = 0;
  memset(pins, 255, sizeof(pins));
  if (!mode) return;
  if (mode > 3 || !(hal::sdModes() & (1u << mode))) {
    state = State::UNSUPPORTED;
    setErr("mode not available on this board");
    return;
  }
  uint8_t def[6];
  hal::sdDefaultPins(def);
  bool fixed = hal::sdPinsFixed();
  for (int i = 0; i < 6; i++) pins[i] = fixed || c.sd_pin[i] == 255 ? def[i] : c.sd_pin[i];
  // pins each mode needs: SPI SCK / MOSI / MISO / CS, 1-bit CLK / CMD / D0, 4-bit all six
  static const uint8_t need[4] = {0, 0x27, 0x07, 0x3F};
  for (int i = 0; i < 6; i++) {
    if (!(need[mode] & (1 << i))) { pins[i] = 255; continue; }
    if (pins[i] == 255 && !fixed) {
      state = State::UNSUPPORTED;
      setErr("set the sd_ pins for this mode");
      return;
    }
  }
  if (!q) {
    size_t got = 0;
    uint8_t* b = hal::sdQueueAlloc(256 * 1024, got);
    uint32_t s = 1;
    while (s * 2 <= got) s *= 2;  // power of two
    if (!b || s < 4096) {
      state = State::UNSUPPORTED;
      setErr("no RAM for the queue");
      return;
    }
    q = b;
    qSize = s;
    qMask = s - 1;
  }
  head = tail = 0;
  starts = ends = closeAt = 0;
  opened = closed = 0;
  dropped = 0;
  fileOpen = mountedOnce = false;
  lastMountMs = 0;
  state = State::NO_CARD;
  enabled_ = true;
  remountReq = true;
}

bool enabled() { return enabled_; }

void startFile() {
  if (!enabled_) return;
  if (state == State::READY || state == State::LOGGING || state == State::ERROR) accept = hal::sdMounted();
  starts.fetch_add(1);
}

void endFile() {
  if (!enabled_) return;
  closeAt.store(head.load());
  ends.fetch_add(1);
}

bool push(const uint8_t* d, size_t n) {
  if (!enabled_ || !accept.load(std::memory_order_relaxed)) { dropped.fetch_add((uint32_t)n); return false; }
  uint32_t h = head.load(std::memory_order_relaxed);
  uint32_t t = tail.load(std::memory_order_acquire);
  if (qSize - (h - t) < n) { dropped.fetch_add((uint32_t)n); return false; }
  uint32_t at = h & qMask;
  size_t a = n < qSize - at ? n : qSize - at;
  memcpy(q + at, d, a);
  if (a < n) memcpy(q, d + a, n - a);
  head.store(h + (uint32_t)n, std::memory_order_release);
  return true;
}

void remount() { remountReq = true; }

void service(uint32_t now) {
  if (!enabled_) return;
  if (!fileOpen && (!hal::sdMounted() || remountReq)) {
    if (remountReq || !mountedOnce || now - lastMountMs >= 3000) {
      bool req = remountReq.exchange(false);
      if (req && hal::sdMounted() && lock(1)) { hal::sdReadClose(); hal::sdUnmount(); unlock(1); }
      tryMount(now);
    }
    if (!hal::sdMounted()) {
      // nothing can be written: keep the counters in step (data of this flight is lost)
      opened = starts.load();
      closed = ends.load();
      tail.store(head.load());
      return;
    }
  }
  if (!fileOpen) {
    if (opened != starts.load()) {
      if (!lock(1)) return;  // the configurator is reading: try again next time (the queue keeps the data)
      opened = starts.load();
      hal::sdReadClose();
      if (total && total - used < MIN_FREE) makeRoom();  // sd_keep: the oldest logs go
      if (total && total - used < MIN_FREE) {
        state = State::FULL;
        setErr("card full");
        unlock(1);
        accept = false;
        return;
      }
      char p[40];
      snprintf(fileName, sizeof(fileName), "AQ%05lu.BBL", (unsigned long)(nextNum % 100000u));
      path(p, sizeof(p), fileName);
      if (!hal::sdCreate(p)) { fail("cannot create the file"); return; }
      nextNum++;
      fileOpen = true;
      fileBytes = 0;
      lastSyncMs = lastWriteMs = now;
      state = State::LOGGING;
    } else {
      closed = ends.load();
      tail.store(head.load());  // stray bytes between flights
      return;
    }
  }
  bool closing = closed != ends.load();
  uint32_t limit = closing ? closeAt.load() : head.load();
  if (!drain(limit, closing || now - lastWriteMs >= 500, now)) {
    fail(hal::sdMounted() ? "write failed (card full or removed)" : "card removed");
    return;
  }
  if (now - lastSyncMs >= 1000) {
    lastSyncMs = now;
    if (!hal::sdSync()) { fail("card removed"); return; }
  }
  if (closing && tail.load() == limit) closeFile(true);
}

Info info() {
  Info i;
  memset(&i, 0, sizeof(i));
  i.state = state;
  i.mode = mode;
  i.type = hal::sdMounted() ? hal::sdCardType() : "";
  i.err = err;
  i.total = total;
  i.used = used;
  memcpy(i.file, fileName, sizeof(i.file));
  i.fileBytes = fileBytes;
  i.queued = head.load() - tail.load();
  i.queueSize = qSize;
  i.dropped = dropped.load();
  i.maxWriteMs = maxWriteMs;
  i.files = files;
  memcpy(i.pin, pins, sizeof(pins));
  return i;
}

bool list(void (*cb)(const char*, uint32_t, void*), void* ctx) {
  if (!enabled_ || !hal::sdMounted() || !lock(2)) return false;
  hal::sdList(DIR, cb, ctx);
  unlock(2);
  return true;
}

bool read(const char* name, uint32_t off, uint8_t* buf, size_t len, size_t& got, uint32_t& size) {
  got = 0;
  size = 0;
  if (!enabled_ || !validName(name) || !hal::sdMounted() || !lock(2)) return false;
  char p[40];
  path(p, sizeof(p), name);
  bool ok = hal::sdRead(p, off, buf, len, got, size);
  if (!ok || off + got >= size) hal::sdReadClose();
  unlock(2);
  return ok;
}

bool remove(const char* name) {
  if (!enabled_ || !validName(name) || !hal::sdMounted() || !lock(2)) return false;
  char p[40];
  path(p, sizeof(p), name);
  hal::sdReadClose();
  bool ok = hal::sdRemove(p);
  if (ok) {
    refreshSpace();
    if (files) files--;
    if (state == State::FULL && total - used >= MIN_FREE) { state = State::READY; accept = true; }
  }
  unlock(2);
  return ok;
}

}  // namespace sdlog
