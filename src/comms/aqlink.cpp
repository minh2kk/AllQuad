#include "../common/size_opt.h"  // -Os on ESP32 (not flight critical)
#include "aqlink.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../bus/bus.h"
#include "../web/credentials.h"
#include "../web/webapi.h"
#include "../logging/sdlog.h"

namespace comms {

uint16_t AqLink::crc16(const uint8_t* d, size_t n, uint16_t crc) {
  for (size_t i = 0; i < n; i++) {
    crc ^= (uint16_t)d[i] << 8;
    for (int b = 0; b < 8; b++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

// ---------------------------------------------------------------- receive
bool AqLink::feed(uint8_t b, uint32_t now) {
  if (state_ > 1 && now - frameStartMs_ > 1000) state_ = 0;  // a frame that never finished
  switch (state_) {
    case 0:
      if (b == 0xA7) { state_ = 1; frameStartMs_ = now; return true; }
      return false;
    case 1:
      state_ = b == 0x51 ? 2 : 0;
      return b == 0x51;
    case 2: case 3: case 4:
      hdr_[state_ - 2] = b;
      if (++state_ == 5) {
        len_ = (uint16_t)(hdr_[1] | hdr_[2] << 8);
        got_ = 0;
        if (len_ > RX_MAX) { state_ = 0; return true; }
        state_ = len_ ? 5 : 6;
      }
      return true;
    case 5:
      rx_[got_++] = b;
      if (got_ >= len_) state_ = 6;
      return true;
    case 6:
      crcBuf_[0] = b;
      state_ = 7;
      return true;
    case 7: {
      crcBuf_[1] = b;
      state_ = 0;
      uint16_t c = crc16(hdr_, 3);
      c = crc16(rx_, len_, c);
      if (c != (uint16_t)(crcBuf_[0] | crcBuf_[1] << 8)) { crcErrors_++; return true; }
      rx_[len_] = 0;
      seq_ = hdr_[0];
      lastMs_ = now ? now : 1;
      frames_++;
      bus::cliActiveUntilMs = now + 3000;  // arming blocked while the configurator is connected
      handle(now);
      return true;
    }
  }
  state_ = 0;
  return false;
}

// ---------------------------------------------------------------- transmit
static void put16(uint8_t* p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put32(uint8_t* p, uint32_t v) { put16(p, v); put16(p + 2, v >> 16); }

void AqLink::reply(int status, uint32_t total, uint32_t off, const uint8_t* data, size_t len) {
  if (len > DATA_MAX) len = DATA_MAX;
  uint8_t* t = tx_;
  t[0] = 0xA7; t[1] = 0x52; t[2] = seq_;
  put16(t + 3, (uint32_t)status);
  put32(t + 5, total);
  put32(t + 9, off);
  put16(t + 13, (uint32_t)len);
  if (len && data != t + 15) memmove(t + 15, data, len);
  uint16_t c = crc16(t + 2, 13 + len);
  put16(t + 15 + len, c);
  txLen_ = 17 + len;
  txPos_ = 0;
}

static size_t jsonMsg(char* out, size_t n, int status, const char* msg) {
  int k = snprintf(out, n, "{\"ok\":%s,\"msg\":\"", status < 300 ? "true" : "false");
  for (const char* p = msg; *p && k < (int)n - 4; p++) {  // messages are plain ASCII, quotes escaped
    if (*p == '"' || *p == '\\') out[k++] = '\\';
    out[k++] = *p;
  }
  k += snprintf(out + k, n - k, "\"}");
  return (size_t)k;
}

void AqLink::replyMsg(int status, const char* msg) {
  char b[200];
  size_t n = jsonMsg(b, sizeof(b), status, msg);
  reply(status, (uint32_t)n, 0, (const uint8_t*)b, n);
}

void AqLink::update(uint32_t now) {
  while (txPos_ < txLen_) {
    size_t room = hal::portWriteSpace(port_);
    if (!room) break;
    size_t n = txLen_ - txPos_ < room ? txLen_ - txPos_ : room;
    size_t w = hal::portWrite(port_, tx_ + txPos_, n);
    if (!w) break;
    txPos_ += w;
  }
  if (txPos_ < txLen_) return;
  // deferred actions once the reply is out
  if (rebootPending_ && now - lastMs_ > 100) {
    rebootPending_ = false;
    if (bootloaderPending_) hal::rebootToBootloader();
    else hal::reboot();
    bootloaderPending_ = false;
  }
  if (wifiRestart_) { wifiRestart_ = false; hal::wifiStart(creds::ssid(), creds::wifiPass()); }
}

// ---------------------------------------------------------------- requests
static int hexv(char c) {
  return c >= '0' && c <= '9' ? c - '0' : (c | 32) >= 'a' && (c | 32) <= 'f' ? (c | 32) - 'a' + 10 : -1;
}
static void urlDecode(char* s) {
  char* o = s;
  for (; *s; s++) {
    if (*s == '+') *o++ = ' ';
    else if (*s == '%' && hexv(s[1]) >= 0 && hexv(s[2]) >= 0) { *o++ = (char)(hexv(s[1]) * 16 + hexv(s[2])); s += 2; }
    else *o++ = *s;
  }
  *o = 0;
}
// value of key in "a=1&b=2" (decoded), false if absent
static bool field(const char* q, const char* key, char* out, size_t n) {
  if (!q) return false;
  size_t kl = strlen(key);
  for (const char* p = q; p && *p;) {
    const char* amp = strchr(p, '&');
    const char* end = amp ? amp : p + strlen(p);
    if ((size_t)(end - p) > kl && !strncmp(p, key, kl) && p[kl] == '=') {
      size_t vl = (size_t)(end - p - kl - 1);
      if (vl >= n) vl = n - 1;
      memcpy(out, p + kl + 1, vl);
      out[vl] = 0;
      urlDecode(out);
      return true;
    }
    p = amp ? amp + 1 : nullptr;
  }
  return false;
}
static uint32_t fieldU(const char* q, const char* key, uint32_t def) {
  char v[16];
  return field(q, key, v, sizeof(v)) ? (uint32_t)strtoul(v, nullptr, 10) : def;
}

// Sink that keeps the window [skip, skip + cap) of a generated document and counts its size:
// pages of a long JSON are produced by running the generator again (cheap, no big buffer).
struct Window {
  uint8_t* buf;
  size_t cap, len;
  uint32_t skip, total;
};
static void windowSink(const char* d, size_t n, void* ctx) {
  Window& w = *static_cast<Window*>(ctx);
  uint32_t start = w.total;
  w.total += (uint32_t)n;
  if (w.total <= w.skip || w.len >= w.cap) return;
  size_t from = start < w.skip ? w.skip - start : 0;
  size_t k = n - from;
  if (k > w.cap - w.len) k = w.cap - w.len;
  memcpy(w.buf + w.len, d + from, k);
  w.len += k;
}

void AqLink::handle(uint32_t now) {
  char* path = (char*)rx_;
  size_t plen = strnlen(path, len_);
  const uint8_t* body = plen < len_ ? rx_ + plen + 1 : nullptr;
  size_t blen = body ? len_ - plen - 1 : 0;
  if (!blen) body = nullptr;
  char* q = strchr(path, '?');
  if (q) *q++ = 0;
  uint32_t off = fieldU(q, "o", 0);

  txLen_ = txPos_ = 0;  // a new request: an unfinished older reply is dropped (the client gave up on it)

  // documents generated on demand, paged
  void (*genFn)(web::Sink, void*, const char*) = nullptr;
  if (!strcmp(path, "/api/params")) genFn = [](web::Sink s, void* c, const char*) { web::paramsJson(s, c); };
  else if (!strcmp(path, "/api/status")) genFn = [](web::Sink s, void* c, const char* qq) { web::statusJson(s, c, fieldU(qq, "full", 1) != 0); };
  else if (!strcmp(path, "/api/mission")) genFn = [](web::Sink s, void* c, const char*) { web::missionJson(s, c); };
  else if (!strcmp(path, "/api/sensors")) genFn = [](web::Sink s, void* c, const char*) { web::sensorsJson(s, c); };
  else if (!strcmp(path, "/api/sd")) genFn = [](web::Sink s, void* c, const char*) { web::sdJson(s, c); };
  else if (!strcmp(path, "/api/backup")) genFn = [](web::Sink s, void* c, const char* qq) { web::backupText(s, c, fieldU(qq, "all", 0) == 1); };
  if (genFn) {
    Window w{tx_ + 15, DATA_MAX, 0, off, 0};
    genFn(windowSink, &w, q);
    reply(200, w.total, off, tx_ + 15, w.len);
    return;
  }

  char msg[120];
  if (!strcmp(path, "/api/hello")) {
    char b[200];
    uint32_t baud = hal::consoleBaud();
    int n = snprintf(b, sizeof(b),
                     "{\"ok\":true,\"name\":\"AllQuad\",\"board\":\"%s\",\"usb\":%s,\"baud\":%lu,\"max\":%u,\"up\":%lu,\"rr\":%d}",
                     hal::platformName(), baud ? "false" : "true", (unsigned long)baud, (unsigned)DATA_MAX,
                     (unsigned long)hal::millis(), hal::resetReason());
    reply(200, (uint32_t)n, 0, (const uint8_t*)b, (size_t)n);
  } else if (!strcmp(path, "/api/set")) {
    if (!q) return replyMsg(400, "no parameters");
    int worst = 200;
    char failed[96] = "";
    for (char* save = nullptr, *kv = strtok_r(q, "&", &save); kv; kv = strtok_r(nullptr, "&", &save)) {
      char* eq = strchr(kv, '=');
      if (!eq) continue;
      *eq = 0;
      urlDecode(kv);
      urlDecode(eq + 1);
      int r = web::setParam(kv, eq + 1);
      if (r != 200) {
        worst = r;
        strncat(failed, kv, sizeof(failed) - strlen(failed) - 2);
        strncat(failed, " ", sizeof(failed) - strlen(failed) - 1);
      }
    }
    replyMsg(worst, worst == 200 ? "applied (not saved)" : (worst == 423 ? "disarm first" : failed));
  } else if (!strcmp(path, "/api/cli")) {
    // the line runs once (request with a body); long output is then read in pages from the buffer
    if (body) {
      cliLen_ = 0;
      char line[160];
      size_t k = blen < sizeof(line) - 1 ? blen : sizeof(line) - 1;
      memcpy(line, body, k);
      line[k] = 0;
      if (!cliBuf_) cliBuf_ = (char*)malloc(CLI_MAX);
      if (!cliBuf_) return replyMsg(400, "out of memory");
      web::cliExec(line, [](const char* d, size_t n, void* ctx) {
        AqLink& l = *static_cast<AqLink*>(ctx);
        size_t k2 = n < CLI_MAX - l.cliLen_ ? n : CLI_MAX - l.cliLen_;
        memcpy(l.cliBuf_ + l.cliLen_, d, k2);
        l.cliLen_ += k2;
      }, this, true);
    }
    size_t n = off >= cliLen_ ? 0 : (cliLen_ - off < DATA_MAX ? cliLen_ - off : DATA_MAX);
    if (n) memcpy(tx_ + 15, cliBuf_ + off, n);
    reply(200, (uint32_t)cliLen_, off, tx_ + 15, n);
  } else if (!strcmp(path, "/api/motor")) {
    float v[4];
    char k[4] = "m1";
    for (int i = 0; i < 4; i++) {
      k[1] = (char)('1' + i);
      char val[16];
      v[i] = field(q, k, val, sizeof(val)) ? (float)atof(val) : 0.0f;
    }
    int r = web::motorTest(v, msg, sizeof(msg));
    replyMsg(r, msg);
  } else if (!strcmp(path, "/api/save")) {
    bool ok = web::save();
    replyMsg(ok ? 200 : 423, ok ? "saved" : "disarm first");
  } else if (!strcmp(path, "/api/cmd")) {
    char c[24] = "";
    field(q, "c", c, sizeof(c));
    int r = web::command(c, msg, sizeof(msg));
    if (r == 202 || r == 205) rebootPending_ = true;
    if (r == 205) { bootloaderPending_ = true; r = 200; }
    replyMsg(r, msg);
  } else if (!strcmp(path, "/api/blackbox")) {
    if (web::armed()) return replyMsg(423, "disarm first");
    if (!off) hal::logFlush();
    uint32_t size = hal::logSize();
    if (!size) return replyMsg(404, "no log");
    size_t n = off >= size ? 0 : (size - off < DATA_MAX ? size - off : DATA_MAX);
    if (n && !hal::logRead(off, tx_ + 15, n)) return replyMsg(400, "read error");
    reply(200, size, off, tx_ + 15, n);
  } else if (!strcmp(path, "/api/sdfile")) {  // ?n=AQ00012.BBL&o=offset: one log file from the SD card
    if (web::armed()) return replyMsg(423, "disarm first");
    char name[16] = "";
    field(q, "n", name, sizeof(name));
    if (!sdlog::validName(name)) return replyMsg(400, "bad file name");
    size_t got = 0;
    uint32_t size = 0;
    if (!sdlog::read(name, off, tx_ + 15, DATA_MAX, got, size)) return replyMsg(409, "no card / busy / no file");
    reply(200, size, off, tx_ + 15, got);
  } else if (!strcmp(path, "/api/restore")) {
    // pieces: o = offset of this piece, n = file size; the file runs once it is complete
    uint32_t total = fieldU(q, "n", 0);
    if (web::armed()) return replyMsg(423, "disarm first");
    if (!total || total > 24 * 1024) return replyMsg(400, "file empty or too large");
    if (off == 0) {
      free(upload_);
      upload_ = (char*)malloc(total + 1);
      uploadLen_ = 0;
      uploadTotal_ = total;
      if (!upload_) return replyMsg(400, "out of memory");
    }
    if (!upload_ || total != uploadTotal_ || off != uploadLen_ || off + blen > total) {
      if (upload_ && off + blen == uploadLen_ && off < uploadLen_) return replyMsg(202, "more");  // repeated piece
      return replyMsg(400, "upload out of order");
    }
    if (blen) memcpy(upload_ + off, body, blen);
    uploadLen_ += (uint32_t)blen;
    if (uploadLen_ < uploadTotal_) return replyMsg(202, "more");
    int r = web::restoreText(upload_, uploadLen_, msg, sizeof(msg));
    free(upload_);
    upload_ = nullptr;
    replyMsg(r, msg);
  } else if (!strcmp(path, "/api/wifi")) {
    // form in the body, like the web page; over USB the current password is not needed
    // (this is how a forgotten WiFi / web password is reset)
    char form[320] = "", ssid[40] = "", wp[70] = "", webp[70] = "";
    size_t k = blen < sizeof(form) - 1 ? blen : sizeof(form) - 1;
    if (body) memcpy(form, body, k);
    form[k] = 0;
    bool hasSsid = field(form, "ssid", ssid, sizeof(ssid));
    field(form, "wifi", wp, sizeof(wp));
    bool hasWeb = field(form, "web", webp, sizeof(webp));
    int r = web::changeWifi(hasSsid ? ssid : nullptr, wp, hasWeb ? webp : nullptr, nullptr, msg, sizeof(msg), true);
    if (r == 200) wifiRestart_ = true;
    replyMsg(r, msg);
  } else if (!strcmp(path, "/api/login") || !strcmp(path, "/api/logout")) {
    replyMsg(200, "usb");  // no sessions over USB
  } else if (!strcmp(path, "/api/ota")) {
    replyMsg(501, "firmware update over USB: use the flasher (esptool / DFU)");
  } else {
    replyMsg(404, "not found");
  }
}

}  // namespace comms
