#include "../common/size_opt.h"  // -Os on ESP32 (not flight critical)
#include "mission.h"

#include <math.h>
#include <string.h>

#include "../bus/bus.h"
#include "../config/config.h"
#include "../hal/hal.h"
#include "../os/os.h"

namespace nav {

static MissionStore store;
static volatile uint32_t gen = 1;

int count() { return store.count; }

bool get(int i, MissionItem& out) {
  os::CriticalGuard g;
  if (i < 0 || i >= store.count) return false;
  out = store.items[i];
  return true;
}

bool set(int i, const MissionItem& it) {
  if (i < 0 || i >= MISSION_MAX) return false;
  os::CriticalGuard g;
  store.items[i] = it;
  gen = gen + 1;
  return true;
}

void setCount(int n) {
  os::CriticalGuard g;
  store.count = (uint16_t)(n < 0 ? 0 : (n > MISSION_MAX ? MISSION_MAX : n));
  gen = gen + 1;
}

void clear() { setCount(0); }
uint32_t generation() { return gen; }

static uint32_t storeCrc() { return fc::crc32(&store, offsetof(MissionStore, crc)); }

bool load() {
  MissionStore tmp;
  if (!hal::storageLoad(hal::Slot::MISSION, &tmp, sizeof(tmp))) return false;
  if (tmp.count > MISSION_MAX || tmp.crc != fc::crc32(&tmp, offsetof(MissionStore, crc))) return false;
  os::CriticalGuard g;
  store = tmp;
  gen = gen + 1;
  return true;
}

// Flash writes stall the CPU (STM32: the whole MCU for a sector erase, ESP32: both cores).
// While armed they are only remembered and done after landing (persistPending()).
static volatile uint8_t pending = 0;

bool save() {
  if (bus::status.get().armed) { pending = pending | 1; return true; }
  pending = pending & ~1;
  MissionStore tmp;
  {
    os::CriticalGuard g;
    store.version = 1;
    store.crc = storeCrc();
    tmp = store;
  }
  return hal::storageSave(hal::Slot::MISSION, &tmp, sizeof(tmp));
}

bool supported(uint16_t cmd) {
  switch (cmd) {
    case CMD_WAYPOINT: case CMD_LOITER_UNLIM: case CMD_LOITER_TURNS: case CMD_LOITER_TIME: case CMD_RTL:
    case CMD_LAND: case CMD_TAKEOFF: case CMD_CONDITION_YAW: case CMD_DO_JUMP: case CMD_CHANGE_SPEED:
    case CMD_DO_SET_ROI_LOC: case CMD_DO_SET_ROI_NONE: case CMD_DO_SET_ROI:
      return true;
    default:
      return false;
  }
}

// ---------------------------------------------------------------- rally points
struct RallyStore {
  uint16_t count;
  uint16_t version;
  MissionItem items[RALLY_MAX];
  uint32_t crc;
};
static RallyStore rally;

int rallyCount() { return rally.count; }

bool rallyGet(int i, MissionItem& out) {
  os::CriticalGuard g;
  if (i < 0 || i >= rally.count) return false;
  out = rally.items[i];
  return true;
}

bool rallySet(int i, const MissionItem& it) {
  if (i < 0 || i >= RALLY_MAX) return false;
  os::CriticalGuard g;
  rally.items[i] = it;
  return true;
}

void rallySetCount(int n) {
  os::CriticalGuard g;
  rally.count = (uint16_t)(n < 0 ? 0 : (n > RALLY_MAX ? RALLY_MAX : n));
}

bool rallyLoad() {
  RallyStore tmp;
  if (!hal::storageLoad(hal::Slot::RALLY, &tmp, sizeof(tmp))) return false;
  if (tmp.count > RALLY_MAX || tmp.crc != fc::crc32(&tmp, offsetof(RallyStore, crc))) return false;
  os::CriticalGuard g;
  rally = tmp;
  return true;
}

bool rallySave() {
  if (bus::status.get().armed) { pending = pending | 2; return true; }
  pending = pending & ~2;
  RallyStore tmp;
  {
    os::CriticalGuard g;
    rally.version = 1;
    rally.crc = fc::crc32(&rally, offsetof(RallyStore, crc));
    tmp = rally;
  }
  return hal::storageSave(hal::Slot::RALLY, &tmp, sizeof(tmp));
}

// ---------------------------------------------------------------- geofence polygon
struct FenceStore {
  uint16_t count;
  uint16_t version;
  MissionItem items[FENCE_MAX];
  uint32_t crc;
};
static FenceStore fence;
static volatile uint32_t fenceGen = 1;

int fenceCount() { return fence.count; }
bool fenceGet(int i, MissionItem& out) {
  os::CriticalGuard g;
  if (i < 0 || i >= fence.count) return false;
  out = fence.items[i];
  return true;
}
bool fenceSet(int i, const MissionItem& it) {
  if (i < 0 || i >= FENCE_MAX) return false;
  os::CriticalGuard g;
  fence.items[i] = it;
  fenceGen = fenceGen + 1;
  return true;
}
void fenceSetCount(int n) {
  os::CriticalGuard g;
  fence.count = (uint16_t)(n < 0 ? 0 : (n > FENCE_MAX ? FENCE_MAX : n));
  fenceGen = fenceGen + 1;
}
bool fenceLoad() {
  FenceStore tmp;
  if (!hal::storageLoad(hal::Slot::FENCE, &tmp, sizeof(tmp))) return false;
  if (tmp.count > FENCE_MAX || tmp.crc != fc::crc32(&tmp, offsetof(FenceStore, crc))) return false;
  os::CriticalGuard g;
  fence = tmp;
  fenceGen = fenceGen + 1;
  return true;
}
bool fenceSave() {
  if (bus::status.get().armed) { pending = pending | 4; return true; }
  pending = pending & ~4;
  FenceStore tmp;
  {
    os::CriticalGuard g;
    fence.version = 1;
    fence.crc = fc::crc32(&fence, offsetof(FenceStore, crc));
    tmp = fence;
  }
  return hal::storageSave(hal::Slot::FENCE, &tmp, sizeof(tmp));
}

bool fenceOutside(double homeLat, double homeLon, float n, float e) {
  // vertices converted to metres from home once per (fence, home) change
  static float vn[FENCE_MAX], ve[FENCE_MAX];
  static int nv = 0;
  static uint32_t cGen = 0;
  static double cLat = 1e9, cLon = 1e9;
  if (cGen != fenceGen || cLat != homeLat || cLon != homeLon) {
    cGen = fenceGen; cLat = homeLat; cLon = homeLon;
    nv = 0;
    double cl = cos(homeLat * 0.017453292519943295);
    for (int i = 0; i < fenceCount() && nv < FENCE_MAX; i++) {
      MissionItem it;
      if (!fenceGet(i, it) || it.cmd != CMD_FENCE_POLYGON_INCLUSION) continue;
      vn[nv] = (float)((it.lat * 1e-7 - homeLat) * 111319.49);
      ve[nv] = (float)((it.lon * 1e-7 - homeLon) * 111319.49 * cl);
      nv++;
    }
  }
  if (nv < 3) return false;
  bool inside = false;  // even-odd ray casting
  for (int i = 0, j = nv - 1; i < nv; j = i++) {
    if ((ve[i] > e) != (ve[j] > e) && n < (vn[j] - vn[i]) * (e - ve[i]) / (ve[j] - ve[i]) + vn[i]) inside = !inside;
  }
  return !inside;
}

void persistPending() {
  if (!pending || bus::status.get().armed) return;
  if (pending & 1) save();
  if (pending & 2) rallySave();
  if (pending & 4) fenceSave();
}

}  // namespace nav
