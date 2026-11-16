// AllQuad - waypoint mission storage. Item commands use MAVLink MAV_CMD numbers so
// missions planned in QGroundControl / Mission Planner map 1:1.
#pragma once
#include <stdint.h>

namespace nav {

enum MissionCmd : uint16_t {
  CMD_WAYPOINT = 16,      // fly to lat/lon/alt, hold param1 seconds
  CMD_LOITER_UNLIM = 17,  // fly to lat/lon/alt and hold there until the mode changes
  CMD_LOITER_TURNS = 18,  // orbit lat/lon: param1 turns, param3 radius (m, negative = counter-clockwise)
  CMD_LOITER_TIME = 19,   // like WAYPOINT, hold param1 seconds
  CMD_RTL = 20,           // return to home (and land if rth_land)
  CMD_LAND = 21,          // land at lat/lon (0,0 = here)
  CMD_TAKEOFF = 22,       // climb to alt above home
  CMD_CONDITION_YAW = 115,// param1 heading (deg), param4 0 absolute / 1 relative (param3 -1 = CCW)
  CMD_DO_JUMP = 177,      // param1 target item, param2 repeat count (-1 = forever)
  CMD_CHANGE_SPEED = 178, // param2 = new horizontal speed (m/s)
  CMD_DO_SET_ROI_LOC = 195, // point the nose at lat/lon while flying
  CMD_DO_SET_ROI_NONE = 197,
  CMD_DO_SET_ROI = 201,   // legacy ROI (lat/lon)
  CMD_FENCE_POLYGON_INCLUSION = 5001,  // geofence vertex (fence list only): stay inside the polygon
  CMD_RALLY_POINT = 5100, // rally point (rally list only)
};

struct MissionItem {
  uint16_t cmd;
  uint8_t frame;          // MAV_FRAME (stored for round-trip; alt is relative to home)
  uint8_t autocontinue;
  int32_t lat, lon;       // degE7
  float alt;              // m above home
  float param1, param2, param3, param4;
};

constexpr int MISSION_MAX = 120;  // iNav: 120 waypoints

struct MissionStore {
  uint16_t count;
  uint16_t version;
  MissionItem items[MISSION_MAX];
  uint32_t crc;
};

// thread-safe accessors (telemetry task writes, control task reads)
int count();
bool get(int i, MissionItem& out);
bool set(int i, const MissionItem& it);  // i < MISSION_MAX
void setCount(int n);
void clear();
uint32_t generation();  // changes whenever the mission is modified
bool load();
bool save();
bool supported(uint16_t cmd);

// ---- rally points (MAVLink mission_type 2): alternative RTH destinations
constexpr int RALLY_MAX = 10;
int rallyCount();
bool rallyGet(int i, MissionItem& out);
bool rallySet(int i, const MissionItem& it);
void rallySetCount(int n);
bool rallyLoad();
bool rallySave();
// ---- geofence polygon (MAVLink mission_type 1, inclusion vertices): the craft must stay inside
constexpr int FENCE_MAX = 16;
int fenceCount();
bool fenceGet(int i, MissionItem& out);
bool fenceSet(int i, const MissionItem& it);
void fenceSetCount(int n);
bool fenceLoad();
bool fenceSave();
// true when a polygon (>= 3 vertices) is stored and pos (NE metres from home) is outside it;
// homeLat / homeLon: home position (deg)
bool fenceOutside(double homeLat, double homeLon, float n, float e);

// saves postponed while armed are written once disarmed (called by the telemetry task)
void persistPending();

}  // namespace nav
