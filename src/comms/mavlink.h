// AllQuad - MAVLink v2 (receives v1 too): QGroundControl / Mission Planner / companion computers.
// Implemented: heartbeat & telemetry streams, PARAM protocol (all firmware parameters),
// MISSION protocol (upload/download/clear, MISSION_ITEM_INT and legacy MISSION_ITEM),
// commands: arm/disarm, RTL, land, set mode, mission start, calibrations, reboot,
// request message (AUTOPILOT_VERSION, HOME_POSITION).
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "../hal/hal.h"

namespace comms {

struct MavFrame {
  uint8_t sysid, compid, seq;
  uint32_t msgid;
  uint8_t len;
  uint8_t payload[255];
};

uint16_t mavCrcAccumulate(uint8_t b, uint16_t crc);
int mavCrcExtra(uint32_t msgid);  // -1 if unknown
// Packs a MAVLink 2 frame (trailing zero bytes of the payload are truncated). Returns length.
size_t mavPack(uint8_t* out, uint32_t msgid, const uint8_t* payload, uint8_t len, uint8_t sysid, uint8_t compid,
               uint8_t seq);

class MavParser {
 public:
  // returns true when a complete frame with a valid CRC for a known message is available
  bool feed(uint8_t b, MavFrame& out);
  uint32_t errors() const { return errors_; }

 private:
  uint8_t buf_[280];
  uint16_t pos_ = 0, need_ = 0;
  bool v2_ = false;
  uint32_t errors_ = 0;
};

class MavlinkLink {
 public:
  // streamOnlyWhenConnected: emit telemetry only after a GCS frame arrived (USB console sharing)
  void begin(hal::Port p, bool streamOnlyWhenConnected = false);
  // readPort = false when another component reads the port and calls feed()
  void update(uint32_t nowMs, bool readPort = true);
  void feed(uint8_t b, uint32_t nowMs) {
    MavFrame f;
    if (parser_.feed(b, f)) handle(f, nowMs);
  }
  void handle(const MavFrame& f, uint32_t nowMs);  // public for tests
  bool seenRecently(uint32_t nowMs) const { return lastRxMs_ && nowMs - lastRxMs_ < 3000; }

 private:
  void send(uint32_t msgid, const uint8_t* payload, uint8_t len);
  void sendHeartbeat();
  void sendSysStatus();
  void sendAttitude(uint32_t nowMs);
  void sendPosition(uint32_t nowMs);
  void sendGpsRaw(uint32_t nowMs);
  void sendVfrHud();
  void sendRcChannels(uint32_t nowMs);
  void sendHome();
  void sendStatusText(const char* text, uint8_t severity);
  void sendParam(int index);
  void sendCommandAck(uint16_t cmd, uint8_t result);
  void sendMissionAck(uint8_t type, uint8_t missionType = 0);
  void sendMissionCount(uint8_t missionType);
  void sendMissionRequest(uint16_t seq);
  void sendMissionItem(uint16_t seq, uint8_t missionType = 0);
  void sendAutopilotVersion();
  void handleCommand(uint16_t cmd, const float p[7], uint32_t nowMs);
  bool forUs(uint8_t targetSys) const;

  hal::Port port_ = hal::Port::CONSOLE;
  bool streamOnlyWhenConnected_ = false;
  MavParser parser_;
  uint8_t txSeq_ = 0;
  uint32_t lastRxMs_ = 0;
  uint32_t tHb_ = 0, tSys_ = 0, tAtt_ = 0, tPos_ = 0, tGps_ = 0, tHud_ = 0, tRc_ = 0, tHome_ = 0, tMc_ = 0;
  int paramStreamIdx_ = -1;
  uint32_t msgSeq_ = 0;
  uint16_t lastReached_ = 0xFFFF;
  // mission upload
  int upCount_ = -1, upNext_ = 0, upRetries_ = 0;
  uint32_t upLastMs_ = 0;
  uint8_t gcsSys_ = 255, gcsComp_ = 190;
  bool upUseInt_ = true;
  uint8_t upType_ = 0;  // 0 mission, 1 fence polygon, 2 rally points
  bool paramDirty_ = false;
  uint32_t paramDirtyMs_ = 0;
};

}  // namespace comms
