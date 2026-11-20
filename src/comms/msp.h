// AllQuad - MSP v1/v2 (Betaflight / iNav MultiWii Serial Protocol)
//   * answers telemetry requests (status, attitude, altitude, GPS, analog, RC, motors, PIDs)
//   * accepts MSP2 sensor messages: rangefinder (0x1F01) and optical flow (0x1F02),
//     e.g. Matek 3901-L0X / MicoAir MTF-01 modules (iNav protocol)
//   * DisplayPort (MSP 182) output for digital FPV OSD (DJI O3 / Walksnail / HDZero)
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "../hal/hal.h"

namespace comms {

struct MspFrame {
  bool v2;
  char dir;          // '<' request, '>' response, '!' error
  uint16_t cmd;
  uint16_t len;
  uint8_t payload[256];
};

uint8_t crc8DvbS2(uint8_t crc, uint8_t b);
size_t mspPack(uint8_t* out, bool v2, char dir, uint16_t cmd, const uint8_t* payload, uint16_t len);

class MspParser {
 public:
  bool feed(uint8_t b, MspFrame& out);

 private:
  enum State : uint8_t { IDLE, HDR_M_X, DIR, V1_LEN, V1_CMD, V2_FLAG, V2_CMD1, V2_CMD2, V2_LEN1, V2_LEN2, PAYLOAD, CRC };
  State st_ = IDLE;
  MspFrame f_{};
  uint16_t idx_ = 0;
  uint8_t chk_ = 0;
};

class FourWay;

class MspLink {
 public:
  void begin(hal::Port p) { port_ = p; }
  // console only: MSP_SET_PASSTHROUGH hands the port to the ESC 4-way interface
  void setFourWay(FourWay* f) { fourWay_ = f; }
  // reported in MSP_VTX_CONFIG (Betaflight: 3 SmartAudio, 4 Tramp, 0xFF unknown / MSP)
  void setVtxType(uint8_t t) { vtxType_ = t; }
  void update(uint32_t nowMs);
  void feed(uint8_t b, uint32_t nowMs) {
    MspFrame f;
    if (parser_.feed(b, f)) handle(f, nowMs);
  }
  void handle(const MspFrame& f, uint32_t nowMs);  // public for tests
  hal::Port port() const { return port_; }
  bool seenRecently(uint32_t nowMs) const { return lastRxMs_ && nowMs - lastRxMs_ < 3000; }
  // DisplayPort (OSD) output
  void dpHeartbeat();
  void dpClear();
  bool dpWrite(uint8_t row, uint8_t col, const char* text, uint8_t attr = 0);  // false: TX buffer full
  void dpDraw();
  void dpOptions(uint8_t font, uint8_t resolution);
  // DisplayPort link: answer as Betaflight (FC variant BTFL) so DJI / Walksnail / HDZero load the
  // Betaflight font and read the arm flag from MSP_STATUS (the goggles leave low-power mode on arm)
  void setDisplayPort(bool on) { displayPort_ = on; }
  // answers go to this function instead of the port (MSP over CRSF); error = not supported / refused
  using ReplyHook = void (*)(void* ctx, const MspFrame& req, const uint8_t* payload, uint16_t len, bool error);
  void setReplyHook(ReplyHook h, void* ctx) { hook_ = h; hookCtx_ = ctx; }
  void canvasRequest(int& cols, int& rows) const { cols = canvasCols_; rows = canvasRows_; }

 private:
  void reply(const MspFrame& req, const uint8_t* payload, uint16_t len);
  hal::Port port_ = hal::Port::CONSOLE;
  MspParser parser_;
  uint32_t lastRxMs_ = 0, lastFlowMs_ = 0;
  FourWay* fourWay_ = nullptr;
  uint8_t vtxType_ = 0xFF;
  bool displayPort_ = false;
  uint8_t canvasCols_ = 0, canvasRows_ = 0;
  ReplyHook hook_ = nullptr;
  void* hookCtx_ = nullptr;
  void error(const MspFrame& req);
  // sensor modules on an MSP port: Micolink (MicoAir MTF-01 / 01P / 02 factory protocol) is decoded too,
  // MAVLink is only recognised (the module must be switched to MSP or Micolink)
  void sniff(uint8_t b, uint32_t now);
  void micolink(const uint8_t* f, uint32_t now);
  void flowSample(float rx, float ry, uint8_t quality, uint32_t now);
  uint8_t mico_[26];  // header + the 20-byte range message (longer frames are summed, not kept: RAM on the F405)
  uint8_t micoN_ = 0, micoSum_ = 0;
  uint8_t win_[6] = {0, 0, 0, 0, 0, 0};
  uint32_t rxBytes_ = 0, mspFrames_ = 0, micoFrames_ = 0, mavHints_ = 0;
};

}  // namespace comms
