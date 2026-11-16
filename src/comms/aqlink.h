// AllQuad - configurator link over the USB / COM port ("AQ link").
//
// The same configurator page that the ESP32 serves over WiFi also runs on a laptop (Chrome / Edge,
// Web Serial API) and talks to the flight controller through its USB console - on every board,
// STM32 included. The page keeps its HTTP-style API; each request travels as one binary frame and
// is answered by the same functions as the web server (web/webapi.h).
//
// Request  (laptop -> FC):  A7 51 | seq | len16 | payload[len] | crc16
//          payload = "/api/route?query" 0x00 body
// Response (FC -> laptop):  A7 52 | seq | status16 | total32 | offset32 | len16 | data[len] | crc16
//          crc16 = CRC-16/CCITT-FALSE over everything after the two magic bytes, little endian fields.
//
// Bodies larger than one frame are paged: the client adds o=<offset> and repeats until
// offset + len == total (parameters, backup, mission, blackbox). Uploads (restore) are sent in
// pieces with o=<offset>&n=<total>. Everything is non-blocking: the reply is sent from a buffer as
// the port has room, so the telemetry task never waits for a slow UART.
//
// Shares the console with the CLI, MSP and MAVLink: the frames are binary (the CLI ignores lines
// while a frame is active) and all other parsers check their own checksums. A connected
// configurator blocks arming like the CLI does (props on a bench). No login: USB means the craft
// is in your hands (like Betaflight / INAV configurators); WiFi credentials can be reset this way.
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "../hal/hal.h"

namespace comms {

class AqLink {
 public:
  static constexpr size_t RX_MAX = 1200;   // request payload (path + body piece)
  static constexpr size_t DATA_MAX = 2048; // response data per frame (a full status fits in one)

  void begin(hal::Port p) { port_ = p; }
  // feed every received byte; returns true while the byte belongs to an AQ frame
  bool feed(uint8_t b, uint32_t nowMs);
  // send pending reply bytes, run deferred actions (reboot, WiFi restart)
  void update(uint32_t nowMs);
  // a configurator is talking: keep the CLI quiet, block arming
  bool active(uint32_t nowMs) const { return state_ != 0 || txPos_ < txLen_ || (lastMs_ && nowMs - lastMs_ < 3000); }
  uint32_t frames() const { return frames_; }
  uint32_t crcErrors() const { return crcErrors_; }

  static uint16_t crc16(const uint8_t* d, size_t n, uint16_t crc = 0xFFFF);

 private:
  void handle(uint32_t nowMs);
  void reply(int status, uint32_t total, uint32_t off, const uint8_t* data, size_t len);
  void replyMsg(int status, const char* msg);

  hal::Port port_ = hal::Port::CONSOLE;
  // receive state machine
  uint8_t state_ = 0;
  uint8_t hdr_[3];
  uint16_t len_ = 0, got_ = 0;
  uint8_t crcBuf_[2];
  uint8_t rx_[RX_MAX + 1];
  uint32_t frameStartMs_ = 0, lastMs_ = 0;
  uint8_t seq_ = 0;
  // transmit
  uint8_t tx_[DATA_MAX + 20];
  size_t txLen_ = 0, txPos_ = 0;
  // restore upload being assembled
  char* upload_ = nullptr;
  uint32_t uploadLen_ = 0, uploadTotal_ = 0;
  // deferred actions after the reply has left
  bool rebootPending_ = false, bootloaderPending_ = false, wifiRestart_ = false;
  // output of the last CLI line (read in pages)
  static constexpr size_t CLI_MAX = 16384;
  char* cliBuf_ = nullptr;
  size_t cliLen_ = 0;
  uint32_t frames_ = 0, crcErrors_ = 0;
};

}  // namespace comms
