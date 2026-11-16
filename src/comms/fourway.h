// AllQuad - ESC passthrough: BLHeli "4-way interface" (Betaflight compatible).
//
// esc-configurator.com (Bluejay, BLHeli_S, AM32, BLHeli_32) or BLHeliSuite connect to the flight
// controller over USB with MSP and send MSP_SET_PASSTHROUGH. From then on the console speaks the
// 4-way protocol and the FC relays every command to the ESC bootloaders over the motor wires
// (one-wire serial, 19200 baud, bit-banged by the HAL). Leaving: cmd_InterfaceExit, or nothing
// received for 60 s. Motors are stopped the whole time; arming is impossible (MSP / configurator
// active, and the fast loop leaves the motor pins alone).
//
// PC -> FC:  2F cmd addrH addrL len(0 = 256) params crc16hi crc16lo     (CRC-16/XMODEM)
// FC -> PC:  2E cmd addrH addrL len params ack crc16hi crc16lo
// FC <-> ESC bootloader (BLHeli BLB): commands + CRC-16 (0xA001, little endian), ack 0x30 = success
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "../hal/hal.h"

namespace comms {

class FourWay {
 public:
  enum Mode : uint8_t { imC2 = 0, imSIL_BLB = 1, imATM_BLB = 2, imSK = 3, imARM_BLB = 4 };

  void begin(hal::Port p) { port_ = p; }
  uint8_t start(uint32_t nowMs);          // MSP_SET_PASSTHROUGH: returns the number of ESCs
  bool active() const { return active_; }
  void feed(uint8_t b, uint32_t nowMs);   // console bytes while active
  void update(uint32_t nowMs);            // inactivity timeout
  void stop();

  static uint16_t crcXmodem(uint16_t crc, uint8_t b);
  static uint16_t crcBlb(uint16_t crc, uint8_t b);  // ESC bootloader CRC (poly 0xA001, reflected)

 private:
  void process();
  // ESC bootloader (BLB) layer
  bool blConnect(uint8_t info[4]);
  void blSend(const uint8_t* d, size_t n, bool withCrc);
  bool blRead(uint8_t* d, size_t n);       // data + CRC + ack (connected)
  int blAck(uint32_t timeoutMs);
  bool blSetAddress(uint16_t addr);
  bool blSetBuffer(const uint8_t* d, size_t n);
  bool blReadCmd(uint8_t cmd, uint16_t addr, uint8_t* d, size_t n);
  bool blWriteCmd(uint8_t cmd, uint16_t addr, const uint8_t* d, size_t n, uint32_t timeoutMs);
  bool blKeepAlive();
  bool blPageErase(uint16_t addr);

  hal::Port port_ = hal::Port::CONSOLE;
  bool active_ = false;
  uint32_t lastMs_ = 0;
  // incoming frame
  uint8_t in_[5 + 256 + 2];
  size_t got_ = 0;
  // current ESC
  int esc_ = 0;
  bool connected_ = false;
  uint8_t mode_ = imSIL_BLB;
};

}  // namespace comms
