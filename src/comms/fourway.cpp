#include "../common/size_opt.h"  // -Os on ESP32 (not flight critical)
#include "fourway.h"

#include <string.h>

#include "../bus/bus.h"

namespace comms {

// 4-way interface commands / acks (Betaflight serial_4way.h)
enum : uint8_t {
  cmd_Remote_Escape = 0x2E, cmd_Local_Escape = 0x2F,
  cmd_InterfaceTestAlive = 0x30, cmd_ProtocolGetVersion = 0x31, cmd_InterfaceGetName = 0x32,
  cmd_InterfaceGetVersion = 0x33, cmd_InterfaceExit = 0x34, cmd_DeviceReset = 0x35,
  cmd_DeviceInitFlash = 0x37, cmd_DeviceEraseAll = 0x38, cmd_DevicePageErase = 0x39, cmd_DeviceRead = 0x3A,
  cmd_DeviceWrite = 0x3B, cmd_DeviceC2CK_LOW = 0x3C, cmd_DeviceReadEEprom = 0x3D, cmd_DeviceWriteEEprom = 0x3E,
  cmd_InterfaceSetMode = 0x3F, cmd_DeviceVerify = 0x40,
};
enum : uint8_t {
  ACK_OK = 0x00, ACK_I_INVALID_CMD = 0x02, ACK_I_INVALID_CRC = 0x03, ACK_I_VERIFY_ERROR = 0x04,
  ACK_D_INVALID_COMMAND = 0x05, ACK_I_INVALID_CHANNEL = 0x08, ACK_I_INVALID_PARAM = 0x09,
  ACK_D_GENERAL_ERROR = 0x0F,
};
static constexpr uint8_t PROTOCOL_VERSION = 108;
static constexpr uint8_t IF_VERSION_HI = 200, IF_VERSION_LO = 6;  // "20.0.06"
static constexpr int ESC_COUNT = 4;

// ESC bootloader
enum : uint8_t {
  brSUCCESS = 0x30, brERRORVERIFY = 0xC0, brERRORCOMMAND = 0xC1, brERRORCRC = 0xC2, brNONE = 0xFF,
  CMD_RUN = 0x00, CMD_PROG_FLASH = 0x01, CMD_ERASE_FLASH = 0x02, CMD_READ_FLASH_SIL = 0x03,
  CMD_VERIFY_FLASH_ARM = 0x04, CMD_READ_EEPROM = 0x04, CMD_PROG_EEPROM = 0x05, CMD_READ_FLASH_ATM = 0x07,
  CMD_KEEP_ALIVE = 0xFD, CMD_SET_BUFFER = 0xFE, CMD_SET_ADDRESS = 0xFF,
};

uint16_t FourWay::crcXmodem(uint16_t crc, uint8_t b) {
  crc ^= (uint16_t)b << 8;
  for (int i = 0; i < 8; i++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  return crc;
}
uint16_t FourWay::crcBlb(uint16_t crc, uint8_t b) {
  for (int i = 0; i < 8; i++) {
    crc = ((b ^ crc) & 1) ? (uint16_t)((crc >> 1) ^ 0xA001) : (uint16_t)(crc >> 1);
    b >>= 1;
  }
  return crc;
}

uint8_t FourWay::start(uint32_t now) {
  hal::escSerialBegin();
  bus::escPassthrough = true;
  active_ = true;
  connected_ = false;
  got_ = 0;
  lastMs_ = now;
  return ESC_COUNT;
}

void FourWay::stop() {
  if (!active_) return;
  active_ = false;
  connected_ = false;
  hal::escSerialEnd();
  bus::escPassthrough = false;
}

void FourWay::update(uint32_t now) {
  if (active_ && now - lastMs_ > 60000) stop();  // configurator closed without "exit"
}

void FourWay::feed(uint8_t b, uint32_t now) {
  if (!active_) return;
  if (got_ == 0 && b != cmd_Local_Escape) return;  // resynchronise on the escape byte
  in_[got_++] = b;
  if (got_ < 5) return;
  size_t len = in_[4] ? in_[4] : 256;
  if (got_ < 5 + len + 2) return;
  lastMs_ = now;
  process();
  got_ = 0;
}

// ---------------------------------------------------------------- ESC bootloader layer
void FourWay::blSend(const uint8_t* d, size_t n, bool withCrc) {
  uint8_t buf[260 + 2];
  uint16_t crc = 0;
  for (size_t i = 0; i < n; i++) { buf[i] = d[i]; crc = crcBlb(crc, d[i]); }
  if (withCrc) { buf[n++] = (uint8_t)crc; buf[n++] = (uint8_t)(crc >> 8); }
  hal::escSerialWrite(esc_, buf, n);
}

int FourWay::blAck(uint32_t timeoutMs) { return hal::escSerialRead(esc_, timeoutMs); }

bool FourWay::blRead(uint8_t* d, size_t n) {
  uint16_t crc = 0;
  for (size_t i = 0; i < n; i++) {
    int c = hal::escSerialRead(esc_, 2);
    if (c < 0) return false;
    d[i] = (uint8_t)c;
    crc = crcBlb(crc, d[i]);
  }
  int lo = hal::escSerialRead(esc_, 2), hi = hal::escSerialRead(esc_, 2), ack = hal::escSerialRead(esc_, 2);
  if (lo < 0 || hi < 0 || ack < 0) return false;
  return (uint16_t)(lo | hi << 8) == crc && ack == brSUCCESS;
}

bool FourWay::blConnect(uint8_t info[4]) {
  static const uint8_t bootInit[] = {0, 0, 0, 0, 0, 0, 0, 0, 0x0D, 'B', 'L', 'H', 'e', 'l', 'i', 0xF4, 0x7D};
  for (int attempt = 0; attempt < 3; attempt++) {
    blSend(bootInit, sizeof(bootInit), false);
    // answer: "471x" signature-hi signature-lo version pages ack (no CRC before connecting)
    uint8_t r[9];
    bool ok = true;
    for (int i = 0; i < 9 && ok; i++) {
      int c = hal::escSerialRead(esc_, 2);
      if (c < 0) ok = false;
      else r[i] = (uint8_t)c;
    }
    if (!ok || r[0] != '4' || r[1] != '7' || r[2] != '1' || r[8] != brSUCCESS) continue;
    info[0] = r[5];  // signature low
    info[1] = r[4];  // signature high
    info[2] = r[3];  // boot message byte
    uint16_t sig = (uint16_t)(r[4] << 8 | r[5]);
    switch (sig) {
      case 0xF310: case 0xF330: case 0xF410: case 0xF390: case 0xF850: case 0xE8B1: case 0xE8B2:
        mode_ = imSIL_BLB;  // BLHeli_S / Bluejay (SiLabs EFM8)
        break;
      case 0x9307: case 0x930A: case 0x930F: case 0x940B:
        mode_ = imATM_BLB;  // Atmel BLHeli
        break;
      default:
        mode_ = imARM_BLB;  // BLHeli_32 / AM32 (many MCU signatures)
    }
    info[3] = mode_;
    connected_ = true;
    return true;
  }
  return false;
}

bool FourWay::blSetAddress(uint16_t addr) {
  if (addr == 0xFFFF) return true;
  uint8_t c[4] = {CMD_SET_ADDRESS, 0, (uint8_t)(addr >> 8), (uint8_t)addr};
  blSend(c, 4, true);
  return blAck(4) == brSUCCESS;
}

bool FourWay::blSetBuffer(const uint8_t* d, size_t n) {
  uint8_t c[4] = {CMD_SET_BUFFER, 0, (uint8_t)(n == 256 ? 1 : 0), (uint8_t)n};
  blSend(c, 4, true);
  if (blAck(4) != -1) return false;  // the bootloader stays silent here
  blSend(d, n, true);
  return blAck(80) == brSUCCESS;
}

bool FourWay::blReadCmd(uint8_t cmd, uint16_t addr, uint8_t* d, size_t n) {
  if (!blSetAddress(addr)) return false;
  uint8_t c[2] = {cmd, (uint8_t)n};
  blSend(c, 2, true);
  return blRead(d, n);
}

bool FourWay::blWriteCmd(uint8_t cmd, uint16_t addr, const uint8_t* d, size_t n, uint32_t timeoutMs) {
  if (!blSetAddress(addr) || !blSetBuffer(d, n)) return false;
  uint8_t c[2] = {cmd, 1};
  blSend(c, 2, true);
  return blAck(timeoutMs) == brSUCCESS;
}

bool FourWay::blKeepAlive() {
  uint8_t c[2] = {CMD_KEEP_ALIVE, 0};
  blSend(c, 2, true);
  return blAck(4) == brERRORCOMMAND;  // "unknown command" = alive
}

bool FourWay::blPageErase(uint16_t addr) {
  if (!blSetAddress(addr)) return false;
  uint8_t c[2] = {CMD_ERASE_FLASH, 1};
  blSend(c, 2, true);
  return blAck(3000) == brSUCCESS;
}

// ---------------------------------------------------------------- 4-way frame
void FourWay::process() {
  uint8_t cmd = in_[1];
  uint16_t addr = (uint16_t)(in_[2] << 8 | in_[3]);
  size_t inLen = in_[4] ? in_[4] : 256;
  uint8_t* param = in_ + 5;
  uint16_t crc = 0;
  for (size_t i = 0; i < 5 + inLen; i++) crc = crcXmodem(crc, in_[i]);
  uint16_t rxCrc = (uint16_t)(in_[5 + inLen] << 8 | in_[5 + inLen + 1]);

  static uint8_t out[256];
  size_t outLen = 1;
  out[0] = param[0];
  uint8_t ack = ACK_OK;
  bool exitAfter = false;
  if (crc != rxCrc) {
    ack = ACK_I_INVALID_CRC;
  } else {
    switch (cmd) {
      case cmd_InterfaceTestAlive:
        if (connected_ && !blKeepAlive()) { ack = ACK_D_GENERAL_ERROR; connected_ = false; }
        break;
      case cmd_ProtocolGetVersion: out[0] = PROTOCOL_VERSION; break;
      case cmd_InterfaceGetName: memcpy(out, "m4wFCIntf", 9); outLen = 9; break;
      case cmd_InterfaceGetVersion: out[0] = IF_VERSION_HI; out[1] = IF_VERSION_LO; outLen = 2; break;
      case cmd_InterfaceExit: exitAfter = true; break;
      case cmd_InterfaceSetMode:
        if (param[0] >= imSIL_BLB && param[0] <= imARM_BLB && param[0] != imSK) mode_ = param[0];
        else ack = ACK_I_INVALID_PARAM;
        break;
      case cmd_DeviceReset:
        if (param[0] >= ESC_COUNT) { ack = ACK_I_INVALID_CHANNEL; break; }
        esc_ = param[0];
        { uint8_t c[2] = {CMD_RUN, 0}; blSend(c, 2, true); }
        connected_ = false;
        break;
      case cmd_DeviceInitFlash:
        if (param[0] >= ESC_COUNT) { ack = ACK_I_INVALID_CHANNEL; break; }
        esc_ = param[0];
        connected_ = false;
        outLen = 4;
        if (!blConnect(out)) { ack = ACK_D_GENERAL_ERROR; memset(out, 0, 4); }
        break;
      case cmd_DeviceRead: {
        size_t n = param[0] ? param[0] : 256;
        uint8_t rc = mode_ == imATM_BLB ? CMD_READ_FLASH_ATM : CMD_READ_FLASH_SIL;
        if (!connected_ || !blReadCmd(rc, addr, out, n)) { ack = ACK_D_GENERAL_ERROR; outLen = 1; out[0] = 0; }
        else outLen = n;
        break;
      }
      case cmd_DeviceReadEEprom: {
        size_t n = param[0] ? param[0] : 256;
        if (mode_ != imATM_BLB) { ack = ACK_I_INVALID_CMD; break; }
        if (!connected_ || !blReadCmd(CMD_READ_EEPROM, addr, out, n)) { ack = ACK_D_GENERAL_ERROR; outLen = 1; out[0] = 0; }
        else outLen = n;
        break;
      }
      case cmd_DeviceWrite:
        if (!connected_ || !blWriteCmd(CMD_PROG_FLASH, addr, param, inLen, 500)) ack = ACK_D_GENERAL_ERROR;
        break;
      case cmd_DeviceWriteEEprom:
        if (mode_ != imATM_BLB) { ack = ACK_D_INVALID_COMMAND; break; }
        if (!connected_ || !blWriteCmd(CMD_PROG_EEPROM, addr, param, inLen, 3000)) ack = ACK_D_GENERAL_ERROR;
        break;
      case cmd_DevicePageErase: {
        if (mode_ != imSIL_BLB && mode_ != imARM_BLB) { ack = ACK_I_INVALID_CMD; break; }
        uint16_t a = (uint16_t)(mode_ == imARM_BLB ? param[0] << 10 : param[0] << 9);  // 1 kB / 512 B pages
        if (!connected_ || !blPageErase(a)) ack = ACK_D_GENERAL_ERROR;
        break;
      }
      case cmd_DeviceVerify: {
        if (mode_ != imARM_BLB) { ack = ACK_I_INVALID_CMD; break; }
        if (!connected_ || !blSetAddress(addr) || !blSetBuffer(param, inLen)) { ack = ACK_D_GENERAL_ERROR; break; }
        uint8_t c[2] = {CMD_VERIFY_FLASH_ARM, 1};
        blSend(c, 2, true);
        int r = blAck(80);
        ack = r == brSUCCESS ? ACK_OK : r == brERRORVERIFY ? ACK_I_VERIFY_ERROR : ACK_D_GENERAL_ERROR;
        break;
      }
      default:
        ack = ACK_I_INVALID_CMD;  // C2 / SK programming and erase-all are not supported
    }
  }
  // reply
  static uint8_t r[5 + 256 + 3];
  size_t k = 0;
  r[k++] = cmd_Remote_Escape;
  r[k++] = cmd;
  r[k++] = (uint8_t)(addr >> 8);
  r[k++] = (uint8_t)addr;
  r[k++] = (uint8_t)(outLen == 256 ? 0 : outLen);
  memcpy(r + k, out, outLen);
  k += outLen;
  r[k++] = ack;
  uint16_t oc = 0;
  for (size_t i = 0; i < k; i++) oc = crcXmodem(oc, r[i]);
  r[k++] = (uint8_t)(oc >> 8);
  r[k++] = (uint8_t)oc;
  hal::portWrite(port_, r, k);
  if (exitAfter) stop();
}

}  // namespace comms
