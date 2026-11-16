// AllQuad - analog video transmitter control.
//
//   SmartAudio (TBS, most analog VTX): 4800 baud 8N2 on the VTX "SA" wire (FC UART TX),
//     frame 00 AA 55 cmd*2+1 len payload crc8(0xD5). Power as dBm (SmartAudio 2.1).
//   Tramp (ImmersionRC, many others): 9600 baud, 16-byte frames 0F cmd data.. sum 00.
//   Digital systems (DJI / Walksnail / HDZero) read and set the same settings over MSP
//   (MSP_VTX_CONFIG / MSP_SET_VTX_CONFIG on the DisplayPort port).
//
// Settings live in the config (vtx_band / vtx_channel / vtx_freq / vtx_power / vtx_pit /
// vtx_low_disarm). The driver sends only what changed, and once more a second later (a VTX that
// was still booting misses the first frame). Only the TX line is used: no read-back.
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "../config/config.h"
#include "../hal/hal.h"

namespace vtx {

constexpr int BANDS = 6, CHANNELS = 8, POWERS = 5;
extern const char* const BAND_NAMES[BANDS];       // "A", "B", "E", "F", "R", "L"
extern const uint16_t FREQ[BANDS][CHANNELS];      // MHz
extern const uint16_t POWER_MW[POWERS];           // 25, 100, 200, 400, 600
extern const uint8_t POWER_DBM[POWERS];           // 14, 20, 23, 26, 28

// frequency from the config (vtx_freq overrides band/channel; band 7 = user band of the VTX table)
uint16_t frequency(const fc::Config& c);
// power levels: the built-in table, or the custom VTX table (vtxt_powers > 0)
int powerCount(const fc::Config& c);
uint16_t powerMw(const fc::Config& c, int level);       // level 1..powerCount
uint8_t powerSaValue(const fc::Config& c, int level);   // SmartAudio: dBm (2.1) or level index

// frame builders (exposed for the tests)
size_t saFrame(uint8_t* out, uint8_t cmd, const uint8_t* payload, uint8_t len);
uint8_t saCrc8(const uint8_t* d, size_t n);
size_t saSetFrequency(uint8_t* out, uint16_t mhz);
size_t saSetPowerDbm(uint8_t* out, uint8_t dbm);
size_t saSetPowerIndex(uint8_t* out, uint8_t index);
size_t saSetPit(uint8_t* out, bool pit);
void trampFrame(uint8_t out[16], char cmd, uint16_t value);

enum class Type : uint8_t { NONE, SMARTAUDIO, TRAMP };

class Driver {
 public:
  void begin(Type t, hal::Port p);
  void update(const fc::Config& c, bool armed, uint32_t nowMs);
  Type type() const { return type_; }
  uint16_t sentFreq() const { return freq_; }
  uint16_t sentPowerMw() const { return powerMw_; }
  bool pit() const { return pit_ == 1; }
  uint32_t frames() const { return frames_; }

 private:
  void sendFrame(int which);  // 0 frequency, 1 power, 2 pit / active
  Type type_ = Type::NONE;
  uint8_t saValue_ = 14;
  bool saIndex_ = false;
  hal::Port port_ = hal::Port::SERIAL1;
  uint16_t freq_ = 0, powerMw_ = 0;
  int8_t pit_ = -1;
  uint8_t pending_ = 0;
  uint32_t nextMs_ = 0, repeatMs_ = 0, frames_ = 0;
  bool repeated_ = true;
};

}  // namespace vtx
