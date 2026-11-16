#include "../common/size_opt.h"  // -Os on ESP32 (not flight critical)
#include "vtx.h"

#include <string.h>

#include "../bus/bus.h"

namespace vtx {

const char* const BAND_NAMES[BANDS] = {"A", "B", "E", "F", "R", "L"};
const uint16_t FREQ[BANDS][CHANNELS] = {
    {5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725},  // A (Boscam A)
    {5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866},  // B
    {5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945},  // E
    {5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880},  // F (Fatshark / ImmersionRC)
    {5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917},  // R (Raceband)
    {5362, 5399, 5436, 5473, 5510, 5547, 5584, 5621},  // L (Lowband)
};
const uint16_t POWER_MW[POWERS] = {25, 100, 200, 400, 600};
const uint8_t POWER_DBM[POWERS] = {14, 20, 23, 26, 28};

int powerCount(const fc::Config& c) { return c.vtxt_powers >= 1 && c.vtxt_powers <= 8 ? c.vtxt_powers : POWERS; }
uint16_t powerMw(const fc::Config& c, int level) {
  int n = powerCount(c);
  int i = level >= 1 && level <= n ? level - 1 : 0;
  return c.vtxt_powers ? c.vtxt_mw[i] : POWER_MW[i];
}
uint8_t powerSaValue(const fc::Config& c, int level) {
  int n = powerCount(c);
  int i = level >= 1 && level <= n ? level - 1 : 0;
  if (!c.vtxt_powers) return POWER_DBM[i];
  return c.vtxt_val[i];
}

uint16_t frequency(const fc::Config& c) {
  if (c.vtx_freq >= 5000 && c.vtx_freq <= 6000) return c.vtx_freq;
  if (c.vtx_band == 7) {
    int ch = c.vtx_channel >= 1 && c.vtx_channel <= CHANNELS ? c.vtx_channel - 1 : 0;
    if (c.vtxt_band[ch] >= 5000 && c.vtxt_band[ch] <= 6000) return c.vtxt_band[ch];
  }
  int b = c.vtx_band >= 1 && c.vtx_band <= BANDS ? c.vtx_band - 1 : 4;
  int ch = c.vtx_channel >= 1 && c.vtx_channel <= CHANNELS ? c.vtx_channel - 1 : 0;
  return FREQ[b][ch];
}

// ---------------------------------------------------------------- SmartAudio
enum : uint8_t { SA_GET_SETTINGS = 0x01, SA_SET_POWER = 0x02, SA_SET_CHANNEL = 0x03, SA_SET_FREQ = 0x04, SA_SET_MODE = 0x05 };

uint8_t saCrc8(const uint8_t* d, size_t n) {
  uint8_t crc = 0;
  for (size_t i = 0; i < n; i++) {
    crc ^= d[i];
    for (int b = 0; b < 8; b++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
  }
  return crc;
}

size_t saFrame(uint8_t* o, uint8_t cmd, const uint8_t* p, uint8_t len) {
  size_t n = 0;
  o[n++] = 0x00;  // pulls the idle-low line high->low once before the sync (as Betaflight does)
  size_t start = n;
  o[n++] = 0xAA;
  o[n++] = 0x55;
  o[n++] = (uint8_t)((cmd << 1) | 1);
  o[n++] = len;
  for (uint8_t i = 0; i < len; i++) o[n++] = p[i];
  o[n] = saCrc8(o + start, n - start);
  n++;
  o[n++] = 0x00;
  return n;
}
size_t saSetFrequency(uint8_t* o, uint16_t mhz) {
  uint8_t p[2] = {(uint8_t)(mhz >> 8), (uint8_t)mhz};
  return saFrame(o, SA_SET_FREQ, p, 2);
}
size_t saSetPowerIndex(uint8_t* o, uint8_t index) {
  uint8_t p = index;  // SmartAudio 1.0 / 2.0: power level index (0-based)
  return saFrame(o, SA_SET_POWER, &p, 1);
}
size_t saSetPowerDbm(uint8_t* o, uint8_t dbm) {
  uint8_t p = (uint8_t)(dbm | 0x80);  // SmartAudio 2.1: power in dBm
  return saFrame(o, SA_SET_POWER, &p, 1);
}
size_t saSetPit(uint8_t* o, bool pit) {
  // mode bits: 0 pit (in-range), 2 leave pit, 3 unlocked
  uint8_t p = pit ? 0x09 : 0x0C;
  return saFrame(o, SA_SET_MODE, &p, 1);
}

// ---------------------------------------------------------------- Tramp
void trampFrame(uint8_t o[16], char cmd, uint16_t v) {
  memset(o, 0, 16);
  o[0] = 0x0F;
  o[1] = (uint8_t)cmd;
  o[2] = (uint8_t)v;
  o[3] = (uint8_t)(v >> 8);
  uint8_t sum = 0;
  for (int i = 1; i < 14; i++) sum = (uint8_t)(sum + o[i]);
  o[14] = sum;
}

// ---------------------------------------------------------------- driver
void Driver::begin(Type t, hal::Port p) {
  type_ = t;
  port_ = p;
  if (t == Type::SMARTAUDIO) hal::portOpen(p, 4800, hal::UartFormat::F8N2);
  else if (t == Type::TRAMP) hal::portOpen(p, 9600);
  freq_ = powerMw_ = 0;
  pit_ = -1;
}

void Driver::update(const fc::Config& c, bool armed, uint32_t now) {
  if (type_ == Type::NONE) return;
  uint16_t f = frequency(c);
  int want = bus::logicVtxPower ? bus::logicVtxPower : c.vtx_power;  // logic condition override
  int level = want >= 1 && want <= powerCount(c) ? want : 1;
  if (!armed && c.vtx_low_disarmed) level = 1;
  uint16_t mw = powerMw(c, level);
  saValue_ = powerSaValue(c, level);
  saIndex_ = c.vtxt_powers && c.vtxt_sa_index;
  int8_t pit = (!armed && c.vtx_pit) ? 1 : 0;  // pit mode is for the bench: arming leaves it
  if (now < 2000) return;                      // let the VTX boot
  if (f != freq_ || mw != powerMw_ || pit != pit_) {
    freq_ = f;
    powerMw_ = mw;
    pit_ = pit;
    pending_ = 3;
    nextMs_ = now;
    repeated_ = false;
  }
  if (pending_ && (int32_t)(now - nextMs_) >= 0) {
    sendFrame(3 - pending_);  // one frame at a time, 150 ms apart (the VTX retunes in between)
    pending_--;
    nextMs_ = now + 150;
    if (!pending_ && !repeated_) { repeatMs_ = now + 1000; }
  } else if (!pending_ && !repeated_ && (int32_t)(now - repeatMs_) >= 0) {
    repeated_ = true;  // once more: a VTX that was still starting missed the first frames
    pending_ = 3;
    nextMs_ = now;
  }
}

void Driver::sendFrame(int which) {
  uint8_t b[16];
  size_t n = 0;
  if (type_ == Type::SMARTAUDIO) {
    n = which == 0 ? saSetFrequency(b, freq_)
      : which == 1 ? (saIndex_ ? saSetPowerIndex(b, saValue_) : saSetPowerDbm(b, saValue_))
      : saSetPit(b, pit_ == 1);
  } else {
    // 'I' = active state: 0 pit mode, 1 transmitting
    trampFrame(b, which == 0 ? 'F' : which == 1 ? 'P' : 'I', which == 0 ? freq_ : which == 1 ? powerMw_ : (pit_ == 1 ? 0 : 1));
    n = 16;
  }
  hal::portWrite(port_, b, n);
  frames_++;
}

}  // namespace vtx
