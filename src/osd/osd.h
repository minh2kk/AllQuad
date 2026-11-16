// AllQuad - On-Screen Display: one renderer, two backends
//   MAX7456 / AT7456E analog OSD chip on the OSD SPI bus (30 x 16 PAL / 30 x 13 NTSC)
//   MSP DisplayPort for digital systems: DJI O3 / O4 / O4 Pro (canvas 53 x 20), Walksnail Avatar (53 x 20),
//   HDZero (50 x 18), MWOSD / analog MSP OSD (30 x 16)
// Element positions are stored on the 30 x 16 SD grid; on a bigger HD canvas each element keeps its
// anchor (left / centre / right, top / bottom) so one layout serves every system.
// Glyphs follow the Betaflight font, which DJI, Walksnail, HDZero and pre-flashed MAX7456 chips all carry.
#pragma once
#include <stdint.h>

#include "../comms/msp.h"

namespace osd {

constexpr int COLS = 30;  // layout grid (SD)
constexpr int ROWS_PAL = 16, ROWS_NTSC = 13;
constexpr int MAX_COLS = 60, MAX_ROWS = 22;  // largest HD canvas

// Betaflight font glyphs (betaflight.com/docs/development/OSD-Glyps)
namespace sym {
constexpr char RSSI = 0x01, THR = 0x04, VOLT = 0x06, MAH = 0x07, M = 0x0C, HOME = 0x11, ROLL = 0x14, PITCH = 0x15,
               SAT_L = 0x1E, SAT_R = 0x1F, ARROW_S = 0x60, SPEED = 0x70, TOTAL_DIST = 0x71, AH_CENTER_L = 0x72,
               AH_CENTER = 0x73, AH_CENTER_R = 0x74, TEMP = 0x7A, LQ = 0x7B, ALT = 0x7F;
constexpr char AH_BAR0 = (char)0x80, LAT = (char)0x89, BATT = (char)0x97, LON = (char)0x98, AMP = (char)0x9A,
               FLY_M = (char)0x9C, KPH = (char)0x9E, MPS = (char)0x9F;
// progress bar (Betaflight font)
constexpr char PB_START = (char)0x8A, PB_FULL = (char)0x8B, PB_HALF = (char)0x8C, PB_EMPTY = (char)0x8D,
               PB_END = (char)0x8E;
}  // namespace sym

enum class Canvas : uint8_t { AUTO = 0, SD = 1, HD50 = 2, HD53 = 3, HD60 = 4 };

class Backend {
 public:
  virtual ~Backend() {}
  virtual bool begin(bool pal) = 0;
  // false: no room in the link's transmit buffer right now (the row is sent again next frame)
  virtual bool writeChars(int row, int col, const char* s, int n) = 0;
  virtual void endFrame() {}
  virtual void clear() {}
  // canvas the display asked for (MSP_SET_OSD_CANVAS), 0 if none
  virtual void requestedCanvas(int& cols, int& rows) { cols = rows = 0; }
  virtual bool digital() const { return false; }
  virtual void setCanvas(int cols, int rows) { (void)cols; (void)rows; }
};

class Max7456 : public Backend {
 public:
  bool begin(bool pal) override;
  bool writeChars(int row, int col, const char* s, int n) override;
};

class DisplayPort : public Backend {
 public:
  explicit DisplayPort(comms::MspLink* link) : link_(link) {}
  bool begin(bool pal) override;
  bool writeChars(int row, int col, const char* s, int n) override;
  void endFrame() override;
  void clear() override;
  void requestedCanvas(int& cols, int& rows) override;
  bool digital() const override { return true; }
  void setCanvas(int cols, int rows) override;

 private:
  comms::MspLink* link_;
  uint32_t lastHb_ = 0;
};

class Renderer {
 public:
  void begin(Backend* b, bool pal);
  void render(uint32_t nowMs);  // call at ~10 Hz
  const char* row(int r) const { return screen_[r]; }  // for tests
  int cols() const { return cols_; }
  int rows() const { return rows_; }
  // canvas for a backend + setting (exposed for tests)
  static void canvasSize(Canvas c, bool digital, bool pal, int reqCols, int reqRows, int& cols, int& rows);

 private:
  void put(uint16_t pos, const char* text);
  void putAt(int row, int col, const char* text);
  void horizon(uint16_t pos, float rollRad, float pitchRad);
  void applyCanvas(int cols, int rows);
  Backend* be_ = nullptr;
  bool pal_ = true;
  int cols_ = COLS, rows_ = ROWS_PAL, sdRows_ = ROWS_PAL;
  char screen_[MAX_ROWS][MAX_COLS + 1];
  char shown_[MAX_ROWS][MAX_COLS + 1];
  int forceFrom_ = 0;  // rows from here on are rewritten in full (-1: diff only); resumes across frames
  uint32_t lastFullMs_ = 0;
  // flight statistics (OSD elements + the screen after disarm)
  void track(uint32_t nowMs, bool armed, bool posValid, float n, float e, float alt, float speed, float vbat,
             float amps, float mah, float homeDist);
  void drawStats();
  bool wasArmed_ = false, havePos_ = false;
  uint32_t armMs_ = 0, flightS_ = 0, statsUntil_ = 0, lastTrackMs_ = 0;
  float lastN_ = 0, lastE_ = 0, distM_ = 0, maxAlt_ = 0, maxSpeed_ = 0, maxDist_ = 0, minVbat_ = 0, mahArm_ = 0,
        mahFlight_ = 0, avgAmps_ = 0;
};

}  // namespace osd
