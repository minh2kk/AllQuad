#include "../common/size_opt.h"  // -Os on ESP32 (not flight critical)
#include "osd.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../bus/bus.h"
#include "../hal/hal.h"

namespace osd {

using namespace fc;

// ---------------------------------------------------------------- MAX7456
bool Max7456::begin(bool pal) {
  if (!hal::spiHasDevice(hal::SpiDev::OSD)) return false;
  hal::spiSetSpeed(hal::SpiDev::OSD, 8000000);
  hal::spiWrite(hal::SpiDev::OSD, 0x00, 0x02);  // VM0 software reset
  hal::delayMs(2);
  uint8_t vm0 = 0;
  hal::spiRead(hal::SpiDev::OSD, 0x00, &vm0, 1);
  hal::spiWrite(hal::SpiDev::OSD, 0x00, (uint8_t)((pal ? 0x40 : 0x00) | 0x08));  // video standard + OSD enable
  hal::spiWrite(hal::SpiDev::OSD, 0x04, 0x04);  // DMM: clear display memory
  hal::delayMs(1);
  hal::spiWrite(hal::SpiDev::OSD, 0x04, 0x00);
  return true;
}

bool Max7456::writeChars(int row, int col, const char* s, int n) {
  // auto-increment mode: one address, then one SPI byte per character (0xFF ends the burst)
  uint16_t addr = (uint16_t)(row * COLS + col);
  hal::spiWrite(hal::SpiDev::OSD, 0x05, (uint8_t)((addr >> 8) & 1));  // DMAH
  hal::spiWrite(hal::SpiDev::OSD, 0x06, (uint8_t)addr);                // DMAL
  hal::spiWrite(hal::SpiDev::OSD, 0x04, 0x01);                         // DMM: auto-increment
  for (int i = 0; i < n; i++) {
    uint8_t ch = (uint8_t)s[i];
    hal::spiWrite(hal::SpiDev::OSD, 0x07, ch == 0xFF ? ' ' : ch);  // DMDI
  }
  hal::spiWrite(hal::SpiDev::OSD, 0x07, 0xFF);  // leave auto-increment
  return true;
}

// ---------------------------------------------------------------- DisplayPort
bool DisplayPort::begin(bool pal) {
  (void)pal;
  if (!link_) return false;
  link_->dpHeartbeat();
  link_->dpClear();
  return true;
}

bool DisplayPort::writeChars(int row, int col, const char* s, int n) {
  char buf[MAX_COLS + 1];
  if (n > MAX_COLS) n = MAX_COLS;
  memcpy(buf, s, n);
  buf[n] = 0;
  return link_->dpWrite((uint8_t)row, (uint8_t)col, buf);  // never blocks the io task
}

void DisplayPort::endFrame() {
  uint32_t now = hal::millis();
  if (now - lastHb_ > 500) { lastHb_ = now; link_->dpHeartbeat(); }
  link_->dpDraw();
}

void DisplayPort::clear() { link_->dpClear(); }

void DisplayPort::requestedCanvas(int& cols, int& rows) { link_->canvasRequest(cols, rows); }

void DisplayPort::setCanvas(int cols, int rows) {
  // MSP_DP_OPTIONS (INAV / ArduPilot style) picks the grid on HDZero / MWOSD: 0 SD, 1 HD 50x18, 2 HD 60x22.
  // DJI O3 / O4 and Walksnail take the 53x20 canvas as it comes (like Betaflight), nothing to send.
  if (cols == 30) link_->dpOptions(0, 0);
  else if (cols == 50) link_->dpOptions(0, 1);
  else if (cols == 60) link_->dpOptions(0, 2);
  (void)rows;
}

// ---------------------------------------------------------------- renderer
void Renderer::canvasSize(Canvas c, bool digital, bool pal, int reqCols, int reqRows, int& cols, int& rows) {
  int sdRows = pal ? ROWS_PAL : ROWS_NTSC;
  switch (c) {
    case Canvas::SD: cols = COLS; rows = sdRows; return;
    case Canvas::HD50: cols = 50; rows = 18; return;
    case Canvas::HD53: cols = 53; rows = 20; return;
    case Canvas::HD60: cols = 60; rows = 22; return;
    default: break;
  }
  if (!digital) { cols = COLS; rows = sdRows; return; }  // MAX7456 is always SD
  if (reqCols >= COLS && reqRows >= ROWS_NTSC) {          // the display told us (MSP_SET_OSD_CANVAS)
    cols = reqCols > MAX_COLS ? MAX_COLS : reqCols;
    rows = reqRows > MAX_ROWS ? MAX_ROWS : reqRows;
    return;
  }
  cols = 53; rows = 20;  // DJI O3 / O4 / Walksnail HD canvas (Betaflight "HD")
}

void Renderer::applyCanvas(int cols, int rows) {
  cols_ = cols;
  rows_ = rows;
  for (int r = 0; r < MAX_ROWS; r++) {
    memset(screen_[r], ' ', MAX_COLS);
    screen_[r][MAX_COLS] = 0;
    memset(shown_[r], ' ', MAX_COLS);
    shown_[r][MAX_COLS] = 0;
  }
  forceFrom_ = 0;
  if (be_) {
    be_->clear();
    be_->setCanvas(cols, rows);
  }
}

void Renderer::begin(Backend* b, bool pal) {
  be_ = b;
  pal_ = pal;
  sdRows_ = pal ? ROWS_PAL : ROWS_NTSC;
  if (be_ && !be_->begin(pal)) be_ = nullptr;
  int rc = 0, rr = 0, c, r;
  if (be_) be_->requestedCanvas(rc, rr);
  canvasSize((Canvas)bus::config.osd_canvas, be_ && be_->digital(), pal, rc, rr, c, r);
  applyCanvas(c, r);
}

// write at a canvas cell (clipped)
void Renderer::putAt(int row, int col, const char* text) {
  if (row < 0 || row >= rows_) return;
  for (int i = 0; text[i]; i++) {
    int x = col + i;
    if (x >= 0 && x < cols_) screen_[row][x] = text[i];
  }
}

// layout cell (30 x 16 grid) -> canvas cell; left / centre / right and top / bottom anchors are kept
void Renderer::put(uint16_t pos, const char* text) {
  if (!(pos & 0x8000)) return;  // element hidden
  int row = (pos & 0x7FFF) / 32, col = (pos & 0x7FFF) % 32;
  if (row >= sdRows_ || col >= COLS) return;
  int len = (int)strlen(text);
  if (cols_ != COLS) {
    int right = COLS - (col + len);  // gap to the right edge
    if (col <= right && col < 10) {
      // left anchored: keep the gap to the left edge
    } else if (right < col && right < 10) {
      col = cols_ - len - right;
    } else {
      col = (int)(((col + len * 0.5f) * cols_) / COLS - len * 0.5f + 0.5f);
    }
  }
  if (rows_ != sdRows_) {
    int bottom = sdRows_ - 1 - row;
    if (row > bottom) row = rows_ - 1 - bottom;
    else if (row > 2) row = (row * (rows_ - 1) + (sdRows_ - 1) / 2) / (sdRows_ - 1);
  }
  if (col + len > cols_) col = cols_ - len;
  putAt(row, col < 0 ? 0 : col, text);
}

// artificial horizon: 9 columns, Betaflight bar glyphs give 9 sub-rows per character
void Renderer::horizon(uint16_t pos, float roll, float pitch) {
  if (!(pos & 0x8000)) return;
  bool glyphs = bus::config.osd_font == 0;
  int cRow = (pos & 0x7FFF) / 32;
  int col = (int)(15.5f * cols_ / COLS);  // centred on the crosshair
  int row = rows_ == sdRows_ ? cRow : (cRow * (rows_ - 1) + (sdRows_ - 1) / 2) / (sdRows_ - 1);
  float slope = -tanf(roll) * (12.0f / 18.0f);                // rows per column (char cell 12 x 18 px)
  float pitchRows = pitch * RAD2DEG / 8.0f;                    // 8 degrees per row
  const int half = cols_ >= 50 ? 6 : 4;
  for (int x = -half; x <= half; x++) {
    float y = row + pitchRows + x * slope;
    if (y < 1 || y >= rows_ - 1) continue;
    int r = (int)y;
    char ch[2] = {glyphs ? (char)(sym::AH_BAR0 + (int)((y - r) * 9.0f)) : '-', 0};
    if (x == 0) continue;  // the crosshair owns the centre
    putAt(r, col + x, ch);
  }
}

static char arrowGlyph(float relDeg) {
  while (relDeg < 0) relDeg += 360;
  int k = ((int)((relDeg + 11.25f) / 22.5f)) % 16;  // 0 ahead, 4 right, 8 behind, 12 left
  return (char)(sym::ARROW_S + ((8 - k) & 15));
}
static const char* arrowAscii(float relDeg) {
  static const char* a[] = {"^", "/", ">", "\\", "V", "/", "<", "\\"};
  while (relDeg < 0) relDeg += 360;
  return a[((int)((relDeg + 22.5f) / 45.0f)) % 8];
}

void Renderer::track(uint32_t now, bool armed, bool posValid, float n, float e, float alt, float speed, float vbat,
                     float amps, float mah, float homeDist) {
  float dt = lastTrackMs_ ? (now - lastTrackMs_) * 1e-3f : 0.1f;
  lastTrackMs_ = now;
  avgAmps_ += (amps - avgAmps_) * fminf(dt / 10.0f, 1.0f);  // ~10 s average for the time left
  if (armed && !wasArmed_) {
    armMs_ = now; distM_ = maxAlt_ = maxSpeed_ = maxDist_ = 0; minVbat_ = vbat; mahArm_ = mah; havePos_ = false;
    statsUntil_ = 0;
  }
  if (!armed && wasArmed_) {
    flightS_ = (now - armMs_) / 1000;
    uint8_t secs = bus::config.osd_stats;
    statsUntil_ = secs && flightS_ >= 5 ? now + secs * 1000u : 0;
  }
  wasArmed_ = armed;
  if (!armed) return;
  mahFlight_ = mah - mahArm_;
  if (posValid) {
    float d = sqrtf((n - lastN_) * (n - lastN_) + (e - lastE_) * (e - lastE_));
    if (havePos_ && d < 50.0f) distM_ += d;  // ignore a position reset
    lastN_ = n; lastE_ = e; havePos_ = true;
  }
  maxAlt_ = fmaxf(maxAlt_, alt);
  maxSpeed_ = fmaxf(maxSpeed_, speed);
  maxDist_ = fmaxf(maxDist_, homeDist);
  if (vbat > 1.0f) minVbat_ = minVbat_ > 1.0f ? fminf(minVbat_, vbat) : vbat;
}

void Renderer::drawStats() {
  char t[40];
  int w = 24, col = (cols_ - w) / 2, row = (rows_ - 9) / 2;
  putAt(row++, col, "     FLIGHT STATS");
  snprintf(t, sizeof(t), "TIME       %02u:%02u", (unsigned)(flightS_ / 60), (unsigned)(flightS_ % 60)); putAt(row++, col, t);
  snprintf(t, sizeof(t), "MAX ALT    %.1f M", maxAlt_); putAt(row++, col, t);
  snprintf(t, sizeof(t), "MAX SPEED  %.0f KMH", maxSpeed_ * 3.6f); putAt(row++, col, t);
  snprintf(t, sizeof(t), "MAX DIST   %.0f M", maxDist_); putAt(row++, col, t);
  snprintf(t, sizeof(t), "DISTANCE   %.2f KM", distM_ / 1000.0f); putAt(row++, col, t);
  snprintf(t, sizeof(t), "USED       %.0f MAH", mahFlight_); putAt(row++, col, t);
  snprintf(t, sizeof(t), "MIN VOLT   %.1f V", minVbat_); putAt(row++, col, t);
  if (distM_ > 100.0f) { snprintf(t, sizeof(t), "EFFICIENCY %.0f MAH/KM", mahFlight_ / (distM_ / 1000.0f)); putAt(row++, col, t); }
}

void Renderer::render(uint32_t now) {
  const Config& c = bus::config;
  // follow a canvas the display announces later (HDZero / Walksnail send it after boot)
  if (be_ && (Canvas)c.osd_canvas == Canvas::AUTO) {
    int rc = 0, rr = 0, nc, nr;
    be_->requestedCanvas(rc, rr);
    canvasSize(Canvas::AUTO, be_->digital(), pal_, rc, rr, nc, nr);
    if (nc != cols_ || nr != rows_) applyCanvas(nc, nr);
  }
  for (int r = 0; r < rows_; r++) memset(screen_[r], ' ', cols_);
  VehicleStatus st = bus::status.get();
  BatteryState b = bus::battery.get();
  NavEstimate e = bus::nav.get();
  AttitudeState a = bus::attitude.get();
  RcInput rc = bus::rc.get();
  GpsData g = bus::gps.get();
  char t[40];
  const uint16_t* P = c.osd_pos;
  const bool G = c.osd_font == 0;  // Betaflight glyphs
  float speed = e.posValid ? sqrtf(e.vn * e.vn + e.ve * e.ve) : 0.0f;
  track(now, st.armed, e.posValid, e.n, e.e, e.alt, speed, b.voltage, b.current, b.mahDrawn,
        e.homeSet ? st.homeDistance : 0.0f);
  if (!st.armed && statsUntil_ && (int32_t)(statsUntil_ - now) > 0) {
    drawStats();
  } else {
  // ---- v24 elements
  const uint16_t* P2 = c.osd_pos2;
  {
    EscTelemetry esc = bus::esc.get();
    float tmax = -100;
    for (int i = 0; i < 4; i++)
      if (esc.lastUpdateMs[i] && ageMs(now, esc.lastUpdateMs[i]) < 2000) tmax = fmaxf(tmax, esc.tempC[i]);
    if (tmax > -100) {
      if (G) snprintf(t, sizeof(t), "%c%3dC", sym::TEMP, (int)tmax); else snprintf(t, sizeof(t), "T%3dC", (int)tmax);
      put(P2[OSD2_ESC_TEMP], t);
    }
  }
  if (G) snprintf(t, sizeof(t), "%c%.2fKM", sym::TOTAL_DIST, distM_ / 1000.0f); else snprintf(t, sizeof(t), "D%.2fKM", distM_ / 1000.0f);
  put(P2[OSD2_DIST_TOTAL], t);
  if (distM_ > 100.0f) { snprintf(t, sizeof(t), "%3.0fMAH/KM", mahFlight_ / (distM_ / 1000.0f)); put(P2[OSD2_EFFICIENCY], t); }
  if (c.bat_capacity) {
    float left = c.bat_capacity * (100 - c.bat_reserve_pct) * 0.01f - b.mahDrawn;
    if (avgAmps_ > 0.5f && left > 0) {
      int sLeft = (int)(left / (avgAmps_ * 1000.0f / 3600.0f));
      snprintf(t, sizeof(t), "%c%02d:%02d", G ? sym::FLY_M : 'L', sLeft / 60 > 99 ? 99 : sLeft / 60, sLeft % 60);
    } else {
      snprintf(t, sizeof(t), "%c--:--", G ? sym::FLY_M : 'L');
    }
    put(P2[OSD2_TIME_LEFT], t);
  }

  bool blink = (now / 400) % 2;
  // low battery: voltage, cell voltage and bar blink (osd_blink)
  bool batShow = !(c.osd_blink && st.armed && (st.warnings & (WARN_BAT_LOW | WARN_BAT_CRIT))) || blink;
  {
    const uint16_t* P3 = c.osd_pos3;
    if (batShow) {
      // battery bar: remaining charge in 8 cells of two halves each
      int pct = b.percent > 100 ? 100 : b.percent, halves = (pct * 16 + 50) / 100;
      char bar[12];
      int k = 0;
      bar[k++] = G ? sym::PB_START : '[';
      for (int i = 0; i < 8; i++) {
        int hv = halves - 2 * i;
        bar[k++] = hv >= 2 ? (G ? sym::PB_FULL : '#') : hv == 1 ? (G ? sym::PB_HALF : '=') : (G ? sym::PB_EMPTY : '-');
      }
      bar[k++] = G ? sym::PB_END : ']';
      bar[k] = 0;
      put(P3[OSD3_BAT_BAR], bar);
      if (b.cells) {
        float cell = b.voltage / b.cells;
        if (G) snprintf(t, sizeof(t), "%c%4.2f%c", sym::BATT, cell, sym::VOLT); else snprintf(t, sizeof(t), "%4.2fV/C", cell);
        put(P3[OSD3_CELL_V], t);
      }
    }
    if (rc.everReceived && rc.rssiDbm) {
      if (G) snprintf(t, sizeof(t), "%c%4d", sym::RSSI, (int)rc.rssiDbm); else snprintf(t, sizeof(t), "%4dDBM", (int)rc.rssiDbm);
      put(P3[OSD3_RSSI_DBM], t);
    }
  }
  if (G) {
    snprintf(t, sizeof(t), "%c%4.1f%c", sym::BATT, b.voltage, sym::VOLT); if (batShow) put(P[OSD_VBAT], t);
    snprintf(t, sizeof(t), "%4.1f%c", b.current, sym::AMP); put(P[OSD_CURRENT], t);
    snprintf(t, sizeof(t), "%4d%c", (int)b.mahDrawn, sym::MAH); put(P[OSD_MAH], t);
  } else {
    snprintf(t, sizeof(t), "%4.1fV", b.voltage); if (batShow) put(P[OSD_VBAT], t);
    snprintf(t, sizeof(t), "%4.1fA", b.current); put(P[OSD_CURRENT], t);
    snprintf(t, sizeof(t), "%4dMAH", (int)b.mahDrawn); put(P[OSD_MAH], t);
  }
  if (!st.failsafe || blink || !c.osd_blink) put(P[OSD_MODE], flightModeName(st.mode));
  uint32_t s = st.armedTimeMs / 1000;
  if (G) snprintf(t, sizeof(t), "%c%02u:%02u", sym::FLY_M, (unsigned)(s / 60), (unsigned)(s % 60));
  else snprintf(t, sizeof(t), "%02u:%02u", (unsigned)(s / 60), (unsigned)(s % 60));
  put(P[OSD_ARM_TIME], t);
  if (e.altValid) {
    if (G) snprintf(t, sizeof(t), "%c%5.1f%c", sym::ALT, e.alt, sym::M); else snprintf(t, sizeof(t), "A%5.1f", e.alt);
    put(P[OSD_ALTITUDE], t);
    if (G) snprintf(t, sizeof(t), "%+4.1f%c", e.vz, sym::MPS); else snprintf(t, sizeof(t), "V%+4.1f", e.vz);
    put(P[OSD_VARIO], t);
  }
  if (e.homeSet) {
    if (G) snprintf(t, sizeof(t), "%c%4d%c", sym::HOME, (int)st.homeDistance, sym::M);
    else snprintf(t, sizeof(t), "H%4dM", (int)st.homeDistance);
    put(P[OSD_HOME_DIST], t);
    float rel = st.homeBearingDeg - e.yaw * RAD2DEG;
    if (G) { t[0] = arrowGlyph(rel); t[1] = 0; put(P[OSD_HOME_DIR], t); }
    else put(P[OSD_HOME_DIR], arrowAscii(rel));
  }
  if (bus::gps.seq()) {
    if (G) snprintf(t, sizeof(t), "%c%c%2u", sym::SAT_L, sym::SAT_R, g.sats); else snprintf(t, sizeof(t), "S%02u", g.sats);
    put(P[OSD_GPS_SATS], t);
  }
  if (e.posValid) {
    int kmh = (int)(sqrtf(e.vn * e.vn + e.ve * e.ve) * 3.6f);
    if (G) snprintf(t, sizeof(t), "%c%3d%c", sym::SPEED, kmh, sym::KPH); else snprintf(t, sizeof(t), "%3dKMH", kmh);
    put(P[OSD_SPEED], t);
  }
  if (rc.linkQuality) {
    if (G) snprintf(t, sizeof(t), "%c%3u", sym::LQ, rc.linkQuality); else snprintf(t, sizeof(t), "LQ%3u", rc.linkQuality);
    put(P[OSD_LINK], t);
  }
  if (G) { const char ch[4] = {sym::AH_CENTER_L, sym::AH_CENTER, sym::AH_CENTER_R, 0}; put(P[OSD_CROSSHAIR], ch); }
  else put(P[OSD_CROSSHAIR], "-+-");
  horizon(P[OSD_HORIZON], a.roll, a.pitch);
  if (g.fix3d) {
    if (G) snprintf(t, sizeof(t), "%c%.6f %c%.6f", sym::LAT, g.lat, sym::LON, g.lon);
    else snprintf(t, sizeof(t), "%.5f %.5f", g.lat, g.lon);
    put(P[OSD_COORDS], t);
  }
  // warnings (highest priority first)
  const char* w = nullptr;
  if (st.warnings & WARN_RX_LOST) w = "FAILSAFE";
  else if (st.warnings & WARN_BAT_CRIT) w = "BATTERY CRITICAL";
  else if (st.warnings & WARN_FENCE) w = "GEOFENCE";
  else if (st.warnings & WARN_GCS_LOST) w = "GCS LOST";
  else if (st.warnings & WARN_IMU2) w = "IMU 2 ACTIVE";
  else if (st.warnings & WARN_BAT_LOW) w = "LOW BATTERY";
  else if (st.warnings & WARN_GPS_LOST) w = "GPS LOST";
  else if (st.warnings & WARN_RESCUE) w = "RESCUE: NO PROGRESS";
  else if (st.warnings & WARN_GPS_GLITCH) w = "GPS GLITCH";
  else if (st.warnings & WARN_EKF) w = "EKF: SENSOR FAULT";
  else if (st.warnings & WARN_VIBRATION) w = "VIBRATION";
  else if (st.warnings & WARN_RPM_LOST) w = "RPM TELEMETRY LOST";
  else if (st.warnings & WARN_LAUNCH) w = "LAUNCH READY";
  else if (st.atState == 1) {
    static const char* const stepName[] = {"", " FF", " I"};
    snprintf(t, sizeof(t), "AUTOTUNE %s%s %u", st.atAxis == 0 ? "ROLL" : st.atAxis == 1 ? "PITCH" : "YAW",
             stepName[st.atStep < 3 ? st.atStep : 0], st.atRound + 1u);
    w = t;
  }
  else if (st.atState == 2) w = "AUTOTUNE DONE - LAND";
  else if (st.warnings & WARN_NO_HEADING) w = "NO HEADING";
  else if (st.warnings & WARN_LANDING) w = "LANDING";
  else if (!st.armed && st.armBlockers) {
    for (int i = 0; i < ARM_COUNT_BITS; i++)
      if (st.armBlockers & (1u << i)) { snprintf(t, sizeof(t), "NOT READY:%s", armCheckName(i)); w = t; break; }
  }
  if (w && (blink || !st.armed || !c.osd_blink)) put(P[OSD_WARNINGS], w);
  }  // normal screen

  if (!be_) return;
  // digital displays may power up after the FC: resend the whole screen once a second
  if (be_->digital() && forceFrom_ < 0 && now - lastFullMs_ >= 1000) { lastFullMs_ = now; forceFrom_ = 0; }
  bool full = true;
  for (int r = 0; r < rows_; r++) {
    bool force = forceFrom_ >= 0 && r >= forceFrom_;
    if (!force && !memcmp(screen_[r], shown_[r], cols_)) continue;
    // send the changed span of the row (only the drawn part on a full refresh)
    int lo = 0, hi = cols_ - 1;
    if (!force) {
      while (lo < cols_ && screen_[r][lo] == shown_[r][lo]) lo++;
      while (hi > lo && screen_[r][hi] == shown_[r][hi]) hi--;
    } else if (be_->digital()) {
      while (lo < cols_ && screen_[r][lo] == ' ' && shown_[r][lo] == ' ') lo++;
      while (hi > lo && screen_[r][hi] == ' ' && shown_[r][hi] == ' ') hi--;
      if (lo >= cols_) continue;
    }
    if (!be_->writeChars(r, lo, screen_[r] + lo, hi - lo + 1)) {  // link busy: carry on next frame
      if (force) forceFrom_ = r;
      full = false;
      break;
    }
    memcpy(shown_[r], screen_[r], cols_);
  }
  if (full) forceFrom_ = -1;
  be_->endFrame();
}

}  // namespace osd
