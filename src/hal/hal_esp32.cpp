// AllQuad - ESP32 / ESP32-S3 HAL (Arduino-ESP32 core 3.x)
//  * motors: DShot300/600 via RMT (telemetry bit, special commands), PWM/OneShot125 via LEDC
//  * second SPI bus (HSPI) for the OSD chip
//  * storage: NVS (Preferences); blackbox: raw flash partition; WiFi AP + MAVLink over UDP
#if defined(ARDUINO) && defined(ESP32) && !defined(ALLQUAD_HOST)
#include <Arduino.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <driver/gpio.h>
#include <driver/rmt_rx.h>
#include <driver/rmt_tx.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_oneshot.h>
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S3
#include <soc/rtc_cntl_reg.h>
#endif
#include <Preferences.h>
#include <SPI.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#include <lwip/sockets.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <FS.h>
#include <SD.h>
#if SOC_SDMMC_HOST_SUPPORTED
#include <SD_MMC.h>
#endif

#include "../common/cycles.h"
#include "../common/math3d.h"
#include "board.h"
#include "dshot_codec.h"
#include "hal.h"
#include "hal_internal.h"

namespace hal {

static const int motorPins[4] = {PIN_MOTOR1, PIN_MOTOR2, PIN_MOTOR3, PIN_MOTOR4};
static fc::MotorProtocol motorProto = fc::MotorProtocol::DSHOT300;
static bool motorsReady = false;
static uint32_t pwmPeriodUs = 2500, pwmMinUs = 1000, pwmMaxUs = 2000;
static constexpr uint8_t LEDC_BITS = 14;
static rmt_data_t dshotFrame[4][16];
static uint16_t dshotT0H, dshotT1H, dshotBit;
static SPIClass* osdSpi = nullptr;

const char* platformName() { return AQ_BOARD_NAME; }

#if ARDUINO_USB_CDC_ON_BOOT
static constexpr bool CONSOLE_UART = false;  // S3 native USB: no baud rate
#else
static constexpr bool CONSOLE_UART = true;
#endif
// 921600: the configurator moves a 2 kB status/parameter page in ~25 ms, a 4 MB blackbox in about a
// minute. CP2102 / CH340 / CH9102 bridges all do it. (platformio.ini monitor_speed matches.)
static constexpr uint32_t CONSOLE_BAUD = 921600;
int resetReason() { return (int)esp_reset_reason(); }
// the ESP-IDF panic handler and task watchdog report through the reset reason (and the coredump partition)
bool lastCrash(CrashInfo&) { return false; }
void crashClear() {}
void watchdogStart() {}
void watchdogKick() {}
bool watchdogRunning() { return false; }
int imuIntPin() {
#if defined(PIN_IMU_INT)
  return PIN_IMU_INT;
#else
  return -1;
#endif
}

#if defined(AQ_CONSOLE_DUAL)
// replies go to the port that sent the last byte (native USB until the bridge port is used)
static bool consoleOnUart = false;
int internal::consoleRead() {
  if (Serial.available()) { consoleOnUart = false; return Serial.read(); }
  if (Serial0.available()) { consoleOnUart = true; return Serial0.read(); }
  return -1;
}
size_t internal::consoleWrite(const uint8_t* d, size_t n) {
  return consoleOnUart ? Serial0.write(d, n) : Serial.write(d, n);
}
size_t internal::consoleWriteSpace() {
  return (size_t)(consoleOnUart ? Serial0.availableForWrite() : Serial.availableForWrite());
}
uint32_t consoleBaud() { return consoleOnUart ? CONSOLE_BAUD : 0; }
#else
uint32_t consoleBaud() { return CONSOLE_UART ? CONSOLE_BAUD : 0; }
#endif

void init() {
#if !ARDUINO_USB_CDC_ON_BOOT
  // configurator frames are up to ~2 kB: buffer them instead of feeding the 128-byte UART FIFO
  Serial.setRxBufferSize(2048);
  Serial.setTxBufferSize(2048);
#endif
  Serial.begin(CONSOLE_BAUD);
#if defined(AQ_CONSOLE_DUAL)
  Serial0.setRxBufferSize(2048);
  Serial0.setTxBufferSize(2048);
  Serial0.begin(CONSOLE_BAUD);  // UART0 = the board's "COM" port (USB-UART bridge)
#endif
  // the console UART carries the configurator's binary frames, CLI, MSP and MAVLink: ESP-IDF log
  // lines (printed straight into the UART FIFO by the precompiled libraries, e.g. ADC calibration
  // errors on chips without an eFuse Vref) would land in the middle of a frame and break it
  esp_log_level_set("*", ESP_LOG_NONE);
  internal::commonInit();
  internal::i2cPrepare(PIN_I2C_SDA, PIN_I2C_SCL);
  internal::i2cStart();  // the ESP32 driver enables the internal pull-ups
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);
#if PIN_OSD_CS >= 0
  osdSpi = new SPIClass(HSPI);
  osdSpi->begin(PIN_OSD_SCK, PIN_OSD_MISO, PIN_OSD_MOSI, -1);
#endif
  analogReadResolution(12);
}

// ---------------- SPI ----------------
SPIClass* internal::spiBus(SpiDev d) { return d == SpiDev::OSD ? osdSpi : &SPI; }
int internal::csPin(SpiDev d) {
  switch (d) {
    case SpiDev::IMU: return PIN_IMU_CS;
    case SpiDev::IMU2: return PIN_IMU2_CS;
    case SpiDev::OSD: return PIN_OSD_CS;
  }
  return -1;
}

// ---------------- serial ports ----------------
HardwareSerial* internal::serialPort(Port p) {
#if defined(BOARD_ESP32S3_DEVKIT) || defined(BOARD_ESP32S3_N16R8)
  switch (p) {
    case Port::SERIAL1: return &Serial1;
    case Port::SERIAL2: return &Serial2;
#if !defined(AQ_CONSOLE_DUAL)
    case Port::SERIAL3: return &Serial0;
#endif
    default: return nullptr;
  }
#elif defined(BOARD_ESP32C6_DEVKIT)
  switch (p) {
    case Port::SERIAL1: return &Serial1;
    case Port::SERIAL2: return &Serial0;  // UART0: the console is the native USB port
    default: return nullptr;
  }
#elif defined(BOARD_ESP32P4)
  switch (p) {
    case Port::SERIAL1: return &Serial1;
    case Port::SERIAL2: return &Serial2;
    default: return nullptr;
  }
#else
  switch (p) {
    case Port::SERIAL1: return &Serial2;
    case Port::SERIAL2: return &Serial1;
    default: return nullptr;
  }
#endif
}

void portOpen(Port p, uint32_t baud, UartFormat fmt, bool inverted) {
  if (p == Port::CONSOLE || p == Port::UDP) return;
  HardwareSerial* s = internal::serialPort(p);
  if (!s) return;
  int rx = -1, tx = -1;
  if (p == Port::SERIAL1) { rx = PIN_SERIAL1_RX; tx = PIN_SERIAL1_TX; }
  if (p == Port::SERIAL2) { rx = PIN_SERIAL2_RX; tx = PIN_SERIAL2_TX; }
  if (p == Port::SERIAL3) { rx = PIN_SERIAL3_RX; tx = PIN_SERIAL3_TX; }
  if (rx < 0 && tx < 0) return;
  s->end();
  s->setRxBufferSize(1024);
  s->setTxBufferSize(1024);
  s->begin(baud, fmt == UartFormat::F8E2 ? SERIAL_8E2 : fmt == UartFormat::F8N2 ? SERIAL_8N2 : SERIAL_8N1, rx, tx, inverted);
  internal::portMarkOpen(p);
  // line errors are reported by the UART driver's event task: counted per port (web: Cổng tab)
  s->onReceiveError([p](hardwareSerial_error_t e) {
    PortStats& st = internal::portStatsRef(p);
    if (e == UART_FIFO_OVF_ERROR) st.overruns++;
    else if (e == UART_BUFFER_FULL_ERROR) st.dropped++;
    else if (e == UART_FRAME_ERROR || e == UART_BREAK_ERROR) st.framing++;
    else if (e == UART_PARITY_ERROR) st.noise++;
  });
}

// ---------------- UDP (WiFi access point) ----------------
static WiFiUDP udp;
static bool udpUp = false;
static IPAddress peer(192, 168, 4, 255);
static uint16_t peerPort = 14550;
static uint8_t udpBuf[1024];
static int udpLen = 0, udpPos = 0;

bool internal::udpSupported() { return true; }

static char apSsid[33], apPass[65];
uint32_t deviceId() { return (uint32_t)(ESP.getEfuseMac() >> 32); }
uint32_t random32() { return esp_random(); }

static bool apStart() {
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(apSsid, apPass)) return false;
  // a phone a few metres away needs little power: less current draw and less 2.4 GHz noise
  WiFi.setTxPower(WIFI_POWER_11dBm);
  return true;
}

bool wifiStart(const char* ssid, const char* pass) {
  strncpy(apSsid, ssid, sizeof(apSsid) - 1);
  strncpy(apPass, pass, sizeof(apPass) - 1);
  if (!apStart()) return false;
  if (udpUp) return true;  // restart with new credentials
  udpUp = udp.begin(14550);
  return udpUp;
}

void netReport(char* out, size_t n) {
  size_t k = 0;
  auto add = [&](const char* fmt, auto... a) {
    if (k < n) { int w = snprintf(out + k, n - k, fmt, a...); if (w > 0) k += (size_t)w; }
  };
  wifi_mode_t mode = WiFi.getMode();
  add("wifi mode %s | SSID %s | channel %d | tx power %.1f dBm\r\n",
      mode == WIFI_MODE_AP ? "AP" : mode == WIFI_MODE_NULL ? "OFF" : mode == WIFI_MODE_APSTA ? "AP+STA" : "STA",
      apSsid, (int)WiFi.channel(), (int)WiFi.getTxPower() * 0.25f);
  if (mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA) {
    add("AP address %s | phones connected %d\r\n", WiFi.softAPIP().toString().c_str(), (int)WiFi.softAPgetStationNum());
    wifi_sta_list_t sl = {};
    if (esp_wifi_ap_get_sta_list(&sl) == ESP_OK && sl.num > 0) {
      esp_netif_pair_mac_ip_t pairs[10] = {};
      int m = sl.num < 10 ? sl.num : 10;
      for (int i = 0; i < m; i++) memcpy(pairs[i].mac, sl.sta[i].mac, 6);
      esp_netif_t* ap = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
      bool ips = ap && esp_netif_dhcps_get_clients_by_mac(ap, m, pairs) == ESP_OK;
      for (int i = 0; i < m; i++) {
        const uint8_t* mc = sl.sta[i].mac;
        add("  phone %02x:%02x:%02x:%02x:%02x:%02x rssi %d dBm ip ", mc[0], mc[1], mc[2], mc[3], mc[4], mc[5], sl.sta[i].rssi);
        if (ips && pairs[i].ip.addr) add(IPSTR "\r\n", IP2STR(&pairs[i].ip));
        else add("(no DHCP address yet)\r\n");
      }
    }
  }
  // the FC asks its own web server for the page, as a phone would
  int s = socket(AF_INET, SOCK_STREAM, 0);
  if (s < 0) { add("http self-test: no socket free (errno %d)\r\n", errno); return; }
  struct timeval tv = {2, 0};
  setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
  sockaddr_in a = {};
  a.sin_family = AF_INET;
  a.sin_port = htons(80);
  a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  uint32_t t0 = millis();
  if (connect(s, (sockaddr*)&a, sizeof(a)) != 0) { add("http self-test: connect to port 80 FAILED (errno %d): web server not listening\r\n", errno); close(s); return; }
  static const char rq[] = "GET / HTTP/1.1\r\nHost: 192.168.4.1\r\nConnection: close\r\n\r\n";
  send(s, rq, sizeof(rq) - 1, 0);
  char buf[256];
  int total = 0, r;
  char first[48] = "";
  while ((r = recv(s, buf, sizeof(buf), 0)) > 0) {
    if (!total) { int l = 0; while (l < r && l < 47 && buf[l] != '\r') { first[l] = buf[l]; l++; } first[l] = 0; }
    total += r;
  }
  close(s);
  add("http self-test: \"%s\", %d bytes in %lu ms%s\r\n", first, total, (unsigned long)(millis() - t0),
      total > 1000 ? " (web server OK)" : " (PROBLEM)");
}

void wifiSetEnabled(bool on) {
  if (on) {
    if (WiFi.getMode() == WIFI_MODE_NULL && apStart()) { udp.stop(); udpUp = udp.begin(14550); }
    else if (WiFi.getMode() != WIFI_MODE_NULL) WiFi.softAP(apSsid, apPass);  // new name/password
  } else {
    udpUp = false;
    WiFi.mode(WIFI_OFF);
  }
}

int internal::udpRead() {
  if (!udpUp) return -1;
  if (udpPos >= udpLen) {
    int n = udp.parsePacket();
    if (n <= 0) return -1;
    udpLen = udp.read(udpBuf, sizeof(udpBuf));
    udpPos = 0;
    peer = udp.remoteIP();  // answer whoever talks to us
    peerPort = udp.remotePort();
    if (udpLen <= 0) return -1;
  }
  return udpBuf[udpPos++];
}

size_t internal::udpWrite(const uint8_t* data, size_t len) {
  if (!udpUp || !data || !len) return 0;
  udp.beginPacket(peer, peerPort);
  udp.write(data, len);
  udp.endPacket();
  return len;
}

// ---------------- motors ----------------
bool motorsAreDshot() {
  return motorProto == fc::MotorProtocol::DSHOT300 || motorProto == fc::MotorProtocol::DSHOT600;
}

// ---- bidirectional DShot: IDF RMT driver directly (one TX + one RX channel on the same pin) ----
// TX: inverted output (idle high), open drain + pull-up, loop-back into the RX channel. RX is armed
// right before each frame; it records the frame itself, the gap and the ESC's answer, and stops 40 us
// after the last edge. The next motorsWrite() decodes it (dshot::decodeRuns skips the FC's own frame).
// Uses every RMT channel (ESP32: 4 TX + 4 RX of 8, ESP32-S3: 4 + 4): no LED strip with bidir DShot.
static bool bidir = false, bidirWanted = false;
static rmt_channel_handle_t bdTx[4], bdRx[4];
static rmt_encoder_handle_t bdEnc[4];
static rmt_symbol_word_t bdTxBuf[4][16];
static rmt_symbol_word_t bdRxBuf[4][64];
static volatile uint16_t bdRxN[4];
static volatile bool bdRxDone[4], bdRxBusy[4];
static float bdErpm[4] = {-1, -1, -1, -1};
static uint32_t bdOkMs[4], bdFrames, bdErrors;
static dshot::Edt bdEdt[4];
static float bdBitTicks = 26.67f;  // telemetry bit in 0.1 us RX ticks

static bool IRAM_ATTR bdOnRecv(rmt_channel_handle_t, const rmt_rx_done_event_data_t* e, void* ctx) {
  int m = (int)(intptr_t)ctx;
  bdRxN[m] = (uint16_t)e->num_symbols;
  bdRxDone[m] = true;
  bdRxBusy[m] = false;
  return false;
}

static bool bidirInit(bool d600) {
  for (int m = 0; m < 4; m++) {
    rmt_rx_channel_config_t rc = {};
    rc.gpio_num = (gpio_num_t)motorPins[m];
    rc.clk_src = RMT_CLK_SRC_DEFAULT;
    rc.resolution_hz = 10000000;
    rc.mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL;
    if (rmt_new_rx_channel(&rc, &bdRx[m]) != ESP_OK) return false;
    rmt_tx_channel_config_t tc = {};
    tc.gpio_num = (gpio_num_t)motorPins[m];
    tc.clk_src = RMT_CLK_SRC_DEFAULT;
    tc.resolution_hz = 40000000;
    tc.mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL;
    tc.trans_queue_depth = 2;
    tc.flags.invert_out = 1;
    tc.flags.io_loop_back = 1;
    tc.flags.io_od_mode = 1;
    if (rmt_new_tx_channel(&tc, &bdTx[m]) != ESP_OK) return false;
    rmt_copy_encoder_config_t ec = {};
    if (rmt_new_copy_encoder(&ec, &bdEnc[m]) != ESP_OK) return false;
    rmt_rx_event_callbacks_t cb = {};
    cb.on_recv_done = bdOnRecv;
    rmt_rx_register_event_callbacks(bdRx[m], &cb, (void*)(intptr_t)m);
    rmt_enable(bdRx[m]);
    rmt_enable(bdTx[m]);
    gpio_pullup_en((gpio_num_t)motorPins[m]);
  }
  bdBitTicks = d600 ? 13.333f : 26.667f;
  return true;
}

static void bidirDecode(int m, uint32_t nowMs) {
  uint8_t lv[130];
  float du[130];
  int k = 0;
  for (int i = 0; i < bdRxN[m] && k < 128; i++) {
    const rmt_symbol_word_t& s = bdRxBuf[m][i];
    uint32_t d[2] = {s.duration0, s.duration1};
    uint8_t l[2] = {(uint8_t)s.level0, (uint8_t)s.level1};
    for (int h = 0; h < 2; h++) {
      if (!d[h]) { if (k) { lv[k] = !lv[k - 1]; du[k++] = 1000; } break; }  // end marker: idle follows
      if (k && lv[k - 1] == l[h]) du[k - 1] += d[h];
      else { lv[k] = l[h]; du[k++] = (float)d[h]; }
    }
  }
  float e;
  bdFrames++;
  uint8_t kind = k ? dshot::decodeRunsEx(lv, du, k, bdBitTicks, e, &bdEdt[m]) : dshot::BAD;
  if (kind == dshot::ERPM) { bdErpm[m] = e; bdOkMs[m] = nowMs; }
  else if (kind == dshot::BAD) bdErrors++;  // an EDT frame keeps the last eRPM
}

static void bidirSend(const uint16_t v[4], int telemetryMotor) {
  uint32_t now = millis();
  rmt_receive_config_t rcfg = {};
  rcfg.signal_range_min_ns = 300;
  rcfg.signal_range_max_ns = 40000;
  rmt_transmit_config_t tcfg = {};
  for (int m = 0; m < 4; m++) {
    if (bdRxDone[m]) { bdRxDone[m] = false; bidirDecode(m, now); }
    if (now - bdOkMs[m] > 100) bdErpm[m] = -1;
    uint16_t pkt = dshot::frame(v[m], m == telemetryMotor || telemetryMotor == -2, true);
    for (int b = 0; b < 16; b++) {
      uint16_t th = (pkt & (0x8000 >> b)) ? dshotT1H : dshotT0H;
      bdTxBuf[m][b].level0 = 1;  // inverted on the pin by the channel
      bdTxBuf[m][b].duration0 = th;
      bdTxBuf[m][b].level1 = 0;
      bdTxBuf[m][b].duration1 = dshotBit - th;
    }
    if (!bdRxBusy[m] && rmt_receive(bdRx[m], bdRxBuf[m], sizeof(bdRxBuf[m]), &rcfg) == ESP_OK) bdRxBusy[m] = true;
    rmt_transmit(bdTx[m], bdEnc[m], bdTxBuf[m], sizeof(bdTxBuf[m]), &tcfg);
  }
}

bool motorsBidirActive() { return bidir && motorsReady; }
void motorsErpm(float erpm[4]) {
  for (int m = 0; m < 4; m++) erpm[m] = bidir ? bdErpm[m] : -1;
}
void motorsBidirStats(uint32_t& frames, uint32_t& errors) { frames = bdFrames; errors = bdErrors; }
bool motorsEdt(int o, dshot::Edt& out) {
  if (o < 0 || o > 3 || !bdEdt[o].frames) return false;
  out = bdEdt[o];
  return true;
}

bool motorsInit(fc::MotorProtocol p, uint16_t loopHz, bool wantBidir) {
  (void)loopHz;
#if SOC_RMT_TX_CANDIDATES_PER_GROUP < 4
  // ESP32-C6 / H2: 2 RMT transmit channels, 4 DShot outputs do not fit
  if (p == fc::MotorProtocol::DSHOT300 || p == fc::MotorProtocol::DSHOT600) p = fc::MotorProtocol::ONESHOT125;
#endif
  motorProto = p;
  bool ok = true;
  if (motorsAreDshot()) {
    bool d600 = p == fc::MotorProtocol::DSHOT600;  // RMT tick 25 ns
    dshotBit = d600 ? 67 : 133;
    dshotT1H = d600 ? 50 : 100;
    dshotT0H = d600 ? 25 : 50;
    bidirWanted = wantBidir;
    bidir = wantBidir && bidirInit(d600);
    if (!bidir)
      for (int i = 0; i < 4; i++) ok &= rmtInit(motorPins[i], RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 40000000);
  } else {
    bidir = false;
    if (p == fc::MotorProtocol::ONESHOT125) { pwmPeriodUs = 500; pwmMinUs = 125; pwmMaxUs = 250; }
    else { pwmPeriodUs = 2500; pwmMinUs = 1000; pwmMaxUs = 2000; }
    for (int i = 0; i < 4; i++) ok &= ledcAttach(motorPins[i], 1000000 / pwmPeriodUs, LEDC_BITS);
  }
  motorsReady = ok;
  float stop[4] = {0, 0, 0, 0};
  motorsWrite(stop, false);
  return ok;
}

static void dshotSend(const uint16_t v[4], int telemetryMotor, bool allTelemetry) {
  if (bidir) {
    uint16_t w[4] = {v[0], v[1], v[2], v[3]};
    bidirSend(w, allTelemetry ? -2 : telemetryMotor);
    return;
  }
  for (int m = 0; m < 4; m++) {
    uint16_t pkt = dshot::frame(v[m], allTelemetry || m == telemetryMotor, false);
    for (int b = 0; b < 16; b++) {
      bool one = pkt & (0x8000 >> b);
      uint16_t th = one ? dshotT1H : dshotT0H;
      dshotFrame[m][b].level0 = 1;
      dshotFrame[m][b].duration0 = th;
      dshotFrame[m][b].level1 = 0;
      dshotFrame[m][b].duration1 = dshotBit - th;
    }
    rmtWriteAsync(motorPins[m], dshotFrame[m], 16);
  }
}

void motorsWrite(const float out[4], bool armed, int telemetryMotor) {
  if (!motorsReady) return;
  if (motorsAreDshot()) {
    uint16_t v[4];
    for (int m = 0; m < 4; m++) v[m] = (armed && out[m] >= 0) ? (uint16_t)(48 + fc::constrainf(out[m], 0, 1) * 1999.0f) : 0;
    dshotSend(v, telemetryMotor, false);
  } else {
    const uint32_t maxDuty = (1u << LEDC_BITS) - 1;
    for (int m = 0; m < 4; m++) {
      float o = (armed && out[m] >= 0) ? fc::constrainf(out[m], 0, 1) : 0.0f;
      float us = pwmMinUs + o * (pwmMaxUs - pwmMinUs);
      ledcWrite(motorPins[m], (uint32_t)(us / pwmPeriodUs * maxDuty));
    }
  }
}

void motorsCommandMask(uint8_t cmd, uint8_t mask) {
  if (!motorsReady || !motorsAreDshot() || cmd > 47) return;
  uint16_t v[4];
  for (int m = 0; m < 4; m++) v[m] = (mask >> m) & 1 ? cmd : 0;
  for (int i = 0; i < 10; i++) {
    dshotSend(v, -1, true);  // commands are sent with the telemetry bit set
    delayMicroseconds(1000);
  }
}
void motorsCommand(uint8_t cmd) { motorsCommandMask(cmd, 0x0F); }

// ---------------- analog ----------------
// analogReadMilliVolts() tries to create a calibration scheme on every call and logs an error each
// time on an ESP32 without an eFuse Vref (older chips, many DevKit V1 clones). One scheme per pin,
// created once: eFuse calibration when present, else line fitting with the nominal 1100 mV Vref,
// else the uncalibrated raw value.
static float adcVolts(int pin) {
  struct Cal { int pin; int state; adc_cali_handle_t h; };
  static Cal cal[2] = {{-1, 0, nullptr}, {-1, 0, nullptr}};
  Cal* c = cal[0].pin == pin || cal[0].pin < 0 ? &cal[0] : &cal[1];
#ifdef AQ_QEMU
  return 0;  // the emulator has no ADC: a conversion never finishes
#endif
  int raw = analogRead(pin);
  if (c->pin != pin) { c->pin = pin; c->state = 0; }
  if (c->state == 0) {
    c->state = 2;
    adc_unit_t unit;
    adc_channel_t ch;
    if (adc_oneshot_io_to_channel(pin, &unit, &ch) == ESP_OK) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
      adc_cali_curve_fitting_config_t cc = {};
      cc.unit_id = unit;
      cc.chan = ch;
      cc.atten = ADC_ATTEN_DB_12;
      cc.bitwidth = ADC_BITWIDTH_12;
      if (adc_cali_create_scheme_curve_fitting(&cc, &c->h) == ESP_OK) c->state = 1;
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
      adc_cali_line_fitting_config_t lc = {};
      lc.unit_id = unit;
      lc.atten = ADC_ATTEN_DB_12;
      lc.bitwidth = ADC_BITWIDTH_12;
#if CONFIG_IDF_TARGET_ESP32
      lc.default_vref = 1100;
#endif
      if (adc_cali_create_scheme_line_fitting(&lc, &c->h) == ESP_OK) c->state = 1;
#endif
    }
  }
  int mv = 0;
  if (c->state == 1 && adc_cali_raw_to_voltage(c->h, raw, &mv) == ESP_OK) return mv / 1000.0f;
  return raw * (3.1f / 4095.0f);  // 12 dB attenuation, roughly 0-3.1 V
}
float adcVbatPin() { return adcVolts(PIN_VBAT); }
float adcCurrentPin() { return PIN_CURRENT >= 0 ? adcVolts(PIN_CURRENT) : -1.0f; }

// ---------------- storage ----------------
static Preferences prefs;
static const char* slotKey(Slot s) {
  return s == Slot::CONFIG ? "cfg" : s == Slot::MISSION ? "mission" : s == Slot::RALLY ? "rally"
       : s == Slot::FENCE ? "fence" : "wifi";
}
bool storageLoad(Slot s, void* data, size_t len) {
  if (!prefs.begin("allquad", true)) return false;
  size_t n = prefs.getBytes(slotKey(s), data, len);
  prefs.end();
  return n == len;
}
void storageInfo(int& copy, uint32_t& seq, int& copies) { copy = 0; seq = 0; copies = 0; }  // NVS
bool storageSave(Slot s, const void* data, size_t len) {
  if (!prefs.begin("allquad", false)) return false;
  size_t n = prefs.putBytes(slotKey(s), data, len);
  prefs.end();
  return n == len;
}

// ---------------- blackbox in a raw flash partition ----------------
// LittleFS was replaced: its metadata updates and sector erases could stall both CPU cores for tens of
// milliseconds in flight (the ESP32 suspends the flash cache on both cores during any flash write).
static const esp_partition_t* bbPart = nullptr;
static uint32_t bbEnd = 0;        // next byte offset to program (page aligned + pageFill)
static uint32_t bbErased = 0;     // [bbEnd, bbErased) is known erased
static uint32_t bbEraseNext = 0;  // next sector to erase while disarmed (== bbErasedTarget when idle)
static uint32_t bbEraseTo = 0;
static uint8_t bbPage[256];
static size_t bbFill = 0;
static volatile uint32_t fastDoneUs = 0;
static bool bbScanned = false;

void markFastLoopDone() { fastDoneUs = (uint32_t)esp_timer_get_time(); }

static bool erasedAt(uint32_t off) {
  uint8_t b[16];
  if (esp_partition_read(bbPart, off, b, sizeof(b)) != ESP_OK) return false;
  for (uint8_t v : b) if (v != 0xFF) return false;
  return true;
}

static void bbScan() {
  if (bbScanned) return;
  bbScanned = true;
  bbPart = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, (esp_partition_subtype_t)0x40, "blackbox");
  if (!bbPart) return;
  uint32_t pages = bbPart->size / 256;
  // writes are contiguous from 0: binary search for the first erased page, continue on a new page
  uint32_t lo = 0, hi = pages;
  while (lo < hi) {
    uint32_t mid = (lo + hi) / 2;
    if (erasedAt(mid * 256) && erasedAt(mid * 256 + 240)) hi = mid; else lo = mid + 1;
  }
  bbEnd = lo * 256;
  // how far is the flash after the log really erased (an erase may have been interrupted)
  uint32_t s = bbEnd;
  while (s < bbPart->size && erasedAt(s) && erasedAt((s | 4095) - 15)) s = (s | 4095) + 1;
  bbErased = s;
  bbEraseNext = bbEraseTo = s;  // nothing to do unless an erase is requested
  if (bbErased < bbPart->size && bbEnd == 0) {  // empty log but dirty flash: finish the erase
    bbEraseNext = bbErased & ~4095u;
    bbEraseTo = bbPart->size;
  }
}

bool logOpen() {
  bbScan();
  return bbPart != nullptr;
}

static void programPage(size_t len) {
  // wait (max 1.5 ms) until the fast loop has just finished, then program inside its idle gap
  uint32_t t0 = (uint32_t)esp_timer_get_time();
  while ((uint32_t)esp_timer_get_time() - fastDoneUs > 150 && (uint32_t)esp_timer_get_time() - t0 < 1500) taskYIELD();
  esp_partition_write(bbPart, bbEnd, bbPage, len);
}

size_t logWrite(const uint8_t* d, size_t n) {
  if (!bbPart) return 0;
  size_t done = 0;
  while (done < n) {
    if (bbEnd + 256 > bbErased) return done;  // log full (or not erased yet): stop recording
    size_t k = n - done < 256 - bbFill ? n - done : 256 - bbFill;
    memcpy(bbPage + bbFill, d + done, k);
    bbFill += k;
    done += k;
    if (bbFill == 256) {
      programPage(256);
      bbEnd += 256;
      bbFill = 0;
    }
  }
  return done;
}

// partial page: program what we have (NOR flash: re-programming the same bytes later is harmless)
void logFlush() {
  if (bbPart && bbFill && bbEnd + 256 <= bbErased) {
    memset(bbPage + bbFill, 0xFF, 256 - bbFill);
    programPage(256);
  }
}

void logErase() {
  bbScan();
  if (!bbPart) return;
  bbEraseNext = 0;
  bbEraseTo = bbEnd > bbErased ? bbEnd : bbErased;  // only what was used
  if (bbEraseTo < bbPart->size && !erasedAt(bbEraseTo)) bbEraseTo = bbPart->size;
  bbEraseTo = (bbEraseTo + 4095) & ~4095u;
  if (bbEraseTo > bbPart->size) bbEraseTo = bbPart->size;
  bbEnd = 0;
  bbFill = 0;
  bbErased = 0;
}

void logService(bool armed) {
  if (!bbPart || armed || bbEraseNext >= bbEraseTo) return;
  esp_partition_erase_range(bbPart, bbEraseNext, 4096);  // ~45 ms, disarmed only (arming is blocked)
  bbEraseNext += 4096;
  if (bbEraseNext > bbErased) bbErased = bbEraseNext;      // erased area grows from the log end
  if (bbEraseNext >= bbEraseTo) bbErased = bbPart->size;   // the rest was erased before
}

bool logBusy() { return bbPart && bbEraseNext < bbEraseTo; }
uint32_t logSize() { return bbEnd + (uint32_t)bbFill; }
uint32_t logCapacity() {
  bbScan();
  return bbPart ? bbPart->size : 0;
}
bool logRead(uint32_t off, uint8_t* buf, size_t len) {
  if (!bbPart || off + len > bbPart->size) return false;
  if (esp_partition_read(bbPart, off, buf, len) != ESP_OK) return false;
  // bytes still in the RAM page buffer
  if (off + len > bbEnd && bbFill) {
    for (size_t i = 0; i < len; i++) {
      uint32_t o = off + i;
      if (o >= bbEnd && o < bbEnd + bbFill) buf[i] = bbPage[o - bbEnd];
    }
  }
  return true;
}

// ---------------- I2C ----------------
void internal::i2cStart() { Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, i2cClock()); }

// ---------------- SD card ----------------
// SPI: the second general SPI bus (the OSD bus, free when no MAX7456 is fitted). SD bus: the SDMMC host
// (ESP32 / S3 / P4; the S3 routes it to any pins, the ESP32 only to its fixed pins 14 / 15 / 2 / 4 / 12 / 13).
static fs::FS* sdFs = nullptr;
static SPIClass* sdSpi = nullptr;
[[maybe_unused]] static bool sdMmc = false;
static const char* sdType = "";
static File sdW, sdR;
static char sdRPath[40] = "";

uint8_t sdModes() {
  uint8_t m = 0;
#if SOC_SPI_PERIPH_NUM >= 3 && PIN_OSD_CS < 0
  m |= 1 << 1;
#endif
#if SOC_SDMMC_HOST_SUPPORTED
  m |= (1 << 2) | (1 << 3);
#endif
  return m;
}
bool sdPinsFixed() { return false; }
void sdDefaultPins(uint8_t p[6]) {
  const int d[6] = {PIN_SD_CLK, PIN_SD_CMD, PIN_SD_D0, PIN_SD_D1, PIN_SD_D2, PIN_SD_D3};
  for (int i = 0; i < 6; i++) p[i] = d[i] < 0 ? 255 : (uint8_t)d[i];
}

// pins the board already uses, or that the chip cannot give away (flash / PSRAM, USB, strapping)
static bool pinBusy(int p) {
  const int used[] = {PIN_MOTOR1, PIN_MOTOR2, PIN_MOTOR3, PIN_MOTOR4, PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_IMU_CS,
                      PIN_IMU2_CS, PIN_I2C_SDA, PIN_I2C_SCL, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PIN_SERIAL2_RX, PIN_SERIAL2_TX,
                      PIN_SERIAL3_RX, PIN_SERIAL3_TX, PIN_SERIAL4_RX, PIN_SERIAL4_TX, PIN_SERIAL5_RX, PIN_SERIAL5_TX, PIN_LED,
                      PIN_BUZZER, PIN_VBAT, PIN_CURRENT,
#if defined(PIN_IMU_INT)
                      PIN_IMU_INT,
#endif
#if PIN_OSD_CS >= 0
                      PIN_OSD_CS, PIN_OSD_SCK, PIN_OSD_MISO, PIN_OSD_MOSI,
#endif
                      -1};
  for (int u : used)
    if (u >= 0 && u == p) return true;
  if (!GPIO_IS_VALID_OUTPUT_GPIO(p)) return true;
#if CONFIG_IDF_TARGET_ESP32
  if (p == 0 || p == 12 || (p >= 6 && p <= 11)) return true;  // boot strap / flash voltage, SPI flash
#elif CONFIG_IDF_TARGET_ESP32S3
  if (p == 0 || p == 45 || p == 46 || p == 19 || p == 20 || (p >= 26 && p <= 32)) return true;
#if defined(BOARD_HAS_PSRAM)
  if (p >= 33 && p <= 37) return true;  // octal PSRAM
#endif
#if defined(AQ_CONSOLE_DUAL)
  if (p == 43 || p == 44) return true;  // UART0 console
#endif
#endif
  return false;
}

void sdUnmount() {
  if (sdW) sdW.close();
  if (sdR) sdR.close();
  sdRPath[0] = 0;
  if (sdFs == &SD) SD.end();
#if SOC_SDMMC_HOST_SUPPORTED
  if (sdFs == &SD_MMC) SD_MMC.end();
#endif
  if (sdSpi) sdSpi->end();
  sdFs = nullptr;
  sdType = "";
}

[[maybe_unused]] static const char* typeName(sdcard_type_t t) {
  return t == CARD_SDHC ? "SDHC/SDXC" : t == CARD_SD ? "SDSC" : t == CARD_MMC ? "MMC" : "?";
}

bool sdMount(uint8_t mode, uint8_t mhz, const uint8_t pin[6], char* err, size_t n) {
  sdUnmount();
  for (int i = 0; i < 6; i++) {
    if (pin[i] == 255) continue;
    if (pinBusy(pin[i])) { snprintf(err, n, "GPIO %d is used by the board / chip", pin[i]); return false; }
    for (int k = 0; k < i; k++)
      if (pin[k] == pin[i]) { snprintf(err, n, "GPIO %d set twice", pin[i]); return false; }
  }
  if (mode == 1) {
#if SOC_SPI_PERIPH_NUM >= 3
    if (!sdSpi) sdSpi = new SPIClass(HSPI);
    sdSpi->begin(pin[0], pin[2], pin[1], pin[5]);  // SCK, MISO (D0), MOSI (CMD), CS (D3)
    if (!SD.begin(pin[5], *sdSpi, (mhz ? mhz : 20) * 1000000u, "/sd", 2, false) || SD.cardType() == CARD_NONE) {
      SD.end();
      sdSpi->end();
      snprintf(err, n, "no card (SPI)");
      return false;
    }
    sdFs = &SD;
    sdMmc = false;
    sdType = typeName(SD.cardType());
    return true;
#endif
  }
#if SOC_SDMMC_HOST_SUPPORTED
  if (mode == 2 || mode == 3) {
    bool one = mode == 2;
    bool ok = one ? SD_MMC.setPins(pin[0], pin[1], pin[2]) : SD_MMC.setPins(pin[0], pin[1], pin[2], pin[3], pin[4], pin[5]);
    if (!ok) { snprintf(err, n, "these pins cannot carry the SD bus"); return false; }
    if (!SD_MMC.begin("/sd", one, false, mhz ? mhz * 1000 : SDMMC_FREQ_DEFAULT, 2) || SD_MMC.cardType() == CARD_NONE) {
      SD_MMC.end();
      snprintf(err, n, "no card (SD bus %d-bit)", one ? 1 : 4);
      return false;
    }
    sdFs = &SD_MMC;
    sdMmc = true;
    sdType = typeName(SD_MMC.cardType());
    return true;
  }
#endif
  snprintf(err, n, "mode not available");
  return false;
}

bool sdMounted() { return sdFs != nullptr; }
void sdSetDma(bool) {}
bool sdDmaActive() { return false; }
const char* sdCardType() { return sdType; }
bool sdSpace(uint64_t& total, uint64_t& used) {
  if (!sdFs) return false;
#if SOC_SDMMC_HOST_SUPPORTED
  if (sdMmc) { total = SD_MMC.totalBytes(); used = SD_MMC.usedBytes(); return true; }
#endif
  total = SD.totalBytes();
  used = SD.usedBytes();
  return true;
}
uint8_t* sdQueueAlloc(size_t want, size_t& got) {
  uint8_t* b = nullptr;
#if defined(BOARD_HAS_PSRAM)
  if (psramFound()) b = (uint8_t*)ps_malloc(want);  // 256 kB in PSRAM: ~8 s of a 1 kHz log
#endif
  got = b ? want : 32 * 1024;
  if (!b) b = (uint8_t*)heap_caps_malloc(got, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
  if (!b) got = 0;
  return b;
}
bool sdCreate(const char* path) {
  if (!sdFs) return false;
  const char* slash = strrchr(path, '/');
  if (slash && slash != path) {
    char dir[40];
    snprintf(dir, sizeof(dir), "%.*s", (int)(slash - path), path);
    if (!sdFs->exists(dir)) sdFs->mkdir(dir);
  }
  sdW = sdFs->open(path, FILE_WRITE);
  return (bool)sdW;
}
size_t sdAppend(const uint8_t* d, size_t n) { return sdW ? sdW.write(d, n) : 0; }
bool sdSync() {
  if (!sdW) return false;
  sdW.flush();  // fflush + fsync: length and FAT on the card
  return true;
}
void sdClose() {
  if (sdW) sdW.close();
}
void sdList(const char* dir, void (*cb)(const char*, uint32_t, void*), void* ctx) {
  if (!sdFs) return;
  File d = sdFs->open(dir);
  if (!d || !d.isDirectory()) return;
  for (File f = d.openNextFile(); f; f = d.openNextFile()) {
    if (!f.isDirectory()) cb(f.name(), (uint32_t)f.size(), ctx);
    f.close();
  }
  d.close();
}
bool sdRemove(const char* path) { return sdFs && sdFs->remove(path); }
bool sdRead(const char* path, uint32_t off, uint8_t* buf, size_t len, size_t& got, uint32_t& size) {
  got = 0;
  if (!sdFs) return false;
  if (!sdR || strcmp(path, sdRPath)) {
    if (sdR) sdR.close();
    sdR = sdFs->open(path, FILE_READ);
    if (!sdR) { sdRPath[0] = 0; return false; }
    snprintf(sdRPath, sizeof(sdRPath), "%s", path);
  }
  size = (uint32_t)sdR.size();
  if (off >= size) return true;
  if (sdR.position() != off && !sdR.seek(off)) return false;
  got = sdR.read(buf, len);
  return true;
}
void sdReadClose() {
  if (sdR) sdR.close();
  sdRPath[0] = 0;
}

// ---------------- firmware slots / rollback ----------------
// The Arduino core would confirm a freshly updated image as soon as it boots; returning true here
// leaves it "pending verify", and the firmware confirms itself only after its self-test
// (app.cpp: tasks running for 15 s). A crash or reset before that boots the previous image again.
}  // namespace hal
extern "C" bool verifyRollbackLater() { return true; }
namespace hal {

FirmwareInfo firmwareInfo() {
  FirmwareInfo f{};
  const esp_partition_t* run = esp_ota_get_running_partition();
  f.running = run ? run->label : "?";
  f.otaSupported = esp_ota_get_next_update_partition(nullptr) != nullptr;
  esp_ota_img_states_t st;
  f.pendingVerify = run && esp_ota_get_state_partition(run, &st) == ESP_OK && st == ESP_OTA_IMG_PENDING_VERIFY;
  return f;
}
void firmwareConfirm() {
  if (firmwareInfo().pendingVerify) esp_ota_mark_app_valid_cancel_rollback();
}
void firmwareReject() { esp_ota_mark_app_invalid_rollback_and_reboot(); }

void reboot() { ESP.restart(); }

// ---------------- bootloader ----------------
bool bootloaderSupported() {
#if CONFIG_IDF_TARGET_ESP32S3
  return true;  // ROM download mode over the same USB port (USB-Serial-JTAG)
#else
  return false;  // ESP32: the flasher resets into the ROM loader with DTR/RTS (auto-reset circuit)
#endif
}
void rebootToBootloader() {
#if CONFIG_IDF_TARGET_ESP32S3
  REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
  esp_restart();
#endif
}

// ---------------- ESC passthrough: one-wire serial on the motor pins ----------------
static bool escActive = false;
static portMUX_TYPE escMux = portMUX_INITIALIZER_UNLOCKED;

bool escSerialBegin() {
  if (escActive) return true;
  for (int i = 0; i < 4; i++) {
    if (motorsReady) {
      if (bidir) {
        rmt_disable(bdTx[i]); rmt_del_channel(bdTx[i]); rmt_del_encoder(bdEnc[i]);
        rmt_disable(bdRx[i]); rmt_del_channel(bdRx[i]);
        bdRxBusy[i] = false;
        bdRxDone[i] = false;
      } else if (motorsAreDshot()) {
        rmtDeinit(motorPins[i]);
      } else {
        ledcDetach(motorPins[i]);
      }
    }
    pinMode(motorPins[i], INPUT_PULLUP);
  }
  motorsReady = false;
  escActive = true;
  return true;
}

void escSerialEnd() {
  if (!escActive) return;
  escActive = false;
  motorsInit(motorProto, 0, bidirWanted);
}

static inline void waitCycles(uint32_t until) {
  while ((int32_t)(fc::cycles() - until) < 0) {}
}

// 19200 baud, 8N1, LSB first; interrupts off for each byte (~0.52 ms) so WiFi cannot stretch a bit
void escSerialWrite(int esc, const uint8_t* d, size_t n) {
  if (!escActive || esc < 0 || esc > 3) return;
  gpio_num_t pin = (gpio_num_t)motorPins[esc];
  const uint32_t bit = (uint32_t)(fc::cyclesPerUs() * 52.083f);
  gpio_set_level(pin, 1);
  pinMode(motorPins[esc], OUTPUT);
  for (size_t i = 0; i < n; i++) {
    uint16_t frame = (uint16_t)((d[i] << 1) | (1u << 9));  // start 0, 8 data, stop 1
    portENTER_CRITICAL(&escMux);
    uint32_t t = fc::cycles();
    for (int k = 0; k < 10; k++) {
      gpio_set_level(pin, (frame >> k) & 1);
      t += bit;
      waitCycles(t);
    }
    portEXIT_CRITICAL(&escMux);
  }
  pinMode(motorPins[esc], INPUT_PULLUP);
}

int escSerialRead(int esc, uint32_t timeoutMs) {
  if (!escActive || esc < 0 || esc > 3) return -1;
  gpio_num_t pin = (gpio_num_t)motorPins[esc];
  const uint32_t bit = (uint32_t)(fc::cyclesPerUs() * 52.083f);
  uint32_t t0 = millis();
  for (;;) {
    // look for the start bit with interrupts off, in 1 ms slices (exact sampling, no watchdog trouble)
    portENTER_CRITICAL(&escMux);
    uint32_t sliceEnd = fc::cycles() + (uint32_t)(fc::cyclesPerUs() * 1000);
    while (gpio_get_level(pin) && (int32_t)(fc::cycles() - sliceEnd) < 0) {}
    if (!gpio_get_level(pin)) {
      uint32_t t = fc::cycles() + bit / 2;  // middle of the start bit
      waitCycles(t);
      int v = -1;
      if (!gpio_get_level(pin)) {
        uint8_t b = 0;
        for (int k = 0; k < 8; k++) {
          t += bit;
          waitCycles(t);
          if (gpio_get_level(pin)) b |= (uint8_t)(1u << k);
        }
        t += bit;
        waitCycles(t);
        if (gpio_get_level(pin)) v = b;  // stop bit
      }
      portEXIT_CRITICAL(&escMux);
      return v;
    }
    portEXIT_CRITICAL(&escMux);
    if (millis() - t0 >= timeoutMs) return -1;
  }
}

// ---------------- LED strip (WS2812 via RMT, 10 MHz ticks) ----------------
static int ledPin = -1;
static rmt_data_t ledData[32 * 24];
bool ledStripBegin(uint8_t pin, uint8_t count) {
  if (!pin || count == 0 || count > 32) return false;
  if (pinBusy(pin)) return false;  // motors, a serial port, the IMU bus ... or a pin the chip keeps (flash / PSRAM / USB)
  // ESP32 has 8 RMT TX channels (DShot uses 4); the S3 has only 4, so with DShot there is none left
  if (bidir) return false;  // bidirectional DShot uses every RMT channel
  if (!rmtInit(pin, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 10000000)) return false;
  ledPin = pin;
  return true;
}
void ledStripShow(const uint8_t* grb, uint8_t count) {
  if (ledPin < 0 || count > 32) return;
  int k = 0;
  for (int i = 0; i < count * 3; i++)
    for (int b = 7; b >= 0; b--) {
      bool one = grb[i] & (1 << b);  // WS2812: 0 = 0.4/0.85 us, 1 = 0.8/0.45 us
      ledData[k].level0 = 1;
      ledData[k].duration0 = one ? 8 : 4;
      ledData[k].level1 = 0;
      ledData[k].duration1 = one ? 4 : 8;
      k++;
    }
  rmtWriteAsync(ledPin, ledData, k);  // the 50 us reset gap is the idle time until the next frame
}

}  // namespace hal
#endif
