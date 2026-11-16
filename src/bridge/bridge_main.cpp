// AllQuad WiFi bridge - ESP32-C3 (SuperMini / DevKitM) between a phone or laptop and a flight
// controller without WiFi (STM32). The FC's UART with the "Configurator" function (serialN_fn 11,
// 460800 baud) is wired to the bridge; the bridge opens a WiFi access point and serves the same
// configurator page, which talks to the FC through a WebSocket (/link) as it would over USB.
//   pio run -e bridge_esp32c3 -t upload
// Wiring: bridge TX (GPIO 21) -> FC RX, bridge RX (GPIO 20) <- FC TX, GND - GND, 5 V or 3.3 V supply.
// WiFi "AllQuad-Link-XXXX", password allquad123 (change it on the page: System tab). Then open
// http://192.168.4.1 (most phones open it by themselves: captive portal).
#if defined(AQ_BRIDGE)
#include <Arduino.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_http_server.h>
#include <unistd.h>

#include "../web/web_ui_data.h"

#ifndef BRIDGE_RX
#define BRIDGE_RX 20
#endif
#ifndef BRIDGE_TX
#define BRIDGE_TX 21
#endif
#ifndef BRIDGE_BAUD
#define BRIDGE_BAUD 460800
#endif
#ifndef BRIDGE_LED
#define BRIDGE_LED 8  // ESP32-C3 SuperMini blue LED (active low)
#endif

namespace {
constexpr const char* VERSION = "1.0";
constexpr const char* DEFAULT_PASS = "allquad123";
httpd_handle_t server = nullptr;
DNSServer dns;
Preferences prefs;
char ssid[33], pass[65];
volatile int linkFd = -1;          // the one WebSocket client (a newer one replaces it)
volatile bool sendQueued = false;  // a send job waits in the server task
uint32_t rxBytes = 0, txBytes = 0;

void loadWifi() {
  prefs.begin("bridge", true);
  String s = prefs.getString("ssid", ""), p = prefs.getString("pass", "");
  prefs.end();
  uint8_t mac[6];
  WiFi.macAddress(mac);
  if (s.length()) snprintf(ssid, sizeof(ssid), "%s", s.c_str());
  else snprintf(ssid, sizeof(ssid), "AllQuad-Link-%02X%02X", mac[4], mac[5]);
  snprintf(pass, sizeof(pass), "%s", p.length() >= 8 ? p.c_str() : DEFAULT_PASS);
}

esp_err_t reply(httpd_req_t* req, const char* status, const char* type, const char* body) {
  httpd_resp_set_status(req, status);
  httpd_resp_set_type(req, type);
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_sendstr(req, body);
}

// "/" -> the page with ?bridge (it then connects through /link by itself)
esp_err_t onIndex(httpd_req_t* req) {
  if (httpd_req_get_url_query_len(req) == 0) {
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/?bridge");
    return httpd_resp_send(req, nullptr, 0);
  }
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  return httpd_resp_send(req, (const char*)WEB_UI_GZ, WEB_UI_GZ_LEN);
}

esp_err_t onInfo(httpd_req_t* req) {
  char b[200];
  snprintf(b, sizeof(b), "{\"ok\":true,\"fw\":\"%s\",\"ssid\":\"%s\",\"baud\":%d,\"defpw\":%s,\"rx\":%lu,\"tx\":%lu}", VERSION,
           ssid, BRIDGE_BAUD, strcmp(pass, DEFAULT_PASS) ? "false" : "true", (unsigned long)rxBytes, (unsigned long)txBytes);
  return reply(req, "200 OK", "application/json", b);
}

// POST "ssid\npassword" (8-63 characters): saved, the access point restarts with it
esp_err_t onWifi(httpd_req_t* req) {
  char b[120];
  int n = req->content_len < sizeof(b) - 1 ? httpd_req_recv(req, b, req->content_len) : -1;
  if (n <= 0) return reply(req, "400 Bad Request", "text/plain", "bad request");
  b[n] = 0;
  char* nl = strchr(b, '\n');
  if (!nl) return reply(req, "400 Bad Request", "text/plain", "ssid\\npassword");
  *nl = 0;
  const char* s = b;
  const char* p = nl + 1;
  size_t sl = strlen(s), pl = strlen(p);
  if (sl < 1 || sl > 32 || pl < 8 || pl > 63) return reply(req, "400 Bad Request", "text/plain", "name 1-32, password 8-63 characters");
  prefs.begin("bridge", false);
  prefs.putString("ssid", s);
  prefs.putString("pass", p);
  prefs.end();
  reply(req, "200 OK", "text/plain", "saved: the bridge restarts with the new WiFi");
  delay(300);
  ESP.restart();
  return ESP_OK;
}

// phones probe these to find a captive portal: send them to the page
esp_err_t onProbe(httpd_req_t* req) {
  httpd_resp_set_status(req, "302 Found");
  httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/?bridge");
  return httpd_resp_send(req, nullptr, 0);
}
esp_err_t notFound(httpd_req_t* req, httpd_err_code_t) { return onProbe(req); }

// WebSocket /link: binary frames are raw bytes for the FC UART
esp_err_t onLink(httpd_req_t* req) {
  if (req->method == HTTP_GET) {  // handshake: this client becomes the link
    int fd = httpd_req_to_sockfd(req);
    int old = linkFd;
    linkFd = fd;
    if (old >= 0 && old != fd) httpd_sess_trigger_close(server, old);
    while (Serial1.available()) Serial1.read();  // stale bytes of the previous client
    return ESP_OK;
  }
  httpd_ws_frame_t f = {};
  if (httpd_ws_recv_frame(req, &f, 0) != ESP_OK) return ESP_FAIL;
  if (!f.len) return ESP_OK;
  if (f.len > 16384) return ESP_FAIL;
  uint8_t* buf = (uint8_t*)malloc(f.len);
  if (!buf) return ESP_ERR_NO_MEM;
  f.payload = buf;
  esp_err_t r = httpd_ws_recv_frame(req, &f, f.len);
  if (r == ESP_OK && (f.type == HTTPD_WS_TYPE_BINARY || f.type == HTTPD_WS_TYPE_TEXT)) {
    Serial1.write(buf, f.len);
    txBytes += f.len;
  }
  free(buf);
  return r;
}

// runs in the server task: what the FC sent goes to the client in one frame
void sendWork(void*) {
  static uint8_t b[2048];
  size_t n = 0;
  while (n < sizeof(b) && Serial1.available()) b[n++] = (uint8_t)Serial1.read();
  sendQueued = false;
  int fd = linkFd;
  if (!n || fd < 0) return;
  if (httpd_ws_get_fd_info(server, fd) != HTTPD_WS_CLIENT_WEBSOCKET) { linkFd = -1; return; }
  httpd_ws_frame_t f = {};
  f.type = HTTPD_WS_TYPE_BINARY;
  f.payload = b;
  f.len = n;
  if (httpd_ws_send_frame_async(server, fd, &f) == ESP_OK) rxBytes += n;
}

void closeFn(httpd_handle_t, int fd) {
  if (fd == linkFd) linkFd = -1;
  close(fd);
}

void startServer() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.max_uri_handlers = 12;
  cfg.lru_purge_enable = true;
  cfg.close_fn = closeFn;
  cfg.stack_size = 6144;
  if (httpd_start(&server, &cfg) != ESP_OK) { server = nullptr; return; }
  const httpd_uri_t routes[] = {
      {"/", HTTP_GET, onIndex, nullptr},
      {"/bridge/info", HTTP_GET, onInfo, nullptr},
      {"/bridge/wifi", HTTP_POST, onWifi, nullptr},
      {"/generate_204", HTTP_GET, onProbe, nullptr},
      {"/hotspot-detect.html", HTTP_GET, onProbe, nullptr},
      {"/connecttest.txt", HTTP_GET, onProbe, nullptr},
  };
  for (const httpd_uri_t& r : routes) httpd_register_uri_handler(server, &r);
  httpd_uri_t ws = {};
  ws.uri = "/link";
  ws.method = HTTP_GET;
  ws.handler = onLink;
  ws.is_websocket = true;
  httpd_register_uri_handler(server, &ws);
  httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, notFound);
}
}  // namespace

void setup() {
  pinMode(BRIDGE_LED, OUTPUT);
  digitalWrite(BRIDGE_LED, HIGH);
  Serial1.setRxBufferSize(8192);
  Serial1.begin(BRIDGE_BAUD, SERIAL_8N1, BRIDGE_RX, BRIDGE_TX);
  WiFi.mode(WIFI_AP);
  loadWifi();
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAP(ssid, pass, 6, 0, 2);
  WiFi.setSleep(false);  // lower latency for the configurator link
  dns.start(53, "*", IPAddress(192, 168, 4, 1));
  startServer();
}

void loop() {
  dns.processNextRequest();
  if (!sendQueued && linkFd >= 0 && Serial1.available()) {
    sendQueued = true;
    if (httpd_queue_work(server, sendWork, nullptr) != ESP_OK) sendQueued = false;
  }
  // LED: on while a page is connected, slow blink while waiting
  digitalWrite(BRIDGE_LED, linkFd >= 0 ? LOW : ((millis() / 500) % 2 ? LOW : HIGH));
  delay(1);
}
#endif
