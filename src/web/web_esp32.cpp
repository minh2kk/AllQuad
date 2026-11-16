// AllQuad - ESP32 HTTP server for the web configurator (ESP-IDF esp_http_server).
//
// Why not the Arduino WebServer: it serves one client at a time, closes the connection after every
// request (a new TCP handshake for every 200 ms status poll) and blocks while waiting for slow
// clients. esp_http_server keeps connections alive, serves several sockets from one small task
// and supports WebSocket, so live status is *pushed* (10 Hz, 2 Hz while armed) over one connection.
//
// Everything runs on core 0 at low priority; the flight-critical fast task owns core 1.
// Access: every /api route and the WebSocket need a session cookie from POST /api/login (web password,
// by default the WiFi password); 5 wrong passwords lock the login for 30 s. No CORS: other web sites
// opened on the phone cannot talk to the drone (SameSite=Strict cookie).
// While armed and MAVLink over WiFi is not used, the access point is switched off (wifi_armed 0):
// no radio interference with a 2.4 GHz RC link and no WiFi interrupts in flight.
#if defined(ARDUINO) && defined(ESP32) && !defined(ALLQUAD_HOST)
#include "../common/size_opt.h"  // -Os on ESP32 (not flight critical)
#include <Arduino.h>
#include <esp_http_server.h>
#include <esp_app_desc.h>
#include <esp_image_format.h>
#include <esp_ota_ops.h>

#include "../bus/bus.h"
#include "../hal/hal.h"
#include "credentials.h"
#include "web.h"
#include "webapi.h"
#include "../logging/sdlog.h"
#include "web_ui_data.h"
#include "webapi.h"
#include "../logging/sdlog.h"

namespace web {

static httpd_handle_t server = nullptr;

// ---------------------------------------------------------------- helpers
static void chunkSink(const char* d, size_t n, void* ctx) {
  httpd_resp_send_chunk(static_cast<httpd_req_t*>(ctx), d, n);
}

static esp_err_t sendJson(httpd_req_t* req, void (*fn)(Sink, void*)) {
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  fn(chunkSink, req);
  return httpd_resp_send_chunk(req, nullptr, 0);
}

static esp_err_t reply(httpd_req_t* req, int code, const char* msg) {
  if (code == 205) code = 200;  // "rebooting into the bootloader": a plain success for the page
  const char* st = code == 200 ? "200 OK" : code == 202 ? "202 Accepted" : code == 400 ? "400 Bad Request"
                 : code == 401 ? "401 Unauthorized" : code == 205 ? "200 OK" : code == 404 ? "404 Not Found" : code == 429 ? "429 Too Many Requests" : code == 409 ? "409 Conflict"
                 : "423 Locked";
  char b[160];
  snprintf(b, sizeof(b), "{\"ok\":%s,\"msg\":\"%s\"}", code < 300 ? "true" : "false", msg);
  httpd_resp_set_status(req, st);
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_sendstr(req, b);
}

static int hexv(char c) { return c >= '0' && c <= '9' ? c - '0' : (c | 32) >= 'a' && (c | 32) <= 'f' ? (c | 32) - 'a' + 10 : -1; }
static void urlDecode(char* s) {
  char* o = s;
  for (; *s; s++) {
    if (*s == '+') *o++ = ' ';
    else if (*s == '%' && hexv(s[1]) >= 0 && hexv(s[2]) >= 0) { *o++ = (char)(hexv(s[1]) * 16 + hexv(s[2])); s += 2; }
    else *o++ = *s;
  }
  *o = 0;
}

// session cookie check; every /api route except /api/login goes through it
static bool authed(httpd_req_t* req) {
  char cookie[160];
  size_t n = httpd_req_get_hdr_value_len(req, "Cookie");
  if (!n || n >= sizeof(cookie) || httpd_req_get_hdr_value_str(req, "Cookie", cookie, sizeof(cookie)) != ESP_OK)
    return false;
  return authorized(cookie, millis());
}
#define REQUIRE_AUTH(req) \
  if (!authed(req)) return reply(req, 401, "login required")

// application/x-www-form-urlencoded body (small forms only)
static bool body(httpd_req_t* req, char* buf, size_t n) {
  if (req->content_len == 0 || req->content_len >= n) return false;
  size_t got = 0;
  while (got < req->content_len) {
    int r = httpd_req_recv(req, buf + got, req->content_len - got);
    if (r <= 0) return false;
    got += (size_t)r;
  }
  buf[got] = 0;
  return true;
}
static bool field(const char* form, const char* key, char* out, size_t n) {
  if (httpd_query_key_value(form, key, out, n) != ESP_OK) return false;
  urlDecode(out);
  return true;
}

// query string into a buffer (nullptr if none)
static char* query(httpd_req_t* req, char* buf, size_t n) {
  size_t len = httpd_req_get_url_query_len(req);
  if (!len || len >= n || httpd_req_get_url_query_str(req, buf, n) != ESP_OK) return nullptr;
  return buf;
}

// ---------------------------------------------------------------- routes
static esp_err_t onIndex(httpd_req_t* req) {
  // ETag = hash of the page: a reload costs a 304 (no body) instead of ~90 kB over WiFi, and a new
  // firmware's page is picked up at once (no stale max-age copy)
  httpd_resp_set_hdr(req, "ETag", WEB_UI_ETAG);
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  char inm[40];
  if (httpd_req_get_hdr_value_str(req, "If-None-Match", inm, sizeof(inm)) == ESP_OK && !strcmp(inm, WEB_UI_ETAG)) {
    httpd_resp_set_status(req, "304 Not Modified");
    return httpd_resp_send(req, nullptr, 0);
  }
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
  return httpd_resp_send(req, (const char*)WEB_UI_GZ, WEB_UI_GZ_LEN);
}
static esp_err_t onParams(httpd_req_t* req) { REQUIRE_AUTH(req); return sendJson(req, paramsJson); }
static esp_err_t onStatus(httpd_req_t* req) { REQUIRE_AUTH(req); return sendJson(req, [](Sink k, void* c) { statusJson(k, c, true); }); }
static esp_err_t onMission(httpd_req_t* req) { REQUIRE_AUTH(req); return sendJson(req, missionJson); }

static esp_err_t onLogin(httpd_req_t* req) {
  char form[160], pw[70] = "";
  if (body(req, form, sizeof(form))) field(form, "pw", pw, sizeof(pw));
  char token[33];
  int r = login(pw, millis(), token);
  if (r != 200) return reply(req, r, r == 429 ? "too many attempts, wait 30 s" : "wrong password");
  char cookie[80];
  snprintf(cookie, sizeof(cookie), "aq=%s; Path=/; HttpOnly; SameSite=Strict", token);
  httpd_resp_set_hdr(req, "Set-Cookie", cookie);
  return reply(req, 200, creds::usingDefaultPassword() ? "default password" : "ok");
}

static esp_err_t onLogout(httpd_req_t* req) {
  char cookie[160];
  if (httpd_req_get_hdr_value_str(req, "Cookie", cookie, sizeof(cookie)) == ESP_OK) logout(cookie);
  httpd_resp_set_hdr(req, "Set-Cookie", "aq=; Path=/; Max-Age=0");
  return reply(req, 200, "logged out");
}

static void rebootLater(void*) {
  vTaskDelay(pdMS_TO_TICKS(300));
  hal::reboot();
}

// ---- backup / restore
static esp_err_t onBackup(httpd_req_t* req) {
  REQUIRE_AUTH(req);
  char q[16], all[4] = "";
  if (query(req, q, sizeof(q))) httpd_query_key_value(q, "all", all, sizeof(all));
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=allquad-backup.txt");
  backupText(chunkSink, req, all[0] == '1');
  return httpd_resp_send_chunk(req, nullptr, 0);
}

static esp_err_t onRestore(httpd_req_t* req) {
  REQUIRE_AUTH(req);
  if (armed()) return reply(req, 423, "disarm first");
  if (req->content_len == 0 || req->content_len > 24 * 1024) return reply(req, 400, "file empty or too large");
  char* text = (char*)malloc(req->content_len + 1);  // only for this request (a full dump is ~8 kB)
  if (!text) return reply(req, 400, "out of memory");
  size_t got = 0;
  while (got < req->content_len) {
    int r = httpd_req_recv(req, text + got, req->content_len - got);
    if (r <= 0) { free(text); return reply(req, 400, "upload interrupted"); }
    got += (size_t)r;
  }
  char msg[120];
  int r = restoreText(text, got, msg, sizeof(msg));
  free(text);
  return reply(req, r, msg);
}

// ---- firmware update: streamed into the inactive slot, verified, then booted once with rollback
static esp_err_t onOta(httpd_req_t* req) {
  REQUIRE_AUTH(req);
  if (armed()) return reply(req, 423, "disarm first");
  const esp_partition_t* slot = esp_ota_get_next_update_partition(nullptr);
  if (!slot) return reply(req, 400, "no update slot (flash the OTA partition table over USB once)");
  if (req->content_len < 64 * 1024 || req->content_len > slot->size) return reply(req, 400, "not a firmware image");
  bus::otaActive = true;  // arming blocked while the flash is being written
  esp_ota_handle_t h = 0;
  esp_err_t err = esp_ota_begin(slot, OTA_WITH_SEQUENTIAL_WRITES, &h);
  char* buf = (char*)malloc(4096);
  if (!buf) { bus::otaActive = false; esp_ota_abort(h); return reply(req, 400, "out of memory"); }
  const size_t bufSize = 4096;
  size_t got = 0;
  bool headerOk = false;
  int timeouts = 0;
  while (err == ESP_OK && got < req->content_len) {
    // fill the buffer completely (the first block must hold the whole image header)
    size_t want = req->content_len - got < bufSize ? req->content_len - got : bufSize, have = 0;
    while (have < want) {
      int r = httpd_req_recv(req, buf + have, want - have);
      if (r == HTTPD_SOCK_ERR_TIMEOUT && ++timeouts < 10) continue;
      if (r <= 0) { err = ESP_FAIL; break; }
      have += (size_t)r;
    }
    if (err != ESP_OK) break;
    if (!got) {
      // image header (magic 0xE9) and app descriptor of an ESP-IDF application
      const esp_app_desc_t* d =
          (const esp_app_desc_t*)(buf + sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t));
      headerOk = (uint8_t)buf[0] == ESP_IMAGE_HEADER_MAGIC && d->magic_word == ESP_APP_DESC_MAGIC_WORD;
      if (!headerOk) { err = ESP_ERR_INVALID_ARG; break; }
    }
    err = esp_ota_write(h, buf, have);
    got += have;
  }
  free(buf);
  if (err == ESP_OK) err = esp_ota_end(h);  // checks the image (checksum, SHA, chip)
  else if (h) esp_ota_abort(h);
  if (err == ESP_OK) err = esp_ota_set_boot_partition(slot);
  bus::otaActive = false;
  if (err != ESP_OK) return reply(req, 400, headerOk ? "update failed, the current firmware stays" : "not an ESP32 firmware image");
  esp_err_t e = reply(req, 200, "update written, rebooting");
  xTaskCreatePinnedToCore(rebootLater, "reboot", 2048, nullptr, 1, nullptr, 0);
  return e;
}

static void restartAp(void*) {
  vTaskDelay(pdMS_TO_TICKS(500));  // let the reply leave first
  hal::wifiStart(creds::ssid(), creds::wifiPass());  // same channel, new name/password
  vTaskDelete(nullptr);
}

static esp_err_t onWifi(httpd_req_t* req) {
  REQUIRE_AUTH(req);
  char form[320], ssid[40] = "", wp[70] = "", webp[70] = "", cur[70] = "";
  if (!body(req, form, sizeof(form))) return reply(req, 400, "empty form");
  bool hasSsid = field(form, "ssid", ssid, sizeof(ssid));
  field(form, "wifi", wp, sizeof(wp));
  bool hasWeb = field(form, "web", webp, sizeof(webp));
  field(form, "cur", cur, sizeof(cur));
  char msg[64];
  int r = changeWifi(hasSsid ? ssid : nullptr, wp, hasWeb ? webp : nullptr, cur, msg, sizeof(msg));
  esp_err_t e = reply(req, r, msg);
  if (r == 200) xTaskCreatePinnedToCore(restartAp, "aprestart", 3072, nullptr, 1, nullptr, 0);
  return e;
}

static esp_err_t onSet(httpd_req_t* req) {
  REQUIRE_AUTH(req);
  static char q[1024];
  if (!query(req, q, sizeof(q))) return reply(req, 400, "no parameters");
  int worst = 200;
  char failed[96] = "";
  for (char* save = nullptr, *kv = strtok_r(q, "&", &save); kv; kv = strtok_r(nullptr, "&", &save)) {
    char* eq = strchr(kv, '=');
    if (!eq) continue;
    *eq = 0;
    urlDecode(kv);
    urlDecode(eq + 1);
    int r = setParam(kv, eq + 1);
    if (r != 200) {
      worst = r;
      strncat(failed, kv, sizeof(failed) - strlen(failed) - 2);
      strncat(failed, " ", sizeof(failed) - strlen(failed) - 1);
    }
  }
  return reply(req, worst, worst == 200 ? "applied (not saved)" : (worst == 423 ? "disarm first" : failed));
}

static esp_err_t onSave(httpd_req_t* req) {
  REQUIRE_AUTH(req);
  bool ok = save();
  return reply(req, ok ? 200 : 423, ok ? "saved" : "disarm first");
}


static esp_err_t onCmd(httpd_req_t* req) {
  REQUIRE_AUTH(req);
  char q[64], c[24] = "";
  if (query(req, q, sizeof(q))) httpd_query_key_value(q, "c", c, sizeof(c));
  char msg[64];
  int r = command(c, msg, sizeof(msg));
  esp_err_t e = reply(req, r, msg);
  if (r == 202) xTaskCreatePinnedToCore(rebootLater, "reboot", 2048, nullptr, 1, nullptr, 0);
  if (r == 205) xTaskCreatePinnedToCore([](void*) { vTaskDelay(pdMS_TO_TICKS(300)); hal::rebootToBootloader(); }, "boot", 2048,
                                        nullptr, 1, nullptr, 0);
  return e;
}

static esp_err_t onBlackbox(httpd_req_t* req) {
  REQUIRE_AUTH(req);
  if (armed()) return reply(req, 423, "disarm first");
  hal::logFlush();
  uint32_t size = hal::logSize();
  if (!size) return reply(req, 404, "no log");
  httpd_resp_set_type(req, "application/octet-stream");
  httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=blackbox.bin");
  uint8_t* chunk = (uint8_t*)malloc(4096);  // one flash sector per read, sent as one HTTP chunk
  if (!chunk) return reply(req, 400, "out of memory");
  for (uint32_t off = 0; off < size; off += 4096) {
    size_t n = size - off < 4096 ? size - off : 4096;
    if (!hal::logRead(off, chunk, n) || httpd_resp_send_chunk(req, (const char*)chunk, n) != ESP_OK) {
      free(chunk);
      return ESP_FAIL;  // client went away
    }
  }
  free(chunk);
  return httpd_resp_send_chunk(req, nullptr, 0);
}

// ---------------------------------------------------------------- WebSocket: live status push
static volatile bool wsSlow = false;
static esp_err_t onWs(httpd_req_t* req) {
  if (req->method == HTTP_GET) {  // handshake: only for a logged-in browser
    if (!authed(req)) { reply(req, 401, "login required"); return ESP_FAIL; }
    return ESP_OK;
  }
  httpd_ws_frame_t f = {};
  uint8_t buf[64];
  f.payload = buf;
  if (httpd_ws_recv_frame(req, &f, 0) != ESP_OK) return ESP_FAIL;  // frame length
  if (f.len && f.len < sizeof(buf) && httpd_ws_recv_frame(req, &f, f.len) == ESP_OK && f.type == HTTPD_WS_TYPE_TEXT) {
    // "r2" / "r10": the page's open tab needs slow or fast updates (last request wins)
    buf[f.len] = 0;
    if (buf[0] == 'r') wsSlow = atoi((const char*)buf + 1) <= 2;
  }
  return ESP_OK;
}

struct Frame {
  char data[4096];
  size_t len;
  bool overflow;
};
static Frame frame;
static int lastWsClients = 0;
static uint32_t pushCount = 0;
static void frameSink(const char* d, size_t n, void* ctx) {
  Frame& f = *static_cast<Frame*>(ctx);
  if (f.len + n > sizeof(f.data)) { f.overflow = true; return; }
  memcpy(f.data + f.len, d, n);
  f.len += n;
}

// runs inside the server task (httpd_queue_work): no locking against the request handlers
static void pushWork(void*) {
  size_t n = 5;
  int fds[5];
  if (httpd_get_client_list(server, &n, fds) != ESP_OK) return;
  int ws = 0;
  for (size_t i = 0; i < n; i++)
    if (httpd_ws_get_fd_info(server, fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET) ws++;
  if (!ws) { lastWsClients = 0; return; }  // nobody listening: do not even build the JSON
  // the slow part (tasks, profile, firmware, ...) once per second and right after a new client
  // connects; that halves the bytes on the air at 10 Hz
  bool full = ws > lastWsClients || ++pushCount >= (bus::status.get().armed ? 2u : 10u);
  if (full) pushCount = 0;
  lastWsClients = ws;
  frame.len = 0;
  frame.overflow = false;
  statusJson(frameSink, &frame, full);
  if (frame.overflow) return;
  httpd_ws_frame_t f = {};
  f.type = HTTPD_WS_TYPE_TEXT;
  f.payload = (uint8_t*)frame.data;
  f.len = frame.len;
  for (size_t i = 0; i < n; i++)
    if (httpd_ws_get_fd_info(server, fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET) httpd_ws_send_frame_async(server, fds[i], &f);
}

static esp_err_t onSensors(httpd_req_t* req) { REQUIRE_AUTH(req); return sendJson(req, sensorsJson); }
static esp_err_t onSd(httpd_req_t* req) { REQUIRE_AUTH(req); return sendJson(req, sdJson); }

// one log file from the SD card, streamed in 4 kB pieces (disarmed, no log being written)
static esp_err_t onSdFile(httpd_req_t* req) {
  REQUIRE_AUTH(req);
  if (armed()) return reply(req, 423, "disarm first");
  char q[48], name[16] = "";
  if (query(req, q, sizeof(q))) httpd_query_key_value(q, "n", name, sizeof(name));
  if (!sdlog::validName(name)) return reply(req, 400, "bad file name");
  uint8_t* chunk = (uint8_t*)malloc(4096);
  if (!chunk) return reply(req, 400, "out of memory");
  size_t got = 0;
  uint32_t size = 0;
  if (!sdlog::read(name, 0, chunk, 4096, got, size)) { free(chunk); return reply(req, 409, "no card / busy / no file"); }
  httpd_resp_set_type(req, "application/octet-stream");
  char disp[64];
  snprintf(disp, sizeof(disp), "attachment; filename=%s", name);
  httpd_resp_set_hdr(req, "Content-Disposition", disp);
  uint32_t off = 0;
  while (got) {
    if (httpd_resp_send_chunk(req, (const char*)chunk, got) != ESP_OK) { free(chunk); return ESP_FAIL; }
    off += (uint32_t)got;
    if (off >= size || !sdlog::read(name, off, chunk, 4096, got, size)) break;
  }
  free(chunk);
  return httpd_resp_send_chunk(req, nullptr, 0);
}

static esp_err_t onCli(httpd_req_t* req) {
  REQUIRE_AUTH(req);
  char line[160];
  if (!body(req, line, sizeof(line))) return reply(req, 400, "empty command");
  httpd_resp_set_type(req, "text/plain; charset=utf-8");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  cliExec(line, chunkSink, req, false);  // over WiFi: the "wifi" command stays USB only
  return httpd_resp_send_chunk(req, nullptr, 0);
}

static esp_err_t onMotor(httpd_req_t* req) {
  REQUIRE_AUTH(req);
  char q[96], v[16];
  float m[4] = {0, 0, 0, 0};
  if (query(req, q, sizeof(q))) {
    char k[3] = "m1";
    for (int i = 0; i < 4; i++) {
      k[1] = (char)('1' + i);
      if (httpd_query_key_value(q, k, v, sizeof(v)) == ESP_OK) m[i] = (float)atof(v);
    }
  }
  char msg[40];
  int r = motorTest(m, msg, sizeof(msg));
  return reply(req, r, msg);
}

static esp_err_t notFound(httpd_req_t* req, httpd_err_code_t) { return reply(req, 404, "not found"); }

bool start() {
  if (server) return true;
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.core_id = 0;                // never on the flight core
  cfg.task_priority = 2;          // below io (5) and sensors (3)
  cfg.stack_size = 6144;
  cfg.max_open_sockets = 5;       // a phone opens 2-4 connections
  cfg.max_uri_handlers = 22;
  cfg.lru_purge_enable = true;    // a new client may evict an idle one
  cfg.recv_wait_timeout = 3;      // s: a stalled client cannot hold the server
  cfg.send_wait_timeout = 3;
  if (httpd_start(&server, &cfg) != ESP_OK) { server = nullptr; return false; }
  const httpd_uri_t routes[] = {
      {"/", HTTP_GET, onIndex, nullptr},
      {"/api/params", HTTP_GET, onParams, nullptr},
      {"/api/status", HTTP_GET, onStatus, nullptr},
      {"/api/mission", HTTP_GET, onMission, nullptr},
      {"/api/set", HTTP_POST, onSet, nullptr},
      {"/api/save", HTTP_POST, onSave, nullptr},
      {"/api/cmd", HTTP_POST, onCmd, nullptr},
      {"/api/blackbox", HTTP_GET, onBlackbox, nullptr},
      {"/api/login", HTTP_POST, onLogin, nullptr},
      {"/api/logout", HTTP_POST, onLogout, nullptr},
      {"/api/wifi", HTTP_POST, onWifi, nullptr},
      {"/api/backup", HTTP_GET, onBackup, nullptr},
      {"/api/restore", HTTP_POST, onRestore, nullptr},
      {"/api/ota", HTTP_POST, onOta, nullptr},
      {"/api/sensors", HTTP_GET, onSensors, nullptr},
      {"/api/sd", HTTP_GET, onSd, nullptr},
      {"/api/sdfile", HTTP_GET, onSdFile, nullptr},
      {"/api/cli", HTTP_POST, onCli, nullptr},
      {"/api/motor", HTTP_POST, onMotor, nullptr},
  };
  for (const httpd_uri_t& r : routes) httpd_register_uri_handler(server, &r);
  httpd_uri_t ws = {};
  ws.uri = "/ws";
  ws.method = HTTP_GET;
  ws.handler = onWs;
  ws.is_websocket = true;
  httpd_register_uri_handler(server, &ws);
  httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, notFound);
  return true;
}

// web task (every 50 ms): status push rate and the access point while armed
void poll() {
  if (!server) return;
  static uint32_t lastPushMs = 0;
  static bool radioOff = false;
  uint32_t now = millis();
  bool isArmed = armed();
  const fc::Config& c = bus::config;
  bool wantOff = isArmed && !c.wifi_armed && !c.wifi_enable;  // MAVLink over WiFi keeps the radio on
  if (wantOff != radioOff) {
    radioOff = wantOff;
    hal::wifiSetEnabled(!radioOff);
  }
  if (radioOff) return;
  uint32_t period = isArmed || wsSlow ? 500 : 100;  // 10 Hz on the ground (2 Hz for slow tabs), 2 Hz in flight
  if (now - lastPushMs >= period) {
    lastPushMs = now;
    httpd_queue_work(server, pushWork, nullptr);
  }
}

}  // namespace web
#endif
