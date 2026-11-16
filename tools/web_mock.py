#!/usr/bin/env python3
"""Preview the web configurator on a PC without a drone.

  tools/build_host.sh && python3 tools/embed_web.py
  python3 tools/web_mock.py [port] [blackbox.bin]     -> http://localhost:8080

Serves build/web/index.html and a fake flight controller API: parameters come from
`build/tests web-dump params`, status is synthetic motion, /api/blackbox serves the
given log (default build/web/normal.bin, made by node test/web/test_analysis.js).
"""
import base64
import hashlib
import json
import math
import struct
import os
import subprocess
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
LOG = sys.argv[2] if len(sys.argv) > 2 else os.path.join(ROOT, "build", "web", "normal.bin")

params = json.loads(subprocess.check_output([os.path.join(ROOT, "build", "tests"), "web-dump", "params"]))
status = json.loads(subprocess.check_output([os.path.join(ROOT, "build", "tests"), "web-dump", "status"]))
by_name = {p["n"]: p for p in params["params"]}
for _n, _v in (("sd_mode", 2), ("bb_backend", 3), ("bb_rate_hz", 500)):  # mock board: SD card 1-bit, logging to it
    if _n in by_name:
        by_name[_n]["v"] = _v
state = {"armed": False}
AUTH = {"pw": "allquad123", "ssid": "AllQuad-ABCD", "tokens": set(), "fails": 0, "lock": 0.0}
WS_CLIENTS = [0]  # open WebSocket connections (tests check that the UI uses one)
T0 = time.time()


def live_status():
    t = time.time() - T0
    s = dict(status)
    s["t"] = int(t * 1000)
    s["armed"] = state["armed"]
    s["mode"] = "ANGLE"
    s["blockers"] = [] if state["armed"] else ["THROTTLE"]
    s["att"] = [round(12 * math.sin(t * 0.7), 1), round(6 * math.sin(t * 0.45), 1), round((t * 10) % 360 - 180, 1)]
    sp = [round(90 * math.sin(t * 2.1), 1), round(40 * math.sin(t * 1.3), 1), 0]
    s["sp"] = sp
    s["rate"] = [round(v * 0.92 + 4 * math.sin(t * 17), 1) for v in sp]
    s["thr"] = 0.42
    s["motors"] = [round(0.42 + 0.05 * math.sin(t * 3 + k), 3) for k in range(4)]
    s["notch"] = [182, 0, 0]
    s["bat"] = {"v": 16.4 - t * 0.001, "a": 11.2, "mah": int(t * 3), "cells": 4, "pct": 86, "warn": False}
    s["rc"] = [1500 + int(300 * math.sin(t * 2.1)), 1500, 1350, 1500, 1000, 1500, 2000, 1000] + [1500] * 8
    boxes = 0
    for i in range(1, 13):
        box, ch = int(by_name["mr%d_mode" % i]["v"]), int(by_name["mr%d_ch" % i]["v"])
        if box and by_name["mr%d_lo" % i]["v"] <= s["rc"][ch] < by_name["mr%d_hi" % i]["v"]:
            boxes |= 1 << box
    s["boxes"] = boxes
    s["lq"] = 100
    s["rxok"] = True
    s["gps"] = {"fix": True, "sats": 14, "lat": 21.0285, "lon": 105.8542, "hacc": 1.2}
    # a mission orbit around waypoint 3 (30 N, 30 E), 3 m/s wind from the south-west
    ang = t * 0.37
    n, e = 30 + 10 * math.cos(ang), 30 + 10 * math.sin(ang)
    s["est"] = {"alt": 10.1 + 0.2 * math.sin(t), "vz": 0.1, "agl": -1, "n": n, "e": e, "pos": True, "home": True,
                "dist": math.hypot(n, e), "vn": -3.7 * math.sin(ang), "ve": 3.7 * math.cos(ang),
                "yaw": math.degrees(math.atan2(30 - e, 30 - n)), "hsrc": 1, "hok": True, "wind": [2.1, 2.1],
                "hover": 0.497, "tgt": [n, e, 10.0], "aglhold": False, "glitch": False, "glitches": 1, "eph": 1.2,
                "mi": 4, "mc": 9}
    s["warn"] = 0
    s["ssid"] = AUTH["ssid"]
    s["defpw"] = AUTH["pw"] == "allquad123"
    s["fw"] = {"version": "2.0", "slot": "ota_0", "ota": True, "pending": False}
    s["perf"] = [38.0, 41.5, 12.8, 9.6, 6.1, 108.0, 171.0, 1000.0, 10.8]  # ESP32-like numbers (illustrative)
    s["late"] = 0
    # per-UART counters: RX on UART1 (CRSF 420000 baud, 150 Hz frames) with a few framing errors, GPS on UART2
    k = int(t * 150)
    s["ports"] = [[0, k * 26, int(t * 40) * 12, 0, 3 if t > 5 else 0, 0, 0], [1, int(t * 1100), int(t * 4), 0, 0, 0, 0]]
    s["rxc"] = [k * 26, k, 2 if t > 5 else 0]
    s["gpsl"] = [1, 115200, int(t * 10), 1]  # GPS on UART2, UBX 115200, 10 frames/s
    # last reset: a hard fault in the telemetry task (STM32 crash record), watchdog running
    s["rr"] = 4 if CRASH else 3
    s["wdt"] = True
    if CRASH:
        s["crash"] = dict(CRASH)
    s["tasks"] = [{"n": "fast", "hz": 1000, "us": 210, "max": 340, "over": 0, "stk": 2100},
                  {"n": "io", "hz": 500, "us": 60, "max": 140, "over": 0, "stk": 2600},
                  {"n": "control", "hz": 250, "us": 180, "max": 420, "over": 0, "stk": 3900},
                  {"n": "sensors", "hz": 100, "us": 320, "max": 900, "over": 0, "stk": 2300},
                  {"n": "telemetry", "hz": 100, "us": 90, "max": 400, "over": 0, "stk": 4700},
                  {"n": "logger", "hz": 500, "us": 40, "max": 2100, "over": 0, "stk": 2900},
                  {"n": "web", "hz": 200, "us": 15, "max": 8200, "over": 3, "stk": 1800}]
    s["sensors"]["gps"] = True
    s["log"] = {"size": os.path.getsize(LOG) if os.path.exists(LOG) else 0, "cap": 2424832, "rate": 500}
    return s


def item(cmd, n=0.0, e=0.0, alt=0.0, p1=0.0, p3=0.0):
    la = round((21.0285 + n / 6371000 * 57.29578) * 1e7) if (n or e) else 0
    lo = round((105.8542 + e / (6371000 * math.cos(math.radians(21.0285))) * 57.29578) * 1e7) if (n or e) else 0
    return {"cmd": cmd, "pos": bool(n or e), "n": n, "e": e, "alt": alt, "p1": p1, "p2": 0, "p3": p3, "p4": 0, "la": la, "lo": lo}


MOTOR = [0.0, 0.0, 0.0, 0.0]
CRASH = {"kind": "hard fault", "where": "telemetry", "t": 148210, "ago": 0, "n": 1, "pc": "0x0800F4E6", "lr": "0x0800E5AF", "cfsr": "0x00010000"}
MISSION = {"home": True, "lat": 21.0285, "lon": 105.8542, "fence": 150,
           "items": [item(22, alt=10), item(195, 0, 40), item(16, 30, 0, 10), item(197), item(18, 30, 30, 10, 1, 10),
                     item(177, p1=2), item(115, p1=90), item(16, 50, 50, 12), item(20)],
           "rally": [item(5100, 60, 40, 8)],
           "poly": [[-80, -90, 210277800, 1058533300], [-80, 110, 210277800, 1058552600], [90, 120, 210293100, 1058553500],
                    [100, -70, 210294000, 1058535200]], "polyOn": True}


# fake SD card contents
SD_FILES = {}
if os.path.exists(LOG):
    with open(LOG, "rb") as _f:
        _b = _f.read()
    SD_FILES = {"AQ00001.BBL": _b, "AQ00002.BBL": _b}


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def authed(self):
        if os.environ.get("MOCK_NO_AUTH"):
            return True
        for part in self.headers.get("Cookie", "").split(";"):
            k, _, v = part.strip().partition("=")
            if k == "aq" and v in AUTH["tokens"]:
                return True
        return False

    def form(self):
        n = int(self.headers.get("Content-Length") or 0)
        return {k: v[0] for k, v in parse_qs(self.rfile.read(n).decode(), keep_blank_values=True).items()}

    def send(self, code, body, ctype="application/json", cookie=None):
        if isinstance(body, (dict, list)):
            body = json.dumps(body).encode()
        self.send_response(code)
        if cookie:
            self.send_header("Set-Cookie", cookie)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def websocket(self):
        """Minimal RFC 6455 server side: push the status as text frames at 10 Hz (like the ESP32)."""
        key = self.headers["Sec-WebSocket-Key"]
        accept = base64.b64encode(hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest())
        self.send_response(101)
        self.send_header("Upgrade", "websocket")
        self.send_header("Connection", "Upgrade")
        self.send_header("Sec-WebSocket-Accept", accept.decode())
        self.end_headers()
        WS_CLIENTS[0] += 1
        SLOW = ("tasks", "perf", "late", "ports", "rxc", "gpsl", "rr", "wdt", "crash", "sensors", "log", "ssid", "fw", "defpw")
        frame = 0
        try:
            while True:
                st = live_status()
                # like the firmware: the slow fields only in every 10th frame (and the first)
                st["full"] = frame % 10 == 0
                if not st["full"]:
                    st = {k: v for k, v in st.items() if k not in SLOW}
                frame += 1
                data = json.dumps(st).encode()
                n = len(data)
                head = b"\x81" + (bytes([n]) if n < 126 else (b"\x7e" + struct.pack(">H", n) if n < 65536 else b"\x7f" + struct.pack(">Q", n)))
                self.wfile.write(head + data)
                self.wfile.flush()
                time.sleep(0.1)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            WS_CLIENTS[0] -= 1
        self.close_connection = True

    def do_POST(self):
        self.do_GET()

    def do_GET(self):
        u = urlparse(self.path)
        q = {k: v[0] for k, v in parse_qs(u.query).items()}
        if u.path == "/":
            with open(os.path.join(ROOT, "build", "web", "index.html"), "rb") as f:
                return self.send(200, f.read(), "text/html; charset=utf-8")
        if u.path == "/api/login":  # same rules as the firmware: 5 wrong -> 30 s lock
            if time.time() < AUTH["lock"]:
                return self.send(429, {"ok": False, "msg": "too many attempts"})
            if self.form().get("pw") != AUTH["pw"]:
                AUTH["fails"] += 1
                if AUTH["fails"] >= 5:
                    AUTH["fails"], AUTH["lock"] = 0, time.time() + 30
                return self.send(401, {"ok": False, "msg": "wrong password"})
            tok = os.urandom(16).hex()
            AUTH["tokens"].add(tok)
            AUTH["fails"] = 0
            return self.send(200, {"ok": True, "msg": "ok"}, cookie="aq=%s; Path=/; HttpOnly; SameSite=Strict" % tok)
        if u.path.startswith("/api/") or u.path == "/ws":
            if not self.authed():
                return self.send(401, {"ok": False, "msg": "login required"})
        if u.path == "/api/backup":
            lines = ["# AllQuad mock backup (%s)" % ("all" if q.get("all") == "1" else "diff")]
            lines += ["set %s %s" % (n, p["v"]) for n, p in by_name.items() if q.get("all") == "1" or n.startswith("roll_")]
            lines += ["mission clear", "rally clear", "save"]
            return self.send(200, ("\n".join(lines) + "\n").encode(), "text/plain")
        if u.path == "/api/restore":
            n = int(self.headers.get("Content-Length") or 0)
            text = self.rfile.read(n).decode()
            changes, cmds = {}, 0
            for line in text.splitlines():
                line = line.strip()
                if not line or line.startswith("#") or line == "save":
                    continue
                cmds += 1
                parts = line.split()
                if parts[0] == "set" and len(parts) == 3 and parts[1] in by_name:
                    changes[parts[1]] = float(parts[2])
                elif parts[0] not in ("mission", "rally"):
                    return self.send(400, {"ok": False, "msg": "%d commands, 1 errors (nothing changed): %s" % (cmds, line)})
            for k, v in changes.items():
                by_name[k]["v"] = v
            return self.send(200, {"ok": True, "msg": "%d commands applied and saved" % cmds})
        if u.path == "/api/ota":
            n = int(self.headers.get("Content-Length") or 0)
            data = self.rfile.read(n)
            if n < 65536 or data[0] != 0xE9:
                return self.send(400, {"ok": False, "msg": "not an ESP32 firmware image"})
            return self.send(200, {"ok": True, "msg": "update written, rebooting"})
        if u.path == "/api/logout":
            AUTH["tokens"].clear()
            return self.send(200, {"ok": True, "msg": "logged out"}, cookie="aq=; Path=/; Max-Age=0")
        if u.path == "/api/wifi":
            f = self.form()
            if f.get("cur") != AUTH["pw"]:
                return self.send(401, {"ok": False, "msg": "wrong password"})
            if f.get("wifi") and not 8 <= len(f["wifi"]) <= 63:
                return self.send(400, {"ok": False, "msg": "WiFi password needs 8-63 characters"})
            AUTH["ssid"] = f.get("ssid") or AUTH["ssid"]
            AUTH["pw"] = f.get("web") or f.get("wifi") or AUTH["pw"]
            AUTH["tokens"].clear()
            return self.send(200, {"ok": True, "msg": "saved"})
        if u.path == "/ws" and self.headers.get("Upgrade", "").lower() == "websocket" and not os.environ.get("MOCK_NO_WS"):
            return self.websocket()
        if u.path == "/api/params":
            params["armed"] = state["armed"]
            return self.send(200, params)
        if u.path == "/api/status":
            return self.send(200, live_status())
        if u.path == "/api/set":
            if state["armed"]:
                return self.send(423, {"ok": False, "msg": "disarm first"})
            for k, v in q.items():
                p = by_name.get(k)
                try:
                    x = float(v)
                except ValueError:
                    return self.send(400, {"ok": False, "msg": k})
                if not p or not (p["min"] <= x <= p["max"]):
                    return self.send(400 if p else 404, {"ok": False, "msg": k})
                p["v"] = x if p["f"] else round(x)
            return self.send(200, {"ok": True, "msg": "applied (not saved)"})
        if u.path == "/api/mission":
            return self.send(200, MISSION)
        if u.path == "/api/sensors":
            t = time.time()
            return self.send(200, {"t": int(t * 1000), "gyro": [math.sin(t * 3) * 20, math.cos(t * 2) * 15, 2.0],
                                   "acc": [0.3 * math.sin(t), 0.2, -9.8], "mag": [0.21, 0.02, 0.41], "magok": True,
                                   "baro": {"ok": True, "alt": 1.2 + 0.1 * math.sin(t), "pa": 101100, "temp": 27.5},
                                   "range": -1, "flow": [0, 0], "flowq": -1,
                                   "q": [math.cos(0.1 * math.sin(t)), 0.1 * math.sin(t), 0.05, 0.0],
                                   "rpm": [-1, -1, -1, -1], "esct": [-1, -1, -1, -1]})
        if u.path == "/api/cli":
            n = int(self.headers.get("Content-Length") or 0)
            line = self.rfile.read(n).decode().strip()
            if line.startswith("get"):
                f = line[3:].strip()
                out = "".join("%s = %s\r\n" % (k, p["v"]) for k, p in by_name.items() if f in k)
            elif line == "crash clear":
                CRASH.clear()
                out = "crash record cleared\r\n"
            elif line == "help":
                out = "status | tasks | perf | get [filter] | set <name> <value> | save\r\n"
            elif line.startswith("logic"):
                rules = state.setdefault("logic", [[0] * 9 for _ in range(8)])
                a = line.split()[1:]
                if len(a) == 10:
                    rules[int(a[0]) - 1] = [int(x) for x in a[1:]]
                    out = "ok (save to keep)\r\n"
                else:
                    out = "".join("logic %d %s\r\n" % (i + 1, " ".join(map(str, r))) for i, r in enumerate(rules))
            else:
                out = "unknown command, type 'help'\r\n"
            return self.send(200, out.encode(), "text/plain")
        if u.path == "/api/motor":
            if state["armed"]:
                return self.send(423, {"ok": False, "msg": "disarm first"})
            MOTOR[:] = [float(q.get("m%d" % i, 0)) for i in range(1, 5)]
            return self.send(200, {"ok": True, "msg": "ok"})
        if u.path == "/mock/motor":
            return self.send(200, {"m": MOTOR})
        if u.path == "/mock/ws_clients":
            return self.send(200, {"n": WS_CLIENTS[0]})
        if u.path == "/api/save":
            return self.send(200, {"ok": True, "msg": "saved"})
        if u.path == "/api/sd":  # fake card: the blackbox file (if any) appears as two logs
            files = [{"n": n, "s": len(b)} for n, b in sorted(SD_FILES.items())]
            used = sum(f["s"] for f in files)
            return self.send(200, {"state": "ready", "mode": 2, "type": "SDHC/SDXC", "err": "", "total": 31_900_000_000,
                                   "used": used + 1_200_000, "file": files[-1]["n"] if files else "", "bytes": 0, "q": 0,
                                   "qs": 262144, "drop": 0, "wms": 38, "pins": [39, 38, 40, 255, 255, 255],
                                   "files": [] if state["armed"] else files, "listed": not state["armed"]})
        if u.path == "/api/sdfile":
            if state["armed"]:
                return self.send(423, {"ok": False, "msg": "disarm first"})
            b = SD_FILES.get(q.get("n", ""))
            if b is None:
                return self.send(409, {"ok": False, "msg": "no file"})
            return self.send(200, b, "application/octet-stream")
        if u.path == "/api/cmd":
            if q.get("c", "").startswith("sd_rm_"):
                SD_FILES.pop(q["c"][6:], None)
            if q.get("c") == "arm":  # mock only: lets tests try the armed lock
                state["armed"] = not state["armed"]
            return self.send(200, {"ok": True, "msg": q.get("c", "")})
        if u.path == "/api/blackbox":
            if not os.path.exists(LOG):
                return self.send(404, {"ok": False, "msg": "no log"})
            with open(LOG, "rb") as f:
                return self.send(200, f.read(), "application/octet-stream")
        self.send(404, {"ok": False, "msg": "not found"})


if __name__ == "__main__":
    print("AllQuad web mock on http://localhost:%d" % PORT)
    ThreadingHTTPServer(("127.0.0.1", PORT), H).serve_forever()
