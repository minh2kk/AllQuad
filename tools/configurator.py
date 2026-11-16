#!/usr/bin/env python3
"""AllQuad Configurator on a laptop.

    python3 tools/configurator.py            opens http://localhost:8180 in the browser

The configurator is the same page the ESP32 serves over WiFi. On a laptop it talks to the flight
controller through the USB cable (COM port) with the browser's Web Serial API: Chrome or Edge on
Windows, macOS, Linux. Works for every board (ESP32, ESP32-S3, STM32) and needs no WiFi.

Opening build/web/index.html directly (double click) works as well; this launcher adds
  - "Mô phỏng (SITL)": the real firmware running on the PC (build/sim serve), handy for trying the
    configurator or developing without a board (needs tools/build_host.sh once).
  - always the current page from src/web/ui (bundled on every load).

Options: --port N (8180), --no-browser.  Python 3 standard library only.
"""
import base64
import hashlib
import json
import os
import struct
import subprocess
import sys
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIM = os.path.join(ROOT, "build", "sim")
sys.path.insert(0, os.path.join(ROOT, "tools"))
import embed_web  # noqa: E402  (bundle + minify of src/web/ui)


def page():
    return embed_web.with_demo(embed_web.minify(embed_web.bundle())).encode("utf-8")


def ws_send(sock, lock, payload, opcode=0x2):
    n = len(payload)
    head = bytes([0x80 | opcode]) + (bytes([n]) if n < 126 else (b"\x7e" + struct.pack(">H", n) if n < 65536 else b"\x7f" + struct.pack(">Q", n)))
    with lock:
        sock.sendall(head + payload)


def ws_recv(rfile):
    """One client frame -> (opcode, payload); None when the connection is gone."""
    h = rfile.read(2)
    if len(h) < 2:
        return None
    op, n = h[0] & 0x0F, h[1] & 0x7F
    if n == 126:
        n = struct.unpack(">H", rfile.read(2))[0]
    elif n == 127:
        n = struct.unpack(">Q", rfile.read(8))[0]
    mask = rfile.read(4) if h[1] & 0x80 else b"\0\0\0\0"
    data = bytearray(rfile.read(n))
    for i in range(len(data)):
        data[i] ^= mask[i & 3]
    return op, bytes(data)


BRIDGE = {"ssid": "AllQuad-Link-SIM", "defpw": True}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_POST(self):
        if self.path.split("?")[0] != "/bridge/wifi":
            self.send_error(404)
            return
        n = int(self.headers.get("Content-Length") or 0)
        lines = self.rfile.read(n).decode(errors="replace").split("\n")
        ok = len(lines) == 2 and 1 <= len(lines[0]) <= 32 and 8 <= len(lines[1]) <= 63
        if ok:
            BRIDGE["ssid"], BRIDGE["defpw"] = lines[0], lines[1] == "allquad123"
        body = b"saved: the bridge restarts with the new WiFi" if ok else b"name 1-32, password 8-63 characters"
        self.send_response(200 if ok else 400)
        self.send_header("Content-Type", "text/plain")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?")[0]
        # /link: what the ESP32-C3 WiFi bridge offers (src/bridge), here wired to the simulator: open
        # http://localhost:8080/?bridge to try the bridge page without the hardware
        if path in ("/sitl", "/link") and self.headers.get("Upgrade", "").lower() == "websocket":
            return self.sitl([SIM, "serve"] if os.access(SIM, os.X_OK) else None)
        # tests only: AQ_TEST_BRIDGES="name=command;..." exposes /bridge/<name> (e.g. a fake ESP32 ROM)
        if path.startswith("/bridge/") and self.headers.get("Upgrade", "").lower() == "websocket":
            bridges = dict(b.split("=", 1) for b in os.environ.get("AQ_TEST_BRIDGES", "").split(";") if "=" in b)
            cmd = bridges.get(path[len("/bridge/"):])
            return self.sitl(cmd.split() if cmd else None)
        if path == "/sitl/available":
            body = json.dumps({"ok": os.access(SIM, os.X_OK)}).encode()
            ctype = "application/json"
        elif path == "/bridge/info":  # stand-in for the bridge's own page (the simulator has no WiFi)
            body = json.dumps({"ok": True, "fw": "sim", "ssid": BRIDGE["ssid"], "baud": 460800, "defpw": BRIDGE["defpw"],
                               "rx": 0, "tx": 0}).encode()
            ctype = "application/json"
        elif path in ("/", "/index.html"):
            body, ctype = page(), "text/html; charset=utf-8"
        else:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def sitl(self, cmd):
        """WebSocket <-> stdin/stdout of a process: the firmware simulator (binary AQ-link frames)."""
        key = self.headers["Sec-WebSocket-Key"]
        accept = base64.b64encode(hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest())
        self.send_response(101)
        self.send_header("Upgrade", "websocket")
        self.send_header("Connection", "Upgrade")
        self.send_header("Sec-WebSocket-Accept", accept.decode())
        self.end_headers()
        self.wfile.flush()
        if not cmd:
            ws_send(self.connection, threading.Lock(), b"\x03\xe8", 0x8)
            return
        proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
        lock = threading.Lock()
        sock = self.connection

        def pump():
            try:
                while True:
                    chunk = proc.stdout.read1(65536) if hasattr(proc.stdout, "read1") else os.read(proc.stdout.fileno(), 65536)
                    if not chunk:
                        break
                    ws_send(sock, lock, chunk)
            except OSError:
                pass

        threading.Thread(target=pump, daemon=True).start()
        try:
            while True:
                f = ws_recv(self.rfile)
                if f is None or f[0] == 0x8:
                    break
                if f[0] == 0x9:
                    ws_send(sock, lock, f[1], 0xA)
                elif f[0] in (0x1, 0x2):
                    proc.stdin.write(f[1])
                    proc.stdin.flush()
        except (OSError, ValueError):
            pass
        finally:
            try:
                proc.stdin.close()
            except OSError:
                pass
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                proc.kill()
        self.close_connection = True


def main():
    port, browser = 8180, True
    args = sys.argv[1:]
    if "--no-browser" in args:
        browser = False
    if "--port" in args:
        port = int(args[args.index("--port") + 1])
    srv = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    srv.daemon_threads = True
    url = "http://localhost:%d/?usb" % port
    print("AllQuad Configurator: %s  (Chrome / Edge; Ctrl+C to stop)" % url)
    if not os.access(SIM, os.X_OK):
        print("  SITL off: run tools/build_host.sh once to try the configurator without a board")
    if browser:
        threading.Timer(0.5, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
