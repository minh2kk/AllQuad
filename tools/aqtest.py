# AllQuad - serial link diagnostic (Windows). Run: aqtest.bat (double click). Writes aqtest_log.txt.
import os, sys, time, struct, traceback
LOG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "aqtest_log.txt")
out = open(LOG, "w", encoding="utf-8")
def log(*a):
    s = " ".join(str(x) for x in a)
    print(s); out.write(s + "\n"); out.flush()
try:
    import serial, serial.tools.list_ports
except Exception as e:
    log("NO PYSERIAL:", e); sys.exit(1)

def crc16(b, crc=0xFFFF):
    for x in b:
        crc ^= x << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc
SEQ = [0]
def frame(path, body=b""):
    SEQ[0] = (SEQ[0] + 1) & 255
    p = path.encode() + b"\0" + body
    f = bytes([SEQ[0], len(p) & 255, len(p) >> 8]) + p
    return b"\xA7\x51" + f + struct.pack("<H", crc16(f))

def show(tag, data):
    txt = data.decode("latin1")
    printable = "".join(c if 32 <= ord(c) < 127 or c in "\r\n" else "." for c in txt)
    log("  %s: %d bytes" % (tag, len(data)))
    if data:
        log("  text: " + printable[:1500].replace("\r", "").replace("\n", "\n        "))
        log("  hex : " + data[:96].hex(" "))

def req(ser, path, body=b"", timeout=3.0):
    rx = b""
    ser.write(frame(path, body)); want = SEQ[0]; t0 = time.time()
    while time.time() - t0 < timeout:
        rx += ser.read(4096)
        i = rx.find(b"\xA7\x52")
        while i >= 0:
            b = rx[i:]
            if len(b) >= 17:
                ln = b[13] | (b[14] << 8)
                if len(b) >= 17 + ln and b[2] == want:
                    st, tot = struct.unpack("<HI", b[3:9])
                    ok = crc16(b[2:15 + ln]) == (b[15 + ln] | (b[16 + ln] << 8))
                    return st, tot, b[15:15 + ln], ok, rx
            i = rx.find(b"\xA7\x52", i + 1)
    return None, None, None, None, rx

log("AllQuad serial test", time.strftime("%Y-%m-%d %H:%M:%S"), "python", sys.version.split()[0], "pyserial", serial.__version__)
ports = list(serial.tools.list_ports.comports())
for p in ports:
    log("PORT %s | %s | vid=%s pid=%s | %s" % (p.device, p.description, hex(p.vid) if p.vid else None, hex(p.pid) if p.pid else None, p.hwid))
if not ports:
    log("no COM ports found")
cands = [p for p in ports if p.vid] or ports
for p in cands:
    for baud in (921600, 115200):
        log("\n=== %s at %d ===" % (p.device, baud))
        try:
            ser = serial.Serial()
            ser.port = p.device; ser.baudrate = baud; ser.timeout = 0.05
            ser.dtr = False; ser.rts = False
            ser.open()
        except Exception as e:
            log("  open failed:", repr(e)); break
        try:
            t0 = time.time(); raw = b""
            while time.time() - t0 < 3.0: raw += ser.read(4096)
            show("idle output 3 s after open", raw)
            st, tot, data, ok, rx = req(ser, "/api/hello")
            if st is None:
                show("no hello answer, received", rx)
            else:
                log("  hello:", st, "crc ok" if ok else "CRC BAD", data.decode("latin1"))
                extra = [c for c in os.environ.get("AQ_SETUP_CMDS", "").split(";") if c.strip()]
                for cmd in extra + ["version", "perf", "net", "scan", "status", "tasks", "get web_enable", "get wifi_enable", "get yaw_reversed", "get vbat_scale", "get imu_sync", "get loop_hz"]:
                    st, tot, data, ok, rx = req(ser, "/api/cli", cmd.encode(), 6)
                    if st is None: show("CLI %r: no answer, received" % cmd, rx)
                    else: log("  CLI %r -> %s %s\n%s" % (cmd, st, "ok" if ok else "CRC BAD", data.decode("latin1")[:1500]))
                    st2, _, d2, ok2, rx2 = req(ser, "/api/hello")
                    log("  hello after CLI:", d2.decode("latin1") if st2 == 200 else ("NO ANSWER " + rx2[:80].hex(" ")))
                # soak: hello every 0.3 s for 40 s, catch resets (uptime going back) and silent periods
                log("  soak test 40 s ...")
                t0 = time.time(); lastUp = None; fails = 0; resets = 0; n = 0
                while time.time() - t0 < 40:
                    st, tot, data, ok, rx = req(ser, "/api/hello", timeout=1.0)
                    n += 1
                    if st != 200:
                        fails += 1
                        if fails <= 3: show("  hello failed at %.1f s, received" % (time.time() - t0), rx)
                        continue
                    import json
                    try: j = json.loads(data.decode("latin1"))
                    except Exception: continue
                    up = j.get("up")
                    if lastUp is not None and up is not None and up < lastUp:
                        resets += 1; log("  RESET detected at %.1f s: uptime %s -> %s ms, reset reason %s" % (time.time() - t0, lastUp, up, j.get("rr")))
                    lastUp = up
                    time.sleep(0.3)
                log("  soak: %d requests, %d failed, %d resets, last hello %s" % (n, fails, resets, data.decode("latin1") if data else None))
                for cmd in ("perf", "tasks"):
                    st, tot, data, ok, rx = req(ser, "/api/cli", cmd.encode(), 6)
                    log("  after soak CLI %r ->\n%s" % (cmd, data.decode("latin1") if data else "no answer"))
            ser.reset_input_buffer()
            ser.write(b"\r\nversion\r\n"); t0 = time.time(); raw = b""
            while time.time() - t0 < 1.5: raw += ser.read(4096)
            show("plain text 'version'", raw)
            ser.close()
            if st is not None: break
        except Exception:
            log(traceback.format_exc())
            try: ser.close()
            except Exception: pass
log("\ndone")
