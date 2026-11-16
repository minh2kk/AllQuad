#!/usr/bin/env python3
"""Runs tools/edgetx/SCRIPTS/TELEMETRY/aqtlm.lua and SCRIPTS/TOOLS/aqcfg.lua in Lua 5.2 (the EdgeTX
Lua version) with a small EdgeTX API stand-in and draws their 128x64 screens into PNG files: a syntax /
runtime check of the scripts and a preview of what the RadioMaster Pocket shows. aqcfg.lua talks MSP over
CRSF to the real firmware code (build/tests crsf-msp: run tools/build_host.sh first).
    pip install lupa pillow ; python3 tools/edgetx/preview.py [out_dir]
Fonts are approximations of the radio's bitmap fonts."""
import os, sys, math
import lupa.lua52 as lupa
from PIL import Image, ImageDraw, ImageFont

import subprocess
HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, "SCRIPTS", "TELEMETRY", "aqtlm.lua")
CFG = os.path.join(HERE, "SCRIPTS", "TOOLS", "aqcfg.lua")
TESTS = os.path.join(os.path.dirname(os.path.dirname(HERE)), "build", "tests")
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "preview")
W, H = 128, 64
FL = dict(SMLSIZE=0x0200, MIDSIZE=0x0400, DBLSIZE=0x0800, INVERS=0x01, BLINK=0x02, RIGHT=0x04, PREC1=0x10, PREC2=0x20,
          SOLID=0, DOTTED=0x55, FORCE=0x08, GREY_DEFAULT=0x40, ERASE=0x80)


def font(px):
    for p in ("/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"):
        if os.path.exists(p):
            return ImageFont.truetype(p, px)
    return ImageFont.load_default()


FONTS = {0: (font(9), 6, 8), FL["SMLSIZE"]: (font(7), 4, 6), FL["MIDSIZE"]: (font(13), 8, 12), FL["DBLSIZE"]: (font(17), 10, 16)}


class Screen:
    def __init__(self):
        self.img = Image.new("1", (W, H), 1)
        self.d = ImageDraw.Draw(self.img)
        self.last_right = 0
        self.texts = []

    def text(self, x, y, s, flags=0):
        size = flags & (FL["SMLSIZE"] | FL["MIDSIZE"] | FL["DBLSIZE"])
        f, adv, h = FONTS.get(size, FONTS[0])
        s = str(s)
        self.texts.append(s)
        w = adv * len(s)
        if flags & FL["RIGHT"]:
            x -= w
        inv = flags & FL["INVERS"]
        if inv:
            self.d.rectangle([x - 1, y - 1, x + w, y + h - 1], fill=0)
        # draw each character into its cell so widths match the radio's monospace fonts
        for i, c in enumerate(s):
            self.d.text((x + i * adv, y - 1), c, font=f, fill=1 if inv else 0)
        self.last_right = x + w

    def number(self, x, y, v, flags=0):
        v = int(v)
        if flags & FL["PREC1"]:
            s = "%.1f" % (v / 10)
        elif flags & FL["PREC2"]:
            s = "%.2f" % (v / 100)
        else:
            s = str(v)
        self.text(x, y, s, flags)


def run(state, events, name):
    scr = Screen()
    L = lupa.LuaRuntime()
    g = L.globals()
    for k, v in FL.items():
        g[k] = v
    g.LCD_W, g.LCD_H = W, H
    g.EVT_VIRTUAL_ENTER, g.EVT_VIRTUAL_NEXT, g.EVT_VIRTUAL_PREV = 100, 101, 102
    tick = {"t": 0}
    lcd = L.table()
    lcd.clear = lambda *a: scr.d.rectangle([0, 0, W, H], fill=1)
    lcd.drawText = lambda x, y, s, f=0: scr.text(int(x), int(y), s, int(f or 0))
    lcd.drawNumber = lambda x, y, v, f=0: scr.number(int(x), int(y), v, int(f or 0))
    lcd.getLastRightPos = lambda *a: scr.last_right
    lcd.drawLine = lambda x1, y1, x2, y2, p=0, f=0: scr.d.line([int(round(x1)), int(round(y1)), int(round(x2)), int(round(y2))], fill=0)
    lcd.drawRectangle = lambda x, y, w, h, f=0: scr.d.rectangle([int(x), int(y), int(x + w - 1), int(y + h - 1)], outline=0)
    lcd.drawFilledRectangle = lambda x, y, w, h, f=0: scr.d.rectangle([int(x), int(y), int(x + w - 1), int(y + h - 1)], fill=0) if w > 0 and h > 0 else None
    lcd.drawPoint = lambda x, y: scr.d.point([int(x), int(y)], fill=0)
    g.lcd = lcd
    haptic = []
    g.playHaptic = lambda *a: haptic.append(a)
    g.getTime = lambda: tick["t"]

    def field(n):
        if n in state["v"]:
            t = L.table()
            t.id = n
            return t
        return None
    g.getFieldInfo = field

    def value(n):
        v = state["v"].get(n, 0)
        if isinstance(v, dict):
            t = L.table()
            t.lat, t.lon = v["lat"], v["lon"]
            return t
        return v
    g.getValue = value
    with open(SCRIPT, encoding="utf-8") as f:
        mod = L.execute(f.read())
    mod.init()
    for st in state.get("steps", [state["v"]]):
        state["v"] = st
        tick["t"] += 100
        mod.background()
    for ev in events:
        tick["t"] += 10
        mod.run(ev)
    os.makedirs(OUT, exist_ok=True)
    big = scr.img.convert("L").resize((W * 4, H * 4), Image.NEAREST)
    # radio-like LCD tint
    big = Image.merge("RGB", [big.point(lambda p: 30 if p == 0 else 168), big.point(lambda p: 40 if p == 0 else 190),
                              big.point(lambda p: 35 if p == 0 else 160)])
    big.save(os.path.join(OUT, name + ".png"))
    return haptic


def save_png(scr, name):
    os.makedirs(OUT, exist_ok=True)
    big = scr.img.convert("L").resize((W * 4, H * 4), Image.NEAREST)
    big = Image.merge("RGB", [big.point(lambda p: 30 if p == 0 else 168), big.point(lambda p: 40 if p == 0 else 190),
                              big.point(lambda p: 35 if p == 0 else 160)])
    big.save(os.path.join(OUT, name + ".png"))


def crc8(b):
    c = 0
    for x in b:
        c ^= x
        for _ in range(8):
            c = ((c << 1) ^ 0xD5) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def run_cfg():
    """aqcfg.lua against the firmware's MSP-over-CRSF code; returns a list of failures"""
    fc = subprocess.Popen([TESTS, "crsf-msp"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
    scr = Screen()
    L = lupa.LuaRuntime(unpack_returned_tuples=True)  # crossfireTelemetryPop returns two values
    g = L.globals()
    for k, v in FL.items():
        g[k] = v
    g.LCD_W, g.LCD_H = W, H
    ev = dict(ENTER=100, NEXT=101, PREV=102, INC=103, DEC=104, EXIT=105, NEXT_PAGE=106, PREV_PAGE=107, ENTER_LONG=108)
    for k, v in ev.items():
        g["EVT_VIRTUAL_" + k] = v
    tick = {"t": 0}
    inbox = []
    lcd = L.table()
    lcd.clear = lambda *a: (scr.d.rectangle([0, 0, W, H], fill=1), scr.texts.clear())
    lcd.drawText = lambda x, y, s, f=0: scr.text(int(x), int(y), s, int(f or 0))
    lcd.drawNumber = lambda x, y, v, f=0: scr.number(int(x), int(y), v, int(f or 0))
    lcd.drawFilledRectangle = lambda x, y, w, h, f=0: scr.d.rectangle([int(x), int(y), int(x + w - 1), int(y + h - 1)], fill=0)
    lcd.drawLine = lambda *a: None
    g.lcd = lcd
    g.getTime = lambda: tick["t"]

    def push(cmd, data):
        body = [int(cmd)] + [int(data[i]) for i in range(1, len(data) + 1)]
        frame = [0xC8, len(body) + 1] + body + [crc8(body)]
        fc.stdin.write(bytes(frame).hex() + "\n")
        fc.stdin.flush()
        while True:
            line = fc.stdout.readline().strip()
            if line == "." or not line:
                break
            b = bytes.fromhex(line)
            assert b[0] == 0xC8 and crc8(b[2:-1]) == b[-1], "bad frame from the FC"
            inbox.append((b[2], list(b[3:-1])))
        return True
    g.crossfireTelemetryPush = push

    def pop():
        if not inbox:
            return None
        cmd, data = inbox.pop(0)
        t = L.table()
        for i, x in enumerate(data):
            t[i + 1] = x
        return cmd, t
    g.crossfireTelemetryPop = pop
    with open(CFG, encoding="utf-8") as f:
        mod = L.execute(f.read())
    mod.init()
    fails = []

    def frame(e=0, n=1):
        for _ in range(n):
            tick["t"] += 2
            mod.run(e)

    frame(0, 120)
    texts = " ".join(scr.texts)
    if "..." in texts.split("Reading")[0] or "Reading" in texts:
        fails.append("parameters not all read: " + texts)
    save_png(scr, "6-cfg-pid")
    roll_d = None
    try:
        roll_d = int(scr.texts[scr.texts.index("Roll D") + 1])
    except (ValueError, IndexError):
        fails.append("Roll D not shown: " + texts)
    # Roll D: down twice, edit, +3, send
    frame(ev["NEXT"], 1); frame(ev["NEXT"], 1); frame(ev["ENTER"], 1)
    for _ in range(3):
        frame(ev["INC"], 1)
    save_png(scr, "7-cfg-edit")
    frame(ev["ENTER"], 1)
    frame(0, 30)
    try:
        now = int(scr.texts[scr.texts.index("Roll D") + 1])
    except (ValueError, IndexError):
        now = None
    if roll_d is None or now != roll_d + 3 or "Set roll_d" not in " ".join(scr.texts):
        fails.append("Roll D set: %s -> %s, %s" % (roll_d, now, scr.texts))
    # VTX page, then SAVE TO FC
    for _ in range(3):
        frame(ev["NEXT_PAGE"], 1)
    frame(0, 5)
    save_png(scr, "8-cfg-vtx")
    if "VTX" not in " ".join(scr.texts) or "R" not in scr.texts:
        fails.append("VTX page: %s" % scr.texts)
    frame(ev["PREV"], 1); frame(ev["ENTER"], 1); frame(0, 30)
    if "Saved" not in " ".join(scr.texts):
        fails.append("save: %s" % scr.texts)
    fc.stdin.close()
    fc.wait(timeout=5)
    return fails


HOME = {"lat": 21.028500, "lon": 105.854200}
AWAY = {"lat": 21.029400, "lon": 105.855100}
base = {"RxBt": 16.4, "Curr": 0.6, "Capa": 0, "Bat%": 100, "FM": "ANGLE*", "VSpd": 0.0, "Alt": 0.0, "GPS": HOME, "GSpd": 0,
        "Hdg": 0, "Sats": 14, "Ptch": 0.0, "Roll": 0.0, "Yaw": 0.0, "RQly": 100, "1RSS": -48, "2RSS": -51, "RSNR": 9, "TPWR": 100}
fly = dict(base, **{"RxBt": 15.1, "Curr": 18.4, "Capa": 642, "Bat%": 58, "FM": "ANGLE", "VSpd": 1.3, "Alt": 23.6, "GPS": AWAY,
                    "GSpd": 42.0, "Hdg": 30, "Ptch": -0.21, "Roll": 0.35, "Yaw": 0.52, "RQly": 96, "1RSS": -71, "2RSS": -75})
low = dict(fly, **{"RxBt": 13.6, "Bat%": 12, "Capa": 1210, "RQly": 41, "FM": "RTH"})
landed = dict(fly, **{"FM": "ANGLE*", "Curr": 0.5, "VSpd": 0, "GSpd": 0})
notel = {}

armedHome = dict(fly, GPS=HOME, Alt=0.5)
ok = True
try:
    run({"v": fly, "steps": [base, armedHome, fly]}, [0], "1-flight")
    run({"v": fly, "steps": [base, armedHome, fly]}, [100], "2-link")
    h = run({"v": low, "steps": [base, armedHome, fly, low]}, [0], "3-low-battery")
    run({"v": landed, "steps": [base, armedHome, fly, landed]}, [0], "4-summary")
    run({"v": notel, "steps": [notel]}, [0], "5-no-telemetry")
    print("haptic on low battery:", h)
    ok = len(h) >= 1
    if os.access(TESTS, os.X_OK):
        f = run_cfg()
        print("aqcfg.lua over MSP/CRSF:", "OK" if not f else f)
        ok = ok and not f
    else:
        print("build/tests missing: aqcfg.lua not checked (tools/build_host.sh)")
except lupa.LuaError as e:
    print("Lua error:", e)
    ok = False
print("previews in", OUT)
sys.exit(0 if ok else 1)
