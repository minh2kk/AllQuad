# AllQuad - flash the ESP32-S3 N16R8 over its COM port, then run the link test. Writes aqflash_log.txt.
import os, sys, subprocess, time, re
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
LOG = os.path.join(HERE, "aqflash_log.txt")
out = open(LOG, "w", encoding="utf-8", errors="replace")
def log(*a):
    s = " ".join(str(x) for x in a)
    try: print(s)
    except Exception: print(s.encode("ascii", "replace").decode())
    out.write(s + "\n"); out.flush()
def run(args):
    log("$ " + " ".join(args))
    # esptool draws its progress bar with Unicode blocks: a captured pipe on Windows is cp1252
    e = dict(os.environ, PYTHONUTF8="1", PYTHONIOENCODING="utf-8", NO_COLOR="1", TERM="dumb")
    p = subprocess.run([sys.executable, "-m", "esptool"] + args, capture_output=True, text=True,
                       encoding="utf-8", errors="replace", env=e)
    log(p.stdout[-4000:]); log(p.stderr[-2000:]); log("exit", p.returncode)
    return p.returncode, p.stdout + p.stderr
try:
    import esptool  # noqa
except ImportError:
    log("installing esptool ...")
    p = subprocess.run([sys.executable, "-m", "pip", "install", "esptool"], capture_output=True, text=True)
    log(p.stdout[-1500:], p.stderr[-1500:])
import serial.tools.list_ports
ports = [p for p in serial.tools.list_ports.comports() if p.vid in (0x1a86, 0x10c4, 0x303a, 0x0403)]
for p in serial.tools.list_ports.comports(): log("PORT", p.device, p.description, hex(p.vid or 0), hex(p.pid or 0))
if not ports: log("no ESP32 port"); sys.exit(1)
port = ports[0].device
img = os.path.join(ROOT, "firmware", "AllQuad-esp32s3-n16r8-full.bin")
code, txt = run(["--chip", "esp32s3", "--port", port, "--baud", "460800", "flash_id"])
size = re.search(r"Detected flash size:\s*(\S+)", txt)
log("flash size:", size.group(1) if size else "?", "| PSRAM:", "yes" if re.search(r"PSRAM", txt) else "?")
if code != 0: log("cannot talk to the ROM bootloader: hold BOOT, press RST, release BOOT, run again"); sys.exit(1)
if not size or size.group(1) != "16MB":
    log("NOT a 16 MB module: not flashing the N16R8 image"); sys.exit(1)
code, txt = run(["--chip", "esp32s3", "--port", port, "--baud", "921600", "--before", "default_reset", "--after", "hard_reset",
                 "write_flash", "-z", "0x0", img])
if code != 0:  # a slower line as a second try
    code, txt = run(["--chip", "esp32s3", "--port", port, "--baud", "460800", "--before", "default_reset", "--after", "hard_reset",
                     "write_flash", "-z", "0x0", img])
if code != 0: sys.exit(1)
log("flashed, waiting for boot ...")
time.sleep(4)
env = dict(os.environ, AQ_SETUP_CMDS="set yaw_reversed 1;set vbat_scale 4.3;set web_enable 1;save")  # this frame
p = subprocess.run([sys.executable, os.path.join(HERE, "aqtest.py")], capture_output=True, text=True, env=env)
log("===== TEST 1 (after flashing) =====")
log(p.stdout[-12000:]); log(p.stderr[-2000:])
# web_enable needs a restart: reset the chip through the bridge, then test again (web task should run)
run(["--chip", "esp32s3", "--port", port, "--after", "hard_reset", "chip_id"])
time.sleep(5)
p = subprocess.run([sys.executable, os.path.join(HERE, "aqtest.py")], capture_output=True, text=True)
log("===== TEST 2 (after restart, WiFi on) =====")
log(p.stdout[-12000:]); log(p.stderr[-2000:])
log("done")
