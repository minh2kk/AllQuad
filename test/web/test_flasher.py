#!/usr/bin/env python3
"""USB firmware flasher of the configurator, end to end without boards:
  * ESP32 and ESP32-S3: the page's ROM-loader code (Web Serial) against test/web/fake_esp_rom.py,
    a ROM emulator that real esptool also flashes successfully (see the check at the start)
  * STM32: the page's DfuSe code (WebUSB) against a DfuSe device emulated in the page
Needs the three firmware builds (pio run) for real images, Playwright + Chromium, esptool (pip).
    python3 test/web/test_flasher.py [screenshot_dir]
"""
import asyncio, hashlib, os, subprocess, sys, tempfile, time
from playwright.async_api import async_playwright

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SHOTS = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "build", "web", "shots")
PORT = 8183
TMP = tempfile.mkdtemp(prefix="aqflash")
FW = {k: os.path.join(ROOT, ".pio", "build", e, "firmware.bin") for k, e in
      (("esp32", "esp32dev"), ("esp32s3", "esp32s3"), ("esp32c6", "esp32c6"), ("stm32", "blackpill_f411ce"))}
fails = []


def check(ok, msg):
    print("[%s] %s" % ("PASS" if ok else "FAIL", msg))
    if not ok:
        fails.append(msg)


FAKE_SERIAL = """
(() => {
  const mk = (bridge, vid) => {
    const p = new EventTarget();
    p.signals = [];
    p.getInfo = () => ({ usbVendorId: vid, usbProductId: 1 });
    p.open = async () => {
      const ws = new WebSocket(location.origin.replace(/^http/, "ws") + "/bridge/" + window.__bridge);
      ws.binaryType = "arraybuffer";
      p.ws = ws;
      await new Promise((r, j) => { ws.onopen = r; ws.onerror = j; });
      let ctl;
      p.readable = new ReadableStream({ start(c) { ctl = c; } });
      ws.onmessage = (ev) => ctl.enqueue(new Uint8Array(ev.data));
      ws.onclose = () => { try { ctl.close(); } catch (e) {} };
      p.writable = new WritableStream({ write(chunk) { ws.send(chunk); } });
    };
    p.setSignals = async (s) => { p.signals.push([s.dataTerminalReady, s.requestToSend]); };
    p.close = async () => { try { p.ws.close(); } catch (e) {} };
    return p;
  };
  const serial = new EventTarget();
  serial.requestPort = async () => (window.__port = mk(window.__bridge, window.__vid));
  serial.getPorts = async () => (window.__port ? [window.__port] : []);
  Object.defineProperty(navigator, "serial", { value: serial });

  // ---- a DfuSe device (STM32F411 system bootloader) ----
  const FL = 0x08000000, SIZE = 512 * 1024;
  const flash = new Uint8Array(SIZE);
  for (let i = 0; i < SIZE; i++) flash[i] = (i * 7 + 3) & 255;          // an old firmware everywhere
  const sectors = [[0, 16384], [16384, 16384], [32768, 16384], [49152, 16384], [65536, 65536], [131072, 131072], [262144, 131072], [393216, 131072]];
  const dfu = { flash, state: 2, ptr: FL, erased: [], left: false, writes: 0, pend: null };
  window.__dfu = dfu;
  const dv = (arr) => ({ data: new DataView(arr.buffer), status: "ok" });
  const dev = {
    vendorId: 0x0483, productId: 0xDF11, configuration: null,
    async open() {}, async close() {},
    async selectConfiguration() { this.configuration = { interfaces: [{ alternates: [{ interfaceName: "@Internal Flash  /0x08000000/04*016Kg,01*064Kg,03*128Kg" }] }] }; },
    async claimInterface() {}, async selectAlternateInterface() {},
    async controlTransferOut(s, data) {
      const d = !data ? new Uint8Array(0) : ArrayBuffer.isView(data) ? new Uint8Array(data.buffer, data.byteOffset, data.byteLength) : new Uint8Array(data);
      if (s.request === 1) {
        if (s.value === 0) {
          const a = d[1] | d[2] << 8 | d[3] << 16 | d[4] << 24 >>> 0;
          if (d[0] === 0x21) dfu.ptr = a >>> 0;
          else if (d[0] === 0x41) {
            const off = (a >>> 0) - FL, sec = sectors.find(([o]) => o === off);
            if (!sec) { dfu.state = 10; return; }
            flash.fill(0xFF, sec[0], sec[0] + sec[1]);
            dfu.erased.push(off);
          }
          dfu.state = 3;
        } else if (d.length === 0) { dfu.left = true; dfu.state = 7; }
        else {
          const off = dfu.ptr - FL + (s.value - 2) * 2048;
          for (let i = 0; i < d.length; i++) if (flash[off + i] !== 0xFF) { dfu.state = 10; return; }  // not erased
          flash.set(d, off);
          dfu.writes++;
          dfu.state = 3;
        }
      } else if (s.request === 4 || s.request === 6) dfu.state = 2;
    },
    async controlTransferIn(s, len) {
      if (s.request === 3) {
        if (dfu.state === 7) throw new DOMException("The device was disconnected.", "NetworkError");
        if (dfu.state === 3) dfu.state = 4;
        else if (dfu.state === 4) dfu.state = 5;
        const st = dfu.state === 10 ? 10 : 0;
        return dv(new Uint8Array([dfu.state === 10 ? 4 : 0, dfu.state === 4 ? 3 : 0, 0, 0, dfu.state, 0]));
      }
      if (s.request === 2) {
        const off = dfu.ptr - FL + (s.value - 2) * len;
        return dv(flash.slice(off, off + len));
      }
      return dv(new Uint8Array(len));
    },
  };
  Object.defineProperty(navigator, "usb", { value: { getDevices: async () => (window.__dfuVisible ? [dev] : []), requestDevice: async () => dev } });
})();
"""


async def flash_esp(b, chip, vid, image, errs):
    dump = os.path.join(TMP, chip + ".bin")
    ctx = await b.new_context(viewport={"width": 1280, "height": 860})
    await ctx.add_init_script(FAKE_SERIAL)
    pg = await ctx.new_page()
    pg.on("pageerror", lambda e: errs.append(str(e)))
    pg.on("dialog", lambda d: asyncio.ensure_future(d.accept()))
    await pg.goto("http://127.0.0.1:%d/?usb" % PORT)
    await pg.evaluate("([b, v]) => { window.__bridge = b; window.__vid = v; }", [chip, vid])
    await pg.click("#connect .flOpen")
    await pg.set_input_files("#flFile", image)
    await pg.wait_for_function("document.getElementById('flInfo').textContent.length > 0")
    info = await pg.inner_text("#flInfo")
    await pg.click("#flGo")
    try:
        await pg.wait_for_function("document.getElementById('flLog').textContent.includes('Xong')", timeout=90000)
    except Exception:
        pass
    log = await pg.inner_text("#flLog")
    sig = await pg.evaluate("window.__port ? window.__port.signals : []")
    await pg.screenshot(path=os.path.join(SHOTS, "flash-%s.png" % chip))
    await ctx.close()
    for _ in range(40):  # the fake ROM writes its flash when the bridge closes
        if os.path.exists(dump + ".ops"):
            break
        time.sleep(0.25)
    return info, log, sig, dump


async def main():
    os.makedirs(SHOTS, exist_ok=True)
    for k, f in FW.items():
        if not os.path.exists(f):
            sys.exit("missing %s: build with pio run first" % f)
    # 0. the emulator itself agrees with esptool (ESP32 ROM, compressed write + MD5)
    ref = os.path.join(TMP, "esptool.bin")
    rom = subprocess.Popen([sys.executable, os.path.join(ROOT, "test/web/fake_esp_rom.py"), "--chip", "esp32", "--tcp", "4019", "--dump", ref])
    time.sleep(0.6)
    r = subprocess.run([sys.executable, "-m", "esptool", "--chip", "esp32", "--port", "socket://127.0.0.1:4019", "--before", "no-reset",
                        "--after", "no-reset", "--no-stub", "write-flash", "0x10000", FW["esp32"]], capture_output=True, text=True)
    rom.wait(timeout=10)
    img = open(FW["esp32"], "rb").read()
    check(r.returncode == 0 and open(ref, "rb").read()[0x10000:0x10000 + len(img)] == img,
          "reference: real esptool flashes the ROM emulator (hash verified)")

    env = dict(os.environ)
    env["AQ_TEST_BRIDGES"] = ";".join("%s=%s %s --chip %s --stdio --dump %s" % (c, sys.executable, os.path.join(ROOT, "test/web/fake_esp_rom.py"), c, os.path.join(TMP, c + ".bin"))
                                      for c in ("esp32", "esp32s3", "esp32c6"))
    srv = subprocess.Popen([sys.executable, os.path.join(ROOT, "tools", "configurator.py"), "--port", str(PORT), "--no-browser"],
                           stdout=subprocess.DEVNULL, env=env)
    time.sleep(1.0)
    try:
        async with async_playwright() as p:
            exe = os.environ.get("CHROMIUM")
            b = await p.chromium.launch(**({"executable_path": exe} if exe else {}))
            errs = []
            # 1. ESP32 (USB-UART bridge): app image -> ota_0, otadata erased
            info, log, sig, dump = await flash_esp(b, "esp32", 0x10C4, FW["esp32"], errs)
            fl = open(dump, "rb").read() if os.path.exists(dump) else b""
            img = open(FW["esp32"], "rb").read()
            ops = open(dump + ".ops").read().split() if os.path.exists(dump + ".ops") else []
            check("ESP32," in info and "ota_0" in info, "ESP32 image recognised: %s" % info[:70])
            check(fl[0x10000:0x10000 + len(img)] == img and fl[0xE000:0x10000] == b"\xff" * 0x2000 and "Xong" in log,
                  "ESP32: firmware written to 0x10000 (compressed, MD5 checked), otadata erased")
            check("10" in ops and "11" in ops and "13" in ops and [False, True] in sig and [True, False] in sig,
                  "ESP32: auto-reset into the ROM (DTR/RTS), compressed write, MD5 (ops %s...)" % " ".join(ops[:6]))
            # 2. wrong chip: an ESP32-S3 image offered to an ESP32
            info, log, sig, dump = await flash_esp(b, "esp32", 0x10C4, FW["esp32s3"], errs)
            check("dành cho ESP32-S3" in log, "ESP32-S3 image refused on an ESP32 board")
            # 3. ESP32-S3 (native USB-Serial-JTAG): 5-word FLASH_DEFL_BEGIN of the S3 ROM
            for f in (os.path.join(TMP, "esp32s3.bin"), os.path.join(TMP, "esp32s3.bin.ops")):
                if os.path.exists(f):
                    os.remove(f)
            info, log, sig, dump = await flash_esp(b, "esp32s3", 0x303A, FW["esp32s3"], errs)
            fl = open(dump, "rb").read() if os.path.exists(dump) else b""
            img = open(FW["esp32s3"], "rb").read()
            check(fl[0x10000:0x10000 + len(img)] == img and "ESP32-S3" in log, "ESP32-S3: firmware written over native USB")
            # 3b. ESP32-C6: chip found by GET_SECURITY_INFO (chip id 13), 8 MB flash
            if os.path.exists(FW["esp32c6"]):
                info, log, sig, dump = await flash_esp(b, "esp32c6", 0x303A, FW["esp32c6"], errs)
                fl = open(dump, "rb").read() if os.path.exists(dump) else b""
                img = open(FW["esp32c6"], "rb").read()
                check("ESP32-C6" in info and fl[0x10000:0x10000 + len(img)] == img and "Chip: ESP32-C6" in log,
                      "ESP32-C6: chip detected by security info, image (chip id 13) written")
                info, log, sig, dump = await flash_esp(b, "esp32c6", 0x303A, FW["esp32s3"], errs)
                check("dành cho ESP32-S3" in log, "ESP32-S3 image refused on an ESP32-C6 board")
            else:
                print("(no esp32c6 build: C6 flasher check skipped)")

            # 4. STM32 DfuSe over WebUSB
            ctx = await b.new_context(viewport={"width": 1280, "height": 860})
            await ctx.add_init_script(FAKE_SERIAL)
            pg = await ctx.new_page()
            pg.on("pageerror", lambda e: errs.append(str(e)))
            await pg.goto("http://127.0.0.1:%d/?usb" % PORT)
            await pg.evaluate("window.__dfuVisible = true")
            await pg.click("#connect .flOpen")
            await pg.click("#flTarget button[data-t=stm]")
            await pg.set_input_files("#flFile", FW["stm32"])
            await pg.wait_for_function("document.getElementById('flInfo').textContent.length > 0")
            info = await pg.inner_text("#flInfo")
            await pg.click("#flGo")
            await pg.wait_for_function("window.__dfu.left || document.getElementById('flLog').textContent.includes('Lỗi')", timeout=60000)
            img = open(FW["stm32"], "rb").read()
            md5 = await pg.evaluate("md5hex(window.__dfu.flash.subarray(0, %d))" % len(img))
            erased = await pg.evaluate("window.__dfu.erased")
            last = await pg.evaluate("window.__dfu.flash[393216 + 10] === (((393216 + 10) * 7 + 3) & 255)")
            log = await pg.inner_text("#flLog")
            await pg.screenshot(path=os.path.join(SHOTS, "flash-stm32.png"))
            need = [o for o in (0, 16384, 32768, 49152, 65536, 131072, 262144, 393216) if o < len(img)]
            check("STM32" in info, "STM32 image recognised")
            check(md5 == hashlib.md5(img).hexdigest() and erased == need and last and "Khớp" in log,
                  "STM32 DFU: %d sectors erased, %d kB written and read back, config sector kept, left DFU" % (len(erased), len(img) // 1024))
            bad = os.path.join(TMP, "bad.bin")
            open(bad, "wb").write(bytes(1000))
            await pg.set_input_files("#flFile", bad)
            await pg.wait_for_timeout(300)
            check(await pg.is_disabled("#flGo") and "Không phải" in await pg.inner_text("#flInfo"), "a file that is not STM32 firmware is refused")
            await ctx.close()
            check(not errs, "no JavaScript errors %s" % errs[:3])
            await b.close()
    finally:
        srv.terminate()
    print("\n" + ("all flasher checks passed" if not fails else "%d flasher checks failed" % len(fails)))
    sys.exit(1 if fails else 0)


asyncio.run(main())
