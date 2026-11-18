#!/usr/bin/env python3
"""PC configurator over USB, end to end: the page from tools/configurator.py talks to the real
firmware (build/sim serve) through
  1. the "SITL" button (WebSocket bridge), and
  2. the Web Serial code path, with navigator.serial replaced by a fake port whose bytes go to the
     same simulator (so the COM-port code runs exactly as with a cable).
Needs build/sim (tools/build_host.sh) and Playwright + Chromium.
    python3 test/web/test_configurator.py [screenshot_dir]
"""
import asyncio, os, subprocess, sys, time
from playwright.async_api import async_playwright

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SHOTS = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "build", "web", "shots")
PORT = 8181
fails = []



async def navto(pg, name):
    """open a tab; on a phone the less used tabs sit behind the "Thêm" button"""
    sel = "nav button[data-tab=%s]" % name
    if not await pg.locator(sel).is_visible() and await pg.locator("#navMore").is_visible():
        await pg.click("#navMore")
    await pg.click(sel)

def check(ok, msg):
    print("[%s] %s" % ("PASS" if ok else "FAIL", msg))
    if not ok:
        fails.append(msg)


# navigator.serial stand-in: one port, bytes carried to the simulator over the launcher's /sitl socket
FAKE_SERIAL = """
(() => {
  class FakePort extends EventTarget {
    async open(opt) {
      this.baud = opt.baudRate;
      window.__opened = (window.__opened || 0) + 1;
      if (window.__busy) { throw new DOMException("Failed to open serial port.", "NetworkError"); }
      const ws = new WebSocket(location.origin.replace(/^http/, "ws") + "/sitl");
      ws.binaryType = "arraybuffer";
      this.ws = ws;
      await new Promise((r, j) => { ws.onopen = r; ws.onerror = j; });
      let ctl;
      this.readable = new ReadableStream({ start(c) { ctl = c; } });
      ws.onmessage = (ev) => ctl.enqueue(new Uint8Array(ev.data));
      ws.onclose = () => { try { ctl.error(new DOMException("The device has been lost.", "NetworkError")); } catch (e) {} };
      this.writable = new WritableStream({ write(chunk) { ws.send(chunk); } });
      // a line error (framing error / break): the current reader fails, Web Serial makes a new one
      this.lineError = (name) => {
        const old = ctl;
        this.readable = new ReadableStream({ start(c) { ctl = c; } });
        old.error(new DOMException("line error", name || "FramingError"));
      };
    }
    async close() { try { this.ws.close(); } catch (e) {} }
    getInfo() { return { usbVendorId: 0x10c4, usbProductId: 0xea60 }; }
  }
  const port = new FakePort();
  window.__port = port;
  const serial = new EventTarget();
  serial.requestPort = async () => port;
  serial.getPorts = async () => (window.__granted ? [port] : []);
  Object.defineProperty(navigator, "serial", { value: serial });
})();
"""


async def main():
    os.makedirs(SHOTS, exist_ok=True)
    if not os.access(os.path.join(ROOT, "build", "sim"), os.X_OK):
        sys.exit("build/sim missing: run tools/build_host.sh")
    srv = subprocess.Popen([sys.executable, os.path.join(ROOT, "tools", "configurator.py"), "--port", str(PORT), "--no-browser"],
                           stdout=subprocess.DEVNULL)
    time.sleep(1.0)
    try:
        async with async_playwright() as p:
            exe = os.environ.get("CHROMIUM")
            b = await p.chromium.launch(**({"executable_path": exe} if exe else {}))
            errs = []

            # ---------------- 0. page as served by the WiFi bridge (?bridge): connects through /link by itself
            pg = await b.new_page(viewport={"width": 412, "height": 900}, is_mobile=True, has_touch=True)
            pg.on("pageerror", lambda e: errs.append(str(e)))
            await pg.goto("http://127.0.0.1:%d/?bridge" % PORT)
            await pg.wait_for_function("LINK.open && LINK.kind === 'bridge' && Object.keys(S.params).length > 100", timeout=15000)
            await pg.wait_for_function("S.status && S.status.mode", timeout=8000)
            check(await pg.is_hidden("#connect") and "Cầu WiFi" in await pg.inner_text("#linkRate"),
                  "WiFi bridge page: connected to the FC through /link without a click")
            await pg.screenshot(path=os.path.join(SHOTS, "bridge.png"))
            await navto(pg, "system")
            await pg.wait_for_selector("#brSsid", timeout=5000)
            await pg.wait_for_function("$('brSsid').value === 'AllQuad-Link-SIM'", timeout=3000)
            check("mặc định" in await pg.inner_text("#brWarn"), "WiFi bridge: System tab warns about the default password")
            await pg.fill("#brSsid", "MyQuad")
            await pg.fill("#brPass", "secret-pass-1")
            await pg.click("#brSave")
            await pg.wait_for_timeout(500)
            info = await pg.evaluate("fetch('/bridge/info').then(r => r.json())")
            check(info["ssid"] == "MyQuad" and not info["defpw"], "WiFi bridge: name / password saved to the bridge")
            await pg.close()

            # ---------------- 1. simulator button, laptop-sized window
            pg = await b.new_page(viewport={"width": 1280, "height": 800})
            pg.on("pageerror", lambda e: errs.append(str(e)))
            await pg.goto("http://127.0.0.1:%d/?usb" % PORT)
            await pg.wait_for_selector("#connect:not([hidden])", timeout=5000)
            await pg.wait_for_selector("#sitlBtn:not([hidden])", timeout=3000)
            check(await pg.is_visible("#comPick") and not await pg.is_visible("#login"),
                  "laptop: connect screen (COM port, simulator), no WiFi login")
            await pg.screenshot(path=os.path.join(SHOTS, "connect.png"))
            await pg.click("#sitlBtn")
            await pg.wait_for_function("document.querySelectorAll('#pidMatrix input').length === 12", timeout=10000)
            await pg.wait_for_timeout(1200)
            check(await pg.is_hidden("#connect") and "SITL" in await pg.inner_text("#sSub") and "ANGLE" in await pg.inner_text("#sMode"),
                  "simulator: parameters and live status over the AQ link")
            n = await pg.evaluate("Object.keys(S.params).length")
            check(n > 200, "all %d parameters loaded through 2 kB pages" % n)
            # change a gain, apply, read it back from the firmware
            await pg.evaluate("setEdit('roll_p', 63)")
            await pg.click("#applyBtn")
            await pg.wait_for_timeout(600)
            back = await pg.evaluate("api('/api/params').then(j => j.params.find(p => p.n === 'roll_p').v)")
            check(abs(back - 63) < 1e-3, "PID change applied in the firmware (roll_p = %s)" % back)
            # CLI lines inside a frame body must never run as CLI commands (shared console)
            await pg.evaluate("linkRequest('/api/nope', '\\nset roll_p 11\\nsave\\n', 1)")
            await pg.wait_for_timeout(300)
            back = await pg.evaluate("api('/api/params').then(j => j.params.find(p => p.n === 'roll_p').v)")
            check(abs(back - 63) < 1e-3, "CLI text inside a frame is not executed (roll_p still %s)" % back)
            await navto(pg, "system")
            await pg.wait_for_timeout(1500)
            check("telemetry" in await pg.inner_text("#taskTable") and await pg.is_hidden("#otaPick")
                  and await pg.is_visible("#comClose") and await pg.is_hidden("#logoutBtn"),
                  "system tab: firmware task table, no OTA / logout over USB")
            async with pg.expect_download() as dl:
                await pg.click("#backupBtn")
            backup = open(await (await dl.value).path()).read()
            check("set roll_p 63" in backup, "backup downloaded over the link")
            edited = backup.replace("set roll_p 63", "set roll_p 58") + "set pitch_p 59\n" + "".join("# filler line %03d to make the file longer than one piece\n" % i for i in range(30))
            fpath = os.path.join(SHOTS, "usb-restore.txt")
            open(fpath, "w").write(edited)
            pg.once("dialog", lambda d: asyncio.ensure_future(d.accept()))
            await pg.set_input_files("#restoreFile", fpath)
            await pg.wait_for_function("document.getElementById('restoreMsg').textContent.includes('Đã khôi phục')", timeout=15000)
            await pg.wait_for_timeout(600)
            check(await pg.evaluate("S.params.roll_p.v === 58 && S.params.pitch_p.v === 59"),
                  "restore of a %d-byte file uploaded in pieces" % len(edited))
            await pg.screenshot(path=os.path.join(SHOTS, "usb-system.png"), full_page=True)
            await pg.click("#comClose")
            await pg.wait_for_selector("#connect:not([hidden])", timeout=3000)
            check("ngắt" in (await pg.inner_text("#comState")).lower(), "disconnect button returns to the connect screen")
            await pg.close()

            # ---------------- 2. Web Serial path (fake port -> same firmware)
            ctx = await b.new_context(viewport={"width": 390, "height": 844})
            await ctx.add_init_script(FAKE_SERIAL)
            pg = await ctx.new_page()
            pg.on("pageerror", lambda e: errs.append(str(e)))
            await pg.goto("http://127.0.0.1:%d/?usb" % PORT)
            await pg.wait_for_selector("#connect:not([hidden])")
            await pg.evaluate("window.__busy = true")
            await pg.click("#comPick")
            await pg.wait_for_selector("#comErr:not([hidden])", timeout=5000)
            check("chương trình khác" in await pg.inner_text("#comErr"), "port held by another program: clear message")
            await pg.evaluate("window.__busy = false")
            await pg.click("#comPick")
            await pg.wait_for_function("document.querySelectorAll('#pidMatrix input').length === 12", timeout=10000)
            await pg.wait_for_timeout(1000)
            check("COM" in await pg.inner_text("#sSub") and await pg.evaluate("LINK.kind === 'usb' && LINK.baud === 921600"),
                  "COM port opened at 921600, configurator connected")
            t0 = await pg.evaluate("S.lastStatusMs")
            await pg.wait_for_timeout(1000)
            rate = await pg.evaluate("S.lastStatusMs") - t0
            check(rate > 0 and await pg.evaluate("Date.now() - S.lastStatusMs < 500"), "live status keeps coming over the port")
            await navto(pg, "modes")
            await pg.wait_for_timeout(500)
            await pg.screenshot(path=os.path.join(SHOTS, "usb-modes.png"))
            # framing error / break on the line (e.g. an ESP32 printing while it resets): not a disconnect
            await pg.evaluate("window.__port.lineError('FramingError')")
            await pg.wait_for_timeout(300)
            await pg.evaluate("window.__port.lineError('BreakError')")
            await pg.wait_for_timeout(800)
            cli = await pg.evaluate("api('/api/cli', { method: 'POST', body: 'get roll_p', text: true, timeout: 4000 })")
            check(await pg.is_hidden("#connect") and await pg.evaluate("LINK.open && Date.now() - S.lastStatusMs < 500") and "roll_p" in cli,
                  "framing error / break on the line: link keeps running, CLI answers (%s)" % cli.strip())
            # cable pulled: the port errors out -> connect screen; plugged back: reconnects by itself
            await pg.evaluate("window.__port.ws.close()")
            await pg.wait_for_selector("#connect:not([hidden])", timeout=5000)
            check("Mất kết nối USB" in await pg.inner_text("#comErr"), "cable pulled: connect screen with a message")
            await pg.evaluate("window.__granted = true")
            await pg.evaluate("""(() => { const e = new Event('connect'); Object.defineProperty(e, 'target', { value: window.__port });
                                       navigator.serial.dispatchEvent(e); })()""")
            await pg.wait_for_selector("#connect", state="hidden", timeout=10000)
            check(await pg.evaluate("LINK.open"), "plugged back in: reconnects without a click")
            check(not errs, "no JavaScript errors %s" % errs[:3])
            await b.close()
    finally:
        srv.terminate()
    print("\n" + ("all configurator checks passed" if not fails else "%d configurator checks failed" % len(fails)))
    sys.exit(1 if fails else 0)


asyncio.run(main())
