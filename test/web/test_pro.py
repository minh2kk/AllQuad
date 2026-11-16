#!/usr/bin/env python3
"""Configurator v3 tools (src/web/ui/pro.js), Playwright + Chromium:
  part 1 against tools/web_mock.py: pre-flight checklist, review / undo, save reminder, drone profiles,
         global search, simple view, theme, compact sidebar, RC link panel, blackbox heatmap + events,
         mission statistics
  part 2 on build/web/AllQuad-Configurator.html: USB link through WebUSB (a fake CDC device whose
         bulk endpoints are wired to the in-browser demo firmware), as on a phone without Web Serial
    python3 test/web/test_pro.py [screenshot_dir]
Needs build/web/normal.bin, the demo build (tools/build_wasm.sh) and tools/embed_web.py output."""
import asyncio, os, subprocess, sys, time
from playwright.async_api import async_playwright

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SHOTS = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "build", "web", "shots")
PAGE = os.path.join(ROOT, "build", "web", "AllQuad-Configurator.html")
PORT = 8099
fails = []


def check(ok, msg):
    print("[%s] %s" % ("PASS" if ok else "FAIL", msg))
    if not ok:
        fails.append(msg)


async def tab(pg, name):
    await pg.click("nav button[data-tab=%s]" % name)
    await pg.wait_for_timeout(350)


# a WebUSB device (CDC ACM: comm interface 0, data interface 1 with bulk EP1 in / out) whose data
# goes to and comes from the demo firmware worker; navigator.serial is hidden as on older Android
FAKE_USB = r"""
(() => {
  try { Object.defineProperty(Navigator.prototype, "serial", { get: () => undefined, configurable: true }); delete Navigator.prototype.serial; } catch (e) {}
  const inQ = [], waiters = [];
  let worker = null;
  window.__usbLog = [];
  const dev = {
    productName: "AllQuad", vendorId: 0x0483, productId: 0x5740, configuration: null, opened: false,
    async open() { this.opened = true;
      worker = new Worker(URL.createObjectURL(new Blob([DEMO_WORKER], { type: "text/javascript" })));
      await new Promise((res) => { worker.onmessage = (ev) => { const m = ev.data;
        if (m.ready) res();
        if (m.out) { const w = waiters.shift(); if (w) w(m.out); else inQ.push(m.out); } };
        worker.postMessage({ wasm: demoBytes() }); }); },
    async selectConfiguration() { this.configuration = { interfaces: [
      { interfaceNumber: 0, alternate: { interfaceClass: 2, endpoints: [{ type: "interrupt", direction: "in", endpointNumber: 2 }] } },
      { interfaceNumber: 1, alternate: { interfaceClass: 10, endpoints: [{ type: "bulk", direction: "in", endpointNumber: 1 }, { type: "bulk", direction: "out", endpointNumber: 1 }] } }] }; },
    async claimInterface(n) { window.__usbLog.push("claim " + n); },
    async controlTransferOut(s) { window.__usbLog.push("ctl " + s.request.toString(16)); return { status: "ok" }; },
    async transferOut(ep, data) { worker.postMessage({ input: new Uint8Array(data) }); return { status: "ok", bytesWritten: data.length }; },
    async transferIn(ep, len) { const d = inQ.shift() || await new Promise((r) => waiters.push(r));
      return { status: "ok", data: new DataView(d.buffer, d.byteOffset, d.byteLength) }; },
    async close() { if (worker) worker.terminate(); this.opened = false; },
  };
  const usb = { async requestDevice() { return dev; }, async getDevices() { return [dev]; }, addEventListener() {} };
  Object.defineProperty(Navigator.prototype, "usb", { get: () => usb, configurable: true });
})();
"""


async def part_mock(b):
    pg = await b.new_page(viewport={"width": 1400, "height": 1000})
    errs = []
    pg.on("pageerror", lambda e: errs.append(str(e)))
    await pg.goto("http://127.0.0.1:%d/" % PORT)
    await pg.wait_for_selector("#login:not([hidden])", timeout=8000)
    await pg.fill("#loginPw", "allquad123")
    await pg.click("#loginForm button[type=submit]")
    await pg.wait_for_function("S.status && Object.keys(S.params).length > 100", timeout=8000)
    await pg.wait_for_timeout(800)

    # status-rate pill
    rate = await pg.inner_text("#linkRate")
    check("Hz" in rate, "strip: status rate pill (%s)" % rate)

    # pre-flight
    await tab(pg, "preflight")
    await pg.wait_for_timeout(600)
    rows = await pg.evaluate("document.querySelectorAll('#pfAuto .pf-row').length")
    bad = await pg.evaluate("[...document.querySelectorAll('#pfAuto .pf-row.bad b')].map(e => e.textContent)")
    check(rows >= 10 and any("arm" in x for x in bad), "pre-flight: %d automatic checks, arming condition flagged (%s)" % (rows, bad))
    await pg.click("#pfManual [data-pf=props]")
    await pg.wait_for_timeout(300)
    check("1/7" in await pg.inner_text("#pfSum"), "pre-flight: manual tick counted and kept")
    await pg.click("#pfAuto [data-go=sensors]")
    await pg.wait_for_timeout(300)
    check(await pg.evaluate("!$('tab-sensors').hidden"), "pre-flight: Sửa opens the tab to fix")
    await pg.screenshot(path=os.path.join(SHOTS, "pro-preflight.png"))

    # review / undo / apply / save reminder
    await pg.evaluate("setEdit('rth_alt', 35); setEdit('roll_p', 50); setEdit('airmode', 0)")
    await pg.click("#reviewBtn")
    await pg.wait_for_timeout(300)
    n = await pg.evaluate("document.querySelectorAll('#reviewDlg .rv-row').length")
    check(n == 3, "review: 3 pending changes listed")
    await pg.click("#reviewDlg [data-undo=airmode]")
    await pg.wait_for_timeout(200)
    check(await pg.evaluate("!('airmode' in S.edits) && document.querySelectorAll('#reviewDlg .rv-row').length === 2"), "review: one change undone")
    await pg.click("#rvApply")
    await pg.wait_for_timeout(800)
    check(await pg.is_visible("#saveNowBtn") and "chưa lưu" in await pg.inner_text("#applyText"), "applied but not saved: reminder bar")
    await pg.click("#saveNowBtn")
    await pg.wait_for_timeout(600)
    check(not await pg.is_visible("#applyBar"), "save: reminder gone")

    # drone profiles
    await tab(pg, "params")
    await pg.fill("#profName", "5 inch")
    await pg.click("#profSave")
    await pg.evaluate("S.params.rth_alt.v = 60; profRender()")
    await pg.click("#profRows [data-pl='0']")
    check(await pg.evaluate("S.edits.rth_alt === 35"), "profiles: loading queues the stored values")
    await pg.evaluate("$('discardBtn').click(); S.params.rth_alt.v = 35")
    skip = await pg.evaluate("!('acc_calibrated' in profList()['5 inch'].v) && !('mag_bias_x' in profList()['5 inch'].v)")
    check(skip, "profiles: calibration values not stored")

    # global search
    await pg.keyboard.press("Control+k")
    await pg.fill("#gsIn", "rth_alt")
    await pg.wait_for_timeout(200)
    await pg.keyboard.press("Enter")
    await pg.wait_for_timeout(700)
    check(await pg.evaluate("!$('tab-nav').hidden && document.activeElement && document.activeElement.dataset.p === 'rth_alt'"), "search: Ctrl+K → rth_alt focused on its tab")

    # simple view
    total = await pg.evaluate("document.querySelectorAll('#tab-pid .prow').length")
    await pg.click("#viewBtn")
    await tab(pg, "pid")
    vis = await pg.evaluate("[...document.querySelectorAll('#tab-pid .prow')].filter(r => r.offsetParent).length")
    check(0 < vis < total, "simple view: %d of %d rows shown" % (vis, total))
    await pg.click("#viewBtn")

    # RC link panel
    await tab(pg, "modes")
    await pg.wait_for_timeout(800)
    check("100%" in await pg.inner_text("#rxStats"), "RC link: statistics shown")

    # blackbox heatmap + events
    await tab(pg, "blackbox")
    await pg.click("#bbDownload")
    await pg.wait_for_selector("#bbEvents .ev", timeout=15000)
    rows = await pg.evaluate("PRO.heat[0] ? PRO.heat[0].rows.filter(Boolean).length : 0")
    check(rows >= 1, "blackbox: noise vs throttle heatmap (%d throttle bands)" % rows)
    await pg.click("#bbEvents .ev >> nth=1")
    check(await pg.evaluate("+$('tsPos').value >= 0"), "blackbox: event moves the time view")
    await pg.screenshot(path=os.path.join(SHOTS, "pro-blackbox.png"), full_page=True)

    # mission statistics
    await tab(pg, "nav")
    await pg.click("#wpEditBtn")
    await pg.wait_for_selector("#wpStats", timeout=4000)
    st = await pg.inner_text("#wpStats")
    check("m" in st and ":" in st, "mission: distance / time / battery (%s)" % st.replace("\n", " "))
    await pg.click("#wpCancel")

    # theme + compact sidebar
    for _ in range(3):
        await pg.click("#themeQuick")
    check(await pg.evaluate("document.documentElement.dataset.theme === 'sun'"), "theme: outdoor theme reachable")
    await pg.click("#themeQuick")
    await pg.click("#navMiniBtn")
    w = await pg.evaluate("document.querySelector('nav.tabs').getBoundingClientRect().width")
    check(w < 100, "compact sidebar (%d px)" % w)
    await pg.click("#navMiniBtn")
    check(not errs, "mock part: no page errors %s" % errs)
    await pg.close()


async def part_webusb(b):
    if not os.path.exists(PAGE):
        check(False, "AllQuad-Configurator.html missing (tools/embed_web.py)")
        return
    pg = await b.new_page(viewport={"width": 412, "height": 900}, is_mobile=True, has_touch=True)
    errs = []
    pg.on("pageerror", lambda e: errs.append(str(e)))
    await pg.add_init_script("window.__fakeUsb = true;")
    await pg.goto("file://" + PAGE)
    await pg.evaluate(FAKE_USB)
    await pg.evaluate("showConnect()")
    check(await pg.is_visible("#usbPick") and not await pg.is_visible("#comPick"), "phone without Web Serial: USB (WebUSB) button offered")
    await pg.click("#usbPick")
    await pg.wait_for_function("LINK.open && LINK.usbdev && Object.keys(S.params).length > 100", timeout=20000)
    log = await pg.evaluate("__usbLog")
    check("claim 1" in log and "ctl 22" in log, "WebUSB: data interface claimed, DTR set (%s)" % log)
    await pg.wait_for_function("S.status && S.status.mode", timeout=8000)
    check(await pg.evaluate("S.linkOk"), "WebUSB: live status from the firmware")
    await pg.screenshot(path=os.path.join(SHOTS, "pro-webusb.png"))
    await pg.evaluate("$('comClose').click()")  # in the System tab
    await pg.wait_for_timeout(400)
    check(await pg.evaluate("!LINK.open && !LINK.usbdev"), "WebUSB: disconnect closes the device")
    check(not errs, "WebUSB part: no page errors %s" % errs)
    await pg.close()


async def main():
    os.makedirs(SHOTS, exist_ok=True)
    async with async_playwright() as p:
        exe = os.environ.get("CHROMIUM")
        b = await p.chromium.launch(**({"executable_path": exe} if exe else {}))
        await part_mock(b)
        await part_webusb(b)
        await b.close()


srv = subprocess.Popen([sys.executable, os.path.join(ROOT, "tools", "web_mock.py"), str(PORT)])
time.sleep(1.5)
try:
    asyncio.run(main())
finally:
    srv.terminate()
print("all v3 tool checks passed" if not fails else "%d FAILED" % len(fails))
sys.exit(1 if fails else 0)
