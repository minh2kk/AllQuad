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

    bad = await pg.evaluate("[...document.querySelectorAll('#pfAuto .pf-row.bad b')].map(e => e.textContent)")
    check("Lần khởi động trước" in bad, "pre-flight: a crash reset is flagged")
    # reset reason / crash record (System tab)
    await tab(pg, "system")
    await pg.wait_for_selector("#relCrash .note", timeout=4000)
    txt = await pg.inner_text("#relPanel")
    check("LỖI FIRMWARE" in txt and "telemetry" in txt and "0x0800F4E6" in txt and "watchdog: đang chạy" in txt, "system: reset reason, crash record, watchdog")
    await pg.screenshot(path=os.path.join(SHOTS, "pro-crash.png"))
    await pg.click("#relClear")
    await pg.wait_for_function("!S.status.crash", timeout=5000)
    await pg.wait_for_timeout(400)
    check("không có" in await pg.inner_text("#relPanel"), "system: crash record cleared")

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

    # per-UART traffic / errors (Ports tab)
    await tab(pg, "ports")
    await pg.wait_for_function("document.querySelectorAll('#psTable tr').length >= 4", timeout=6000)
    await pg.wait_for_timeout(2500)
    txt = await pg.inner_text("#psTable")
    check("UART1" in txt and "UART2" in txt and "Receiver" in txt and "khung/s" in txt, "ports: per-UART traffic and receiver frames")
    check("UBX 115200 baud" in txt, "ports: GPS protocol and baud found")
    check(await pg.evaluate("PS.rxHz > 100 && PS.rate[0] && PS.rate[0][0] > 1000"), "ports: rates from the counters (rx %s)" % await pg.evaluate("PS.rxHz"))
    await pg.screenshot(path=os.path.join(SHOTS, "pro-ports.png"), full_page=True)
    await pg.click("#psReset")
    check("0 lỗi" in await pg.inner_text("#psTable"), "ports: reset starts the error count over")

    # PID tuning assistant: frame -> test flight -> analysis round
    await pg.evaluate("localStorage.removeItem('aq-tune')")
    await tab(pg, "pid")
    await pg.wait_for_selector("#tunePanel [data-tf='5']", timeout=4000)
    await pg.click("#tunePanel [data-tf='5']")
    await pg.wait_for_selector("#tuneFlown", timeout=3000)
    check(await pg.evaluate("tuneState().step === 1 && Object.keys(S.edits).length >= 0"), "tuning assistant: frame chosen, test-flight step")
    await pg.click("#tuneFlown")
    await pg.wait_for_timeout(400)
    check(await pg.evaluate("!$('tab-blackbox').hidden"), "tuning assistant: goes to the blackbox tab")
    await pg.evaluate("$('discardBtn') && $('discardBtn').click()")

    # blackbox heatmap + events
    await tab(pg, "blackbox")
    await pg.click("#bbDownload")
    await pg.wait_for_selector("#bbEvents .ev", timeout=15000)
    rows = await pg.evaluate("PRO.heat[0] ? PRO.heat[0].rows.filter(Boolean).length : 0")
    check(rows >= 1, "blackbox: noise vs throttle heatmap (%d throttle bands)" % rows)
    await pg.click("#bbEvents .ev >> nth=1")
    check(await pg.evaluate("+$('tsPos').value >= 0"), "blackbox: event moves the time view")
    path = await pg.evaluate("$('bbPathPanel') && !$('bbPathPanel').hidden ? $('bbPathInfo').textContent : ''")
    check("m bay" in path, "blackbox: flight path from the logged position (%s)" % path)
    if path:
        await pg.locator("#bbPathPanel").screenshot(path=os.path.join(SHOTS, "pro-path.png"))
    t = await pg.evaluate("tuneState()")
    check(t["step"] == 2 and len(t["rounds"]) == 1, "tuning assistant: the analysed log is round 1")
    await pg.click("#bbPin")
    check(await pg.evaluate("bbRefSpec(0).length === 2"), "compare: pinned flight's spectrum overlaid")
    await pg.click("#bbUnpin")
    await tab(pg, "pid")
    await pg.wait_for_selector("#tunePanel table", timeout=3000)
    check("Vòng" in await pg.inner_text("#tunePanel"), "tuning assistant: round table on the PID tab")
    await pg.screenshot(path=os.path.join(SHOTS, "pro-tune.png"))
    await tab(pg, "blackbox")
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


async def part_pages(b):
    """GitHub Pages copy: firmware offered from firmware/manifest.json, downloaded and checked"""
    import http.server, threading, shutil, tempfile, json, functools
    d = tempfile.mkdtemp()
    if subprocess.call([sys.executable, os.path.join(ROOT, "tools", "make_pages.py"), d], stdout=subprocess.DEVNULL) != 0:
        check(False, "make_pages.py")
        return
    man = json.load(open(os.path.join(d, "firmware", "manifest.json")))
    stm = [f for f in man["files"] if f["target"] == "stm"]
    if not stm:
        print("(no STM32 build in .pio: Pages firmware check skipped)")
        return
    # a second, corrupted copy of the first STM32 file (SHA-256 must catch it)
    bad = dict(stm[0], file="corrupt.bin", name="corrupt")
    open(os.path.join(d, "firmware", "corrupt.bin"), "wb").write(b"\0" * bad["size"])
    man["files"].append(bad)
    json.dump(man, open(os.path.join(d, "firmware", "manifest.json"), "w"))
    class Quiet(http.server.SimpleHTTPRequestHandler):
        def log_message(self, *a):
            pass
    srv = http.server.ThreadingHTTPServer(("127.0.0.1", 8097), functools.partial(Quiet, directory=d))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    pg = await b.new_page(viewport={"width": 1200, "height": 900})
    errs = []
    pg.on("pageerror", lambda e: errs.append(str(e)))
    try:
        await pg.goto("http://127.0.0.1:8097/")
        await pg.wait_for_selector("#connect:not([hidden])", timeout=8000)
        await pg.click("#connect .flOpen")
        await pg.click("#flTarget [data-t=stm]")
        await pg.wait_for_selector("#flRel:not([hidden])", timeout=5000)
        n = await pg.evaluate("$('flRelSel').options.length")
        check(n == len(stm) + 1, "Pages: %d STM32 firmware files offered from the manifest" % n)
        await pg.click("#flRelGet")
        await pg.wait_for_function("/STM32/.test($('flInfo').textContent)", timeout=8000)
        check(await pg.evaluate("!$('flGo').disabled && FL.data.length") == stm[0]["size"], "Pages: firmware downloaded, size + SHA-256 checked, ready to flash")
        await pg.select_option("#flRelSel", str(len(man["files"]) - 1))
        await pg.click("#flRelGet")
        await pg.wait_for_function("/SHA-256/.test($('flInfo').textContent)", timeout=8000)
        check(await pg.evaluate("$('flGo').disabled"), "Pages: a file that does not match its SHA-256 is refused")
        await pg.screenshot(path=os.path.join(SHOTS, "pro-pages.png"))
        check(not errs, "Pages part: no page errors %s" % errs)
    finally:
        srv.shutdown()
        shutil.rmtree(d, ignore_errors=True)
        await pg.close()


async def main():
    os.makedirs(SHOTS, exist_ok=True)
    async with async_playwright() as p:
        exe = os.environ.get("CHROMIUM")
        b = await p.chromium.launch(**({"executable_path": exe} if exe else {}))
        await part_mock(b)
        await part_webusb(b)
        await part_pages(b)
        await b.close()


srv = subprocess.Popen([sys.executable, os.path.join(ROOT, "tools", "web_mock.py"), str(PORT)])
time.sleep(1.5)
try:
    asyncio.run(main())
finally:
    srv.terminate()
print("all v3 tool checks passed" if not fails else "%d FAILED" % len(fails))
sys.exit(1 if fails else 0)
