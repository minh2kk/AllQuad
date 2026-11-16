#!/usr/bin/env python3
"""Web configurator UI test (Playwright + Chromium) against tools/web_mock.py.
   pip install playwright ; python3 test/web/test_ui.py [screenshot_dir]
Needs build/tests (tools/build_host.sh) and build/web/normal.bin (node test/web/test_analysis.js)."""
import asyncio, json, os, subprocess, sys, time
from playwright.async_api import async_playwright

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SHOTS = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "build", "web", "shots")
PORT = 8097
fails = []

def check(ok, msg):
    print("[%s] %s" % ("PASS" if ok else "FAIL", msg))
    if not ok: fails.append(msg)

async def main():
    os.makedirs(SHOTS, exist_ok=True)
    async with async_playwright() as p:
        exe = os.environ.get("CHROMIUM")
        b = await p.chromium.launch(**({"executable_path": exe} if exe else {}))
        pg = await b.new_page(viewport={"width": 390, "height": 844})
        errs = []
        pg.on("pageerror", lambda e: errs.append(str(e)))
        await pg.goto("http://127.0.0.1:%d/" % PORT)
        await pg.wait_for_selector("#login:not([hidden])", timeout=8000)
        await pg.fill("#loginPw", "nope")
        await pg.click("#loginForm button[type=submit]")
        await pg.wait_for_timeout(400)
        check(await pg.is_visible("#loginErr") and await pg.is_visible("#login"), "login screen: wrong password refused")
        await pg.screenshot(path=os.path.join(SHOTS, "login.png"))
        await pg.fill("#loginPw", "allquad123")
        await pg.click("#loginForm button[type=submit]")
        await pg.wait_for_function("document.querySelectorAll('#pidMatrix input').length === 12")
        await pg.wait_for_timeout(800)
        check("ANGLE" in await pg.inner_text("#sMode"), "live status shown in the header")
        check(await pg.is_visible("#defPw"), "factory-password warning shown")
        import urllib.request
        ws = json.loads(urllib.request.urlopen("http://127.0.0.1:%d/mock/ws_clients" % PORT).read())["n"]
        polls = await pg.evaluate("S.polling === true")
        check(ws == 1 and not polls, "live status over one WebSocket (no polling)")
        await pg.click("nav button[data-tab=pid]")
        before = await pg.input_value("#pidMatrix input[data-p=roll_p]")
        await pg.locator("#pidMatrix .stepper").first.locator("button").last.click()
        after = await pg.input_value("#pidMatrix input[data-p=roll_p]")
        check(float(after) == float(before) + 1 and await pg.is_visible("#applyBar"), "PID + button stages a change")
        await pg.click("#applyBtn")
        await pg.wait_for_timeout(300)
        # applied: the bar turns into the "not saved to flash yet" reminder
        check(not await pg.is_visible("#applyBtn") and await pg.is_visible("#saveNowBtn"), "change applied (save reminder shown)")
        await pg.screenshot(path=os.path.join(SHOTS, "pid.png"), full_page=True)
        await pg.click("nav button[data-tab=modes]")
        rows = await pg.locator("#modeList .mrow").count()
        await pg.click("#modeAdd")
        last = pg.locator("#modeList .mrow").last
        await last.locator("select").first.select_option("3")   # HORIZON
        await last.locator("select").nth(1).select_option("10")  # AUX7
        await last.locator(".seg button").nth(1).click()          # middle position
        await pg.click("#applyBtn")
        await pg.wait_for_timeout(1200)
        check(rows == 9 and "HORIZON" in await pg.inner_text("#modeActive"),
              "modes: default 9 ranges, new HORIZON range on AUX7 becomes active")
        await pg.screenshot(path=os.path.join(SHOTS, "modes.png"), full_page=True)
        await pg.click("nav button[data-tab=nav]")
        await pg.wait_for_timeout(1500)
        kv = await pg.inner_text("#navKv")
        check(await pg.evaluate("S.mission && S.mission.items.length === 9 && S.mission.rally.length === 1")
              and "Tây Nam" in kv and "compass" in kv, "navigation: mission loaded, wind & heading shown")
        await pg.screenshot(path=os.path.join(SHOTS, "nav.png"))
        await pg.click("nav button[data-tab=blackbox]")
        await pg.click("#bbDownload")
        await pg.wait_for_selector("#bbResults:not([hidden])", timeout=30000)
        n = await pg.locator("#bbSuggest .sug").count()
        check(n >= 1, "blackbox analysed: %d suggestions" % n)
        rows = await pg.locator("#bbPlan [data-pl]").count()
        if rows:
            await pg.click("#planTake")
            pend = await pg.evaluate("Object.keys(S.edits).length")
            check(pend >= 1, "tuning plan: %d changes, taken into the pending changes (%d)" % (rows, pend))
            await pg.click("#discardBtn")
        else:
            check(await pg.locator("#bbPlan").count() == 1, "tuning plan rendered (no change needed for this log)")
        await pg.screenshot(path=os.path.join(SHOTS, "blackbox.png"), full_page=True)
        # SD card: status + one file per flight, analysed straight from the card, deleted
        await pg.wait_for_selector("#sdFiles .sdrow", timeout=5000)
        nfiles = await pg.locator("#sdFiles .sdrow").count()
        info = await pg.inner_text("#sdInfo")
        check(nfiles == 2 and "Sẵn sàng" in info and "31.9 GB" in info, "SD card: state, space and 2 logs listed (%s)" % info)
        await pg.evaluate("S.analysis = null; document.getElementById('bbResults').hidden = true")
        await pg.click("#sdFiles button[data-sda='AQ00002.BBL']")
        await pg.wait_for_selector("#bbResults:not([hidden])", timeout=30000)
        check(True, "SD card: a log analysed straight from the card")
        pg.once("dialog", lambda d: asyncio.ensure_future(d.accept()))
        await pg.click("#sdFiles button[data-sdd='AQ00001.BBL']")
        await pg.wait_for_function("document.querySelectorAll('#sdFiles .sdrow').length === 1", timeout=5000)
        check(True, "SD card: log deleted, list refreshed")
        await pg.screenshot(path=os.path.join(SHOTS, "sdcard.png"))
        await pg.click("nav button[data-tab=system]")
        wire = await pg.inner_text("#sdWire")
        opts = await pg.locator("#sdParams select[data-p=sd_mode] option").count()
        check("CLK → GPIO 30" in wire and "D0 → GPIO 32" in wire and opts == 4,
              "SD wiring: mode choice and pin map for SD bus 1-bit (%s)" % wire)
        await pg.select_option("#sdParams select[data-p=sd_mode]", "1")
        wire = await pg.inner_text("#sdWire")
        check("SCK" in wire and "CS → GPIO 35" in wire and await pg.locator("#sdOpt input[data-p=sd_d3]").count() == 1,
              "SD wiring: SPI shows SCK / MOSI / MISO / CS")
        await pg.click("#discardBtn")
        await pg.click("nav button[data-tab=params]")
        await pg.fill("#pSearch", "fence")
        vis = await pg.locator("#paramGroups .prow:visible").count()
        check(0 < vis < 20, "parameter search filters rows (%d)" % vis)
        # a server without WebSocket support: the page must fall back to polling
        srv2 = subprocess.Popen([sys.executable, os.path.join(ROOT, "tools", "web_mock.py"), str(PORT + 1)],
                                env={**os.environ, "MOCK_NO_WS": "1", "MOCK_NO_AUTH": "1"})
        try:
            await asyncio.sleep(1)
            p2 = await b.new_page(viewport={"width": 390, "height": 844})
            await p2.goto("http://127.0.0.1:%d/" % (PORT + 1))
            await p2.wait_for_timeout(5000)
            check(await p2.evaluate("S.polling === true && S.linkOk === true"), "falls back to polling without WebSocket")
        finally:
            srv2.terminate()
        # backup download, restore from an edited file, firmware update
        await pg.click("nav button[data-tab=system]")
        await pg.wait_for_timeout(1500)  # > 10 WebSocket frames: lite frames must keep the slow fields
        tbl = await pg.inner_text("#taskTable")
        check("telemetry" in tbl and "4700 B" in tbl, "task table with stack headroom survives lite status frames")
        check(await pg.evaluate("S.status.full === false || S.status.full === true") and
              await pg.evaluate("!!S.status.fw && !!S.status.tasks"), "merged status keeps firmware info")
        async with pg.expect_download() as dl:
            await pg.click("#backupBtn")
        path = await (await dl.value).path()
        backup = open(path).read()
        check("set roll_p" in backup, "backup file downloaded (CLI text)")
        edited = backup.replace([l for l in backup.splitlines() if l.startswith("set roll_p")][0], "set roll_p 63")
        bpath = os.path.join(SHOTS, "edited-backup.txt")
        open(bpath, "w").write(edited)
        pg.once("dialog", lambda d: asyncio.ensure_future(d.accept()))
        await pg.set_input_files("#restoreFile", bpath)
        await pg.wait_for_function("document.getElementById('restoreMsg').textContent.includes('Đã khôi phục')", timeout=8000)
        await pg.wait_for_timeout(800)
        check(await pg.evaluate("S.params.roll_p.v === 63"), "restore applies the file and reloads the settings")
        fw = os.path.join(SHOTS, "firmware.bin")
        open(fw, "wb").write(bytes([0xE9]) + bytes(200000))
        pg.once("dialog", lambda d: asyncio.ensure_future(d.accept()))
        await pg.set_input_files("#otaFile", fw)
        await pg.wait_for_function("document.getElementById('otaNote').textContent.includes('khởi động lại')", timeout=10000)
        check(await pg.inner_text("#otaPct") == "100%", "firmware upload with progress, then reboot")
        await pg.screenshot(path=os.path.join(SHOTS, "system.png"), full_page=True)
        # change the WiFi / web password: logged out, the new password works, the warning is gone
        await pg.click("nav button[data-tab=system]")
        await pg.fill("#secSsid", "MyQuad")
        await pg.fill("#secWifi", "s3cret-pass")
        await pg.fill("#secCur", "allquad123")
        await pg.click("#secForm button[type=submit]")
        await pg.wait_for_selector("#login:not([hidden])", timeout=8000)
        await pg.fill("#loginPw", "s3cret-pass")
        await pg.click("#loginForm button[type=submit]")
        await pg.wait_for_timeout(1500)
        check(not await pg.is_visible("#login") and not await pg.is_visible("#defPw")
              and await pg.input_value("#secSsid") == "MyQuad", "password change: re-login with the new password")
        check(not errs, "no JavaScript errors %s" % errs)
        await b.close()

srv = subprocess.Popen([sys.executable, os.path.join(ROOT, "tools", "web_mock.py"), str(PORT)])
try:
    time.sleep(1)
    asyncio.run(main())
finally:
    srv.terminate()
print("\n%s" % ("all UI checks passed" if not fails else "%d UI checks failed" % len(fails)))
sys.exit(1 if fails else 0)
