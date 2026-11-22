#!/usr/bin/env python3
"""Usability layer of the configurator (src/web/ui/ux.js) on the in-browser demo drone:
long tabs as pages, phone navigation sheet, one-step save + reboot reminder, bit-mask tick boxes,
the parameters tab order, the connect screen, and the firmware flasher (target from the file, board check).
Needs build/web/AllQuad-Configurator.html (tools/embed_web.py with the WebAssembly demo) and Playwright.
    python3 test/web/test_ux.py [screenshot_dir]
"""
import asyncio, os, sys
from playwright.async_api import async_playwright

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
PAGE = os.path.join(ROOT, "build", "web", "AllQuad-Configurator.html")
SHOTS = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "build", "web", "shots")
STM_BIN = os.path.join(ROOT, ".pio", "build", "stm32h743_devebox", "firmware.bin")
fails = []


def check(ok, msg):
    print("[%s] %s" % ("PASS" if ok else "FAIL", msg))
    if not ok:
        fails.append(msg)


async def demo(b, vp):
    ctx = await b.new_context(viewport=vp, accept_downloads=True)
    await ctx.add_init_script("try { localStorage.setItem('aq.pages', '1') } catch (e) {}")  # paging on (off for other tests)
    pg = await ctx.new_page()
    errs = []
    pg.on("pageerror", lambda e: errs.append(str(e)))
    pg.on("dialog", lambda d: asyncio.ensure_future(d.accept()))
    await pg.goto("file://" + PAGE)
    return ctx, pg, errs


async def main():
    os.makedirs(SHOTS, exist_ok=True)
    async with async_playwright() as p:
        exe = os.environ.get("CHROMIUM")
        b = await p.chromium.launch(**({"executable_path": exe} if exe else {}))

        # ---------------- desktop
        ctx, pg, errs = await demo(b, {"width": 1366, "height": 860})
        await pg.wait_for_selector("#demoBtn:not([hidden])", timeout=5000)
        check(not await pg.is_visible("#comBaud") and await pg.is_visible(".con-help summary"),
              "connect: baud rate and troubleshooting folded under 'Không kết nối được?'")
        await pg.screenshot(path=os.path.join(SHOTS, "ux-connect.png"))

        # flasher: the file decides ESP32 / STM32, the board name is read from the image
        await pg.click("#connect .flOpen")
        await pg.click("#flTarget [data-t=esp]")
        if os.path.exists(STM_BIN):
            await pg.set_input_files("#flFile", STM_BIN)
            await pg.wait_for_timeout(300)
            info = await pg.inner_text("#flInfo")
            tgt = await pg.evaluate("FL.target")
            check(tgt == "stm" and "Board: STM32H743-DevEBox" in info and await pg.is_enabled("#flGo"),
                  "flasher: STM32 file switches the loader to DFU and names its board (%s)" % info)
            mm = await pg.evaluate("""(() => { const o = LINK.open, h = S.lastHello; LINK.open = true;
                S.lastHello = { board: 'STM32H743-WeAct (480 MHz, rev V, crystal 25 MHz)' }; flCheck();
                const r = [FL.mismatch, $('flInfo').textContent, $('flInfo').classList.contains('bad')];
                LINK.open = o; S.lastHello = h; flCheck(); return r; })()""")
            check(mm[0] == "STM32H743-WeAct" and "Cảnh báo" in mm[1] and mm[2], "flasher: file for another board than the connected FC is flagged")
            await pg.screenshot(path=os.path.join(SHOTS, "ux-flasher.png"))
        else:
            print("(no .pio/build/stm32h743_devebox/firmware.bin: flasher file checks skipped)")
        names = await pg.evaluate("""(() => { const s = (t) => new TextEncoder().encode('x'.repeat(2000) + t + '\\0' + 'y'.repeat(100));
            return [uxBoardOf(s('ESP32-S3-DevKitC')), uxBoardOf(s('ESP32-DevKit')), uxBoardOf(s('STM32F405')), uxBoardOf(s('nothing'))]; })()""")
        check(names == ["ESP32-S3-DevKitC", "ESP32-DevKit", "STM32F405", ""], "flasher: board names told apart (%s)" % names)
        await pg.click("#flClose")

        await pg.click("#demoBtn")
        await pg.wait_for_function("Object.keys(S.params).length > 200", timeout=20000)
        await pg.wait_for_timeout(1500)

        # pages
        await pg.click("nav button[data-tab=pid]")
        await pg.wait_for_timeout(300)
        pills = await pg.evaluate("[...document.querySelectorAll('#tab-pid > .jump a')].map(a => a.textContent)")
        check(pills == ["PID", "Cảm giác bay", "Rates", "Bộ lọc", "Tất cả"], "PID tab: page pills %s" % pills)
        check(await pg.is_visible("#pidMatrix") and not await pg.is_visible("#filterParams"), "PID tab: one page at a time (filters hidden on the PID page)")
        await pg.click("#tab-pid > .jump a:text-is('Bộ lọc')")
        await pg.wait_for_timeout(200)
        check(await pg.is_visible("#filterParams") and not await pg.is_visible("#pidMatrix"), "PID tab: 'Bộ lọc' page shows the filters only")
        await pg.screenshot(path=os.path.join(SHOTS, "ux-pages.png"))
        await pg.click("nav button[data-tab=modes]")
        await pg.click("nav button[data-tab=pid]")
        await pg.wait_for_timeout(200)
        check(await pg.is_visible("#filterParams"), "PID tab: the page is remembered")
        await pg.evaluate("document.querySelector('#tab-pid > .jump a').click(); $('filterParams').scrollIntoView()")
        await pg.wait_for_timeout(200)
        check(await pg.is_visible("#filterParams"), "scrolling to an element (search, links) opens its page")
        await pg.click("#tab-pid > .jump a:text-is('Tất cả')")
        check(await pg.is_visible("#filterParams") and await pg.is_visible("#pidMatrix"), "PID tab: 'Tất cả' shows the whole tab")

        # bit masks
        check(await pg.locator("#tab-pid [data-pbits=at_steps]").count() == 4 and await pg.locator("#tab-pid [data-pbits=at_axes]").count() == 3,
              "autotune steps / axes as tick boxes")
        await pg.evaluate("document.querySelector('[data-pbits=at_steps][data-bit=\"8\"]').scrollIntoView()")
        await pg.click("[data-pbits=at_steps][data-bit='8']")
        check(await pg.evaluate("S.edits.at_steps") == 7, "unticking 'Bộ lọc' stages at_steps 15 -> 7")
        check(await pg.evaluate("[...document.querySelectorAll('select[data-p=prof_ch] option')].map(o => o.textContent).slice(0, 2)") == ["Không dùng", "AUX1 (CH5)"],
              "profile channel is a list of AUX channels")

        # one-step save, reboot reminder
        await pg.evaluate("setEdit('loop_hz', 2000)")
        await pg.wait_for_timeout(100)
        check(await pg.is_visible("#applySaveBtn") and await pg.is_visible("#applyBtn"), "apply bar: Lưu (apply + flash) next to 'Chỉ áp dụng'")
        check(await pg.evaluate("document.body.classList.contains('barshow')"), "apply bar shown: page gets room at the bottom")
        await pg.click("#applySaveBtn")
        await pg.wait_for_timeout(800)
        txt = await pg.inner_text("#applyText")
        check(not await pg.evaluate("Object.keys(S.edits).length") and await pg.is_visible("#rebootNowBtn") and "khởi động lại" in txt,
              "saved; a setting that needs a reboot leaves a reminder: " + txt)
        await pg.screenshot(path=os.path.join(SHOTS, "ux-reboot.png"))
        await pg.click("#rebootLaterBtn")
        check(not await pg.is_visible("#applyBar"), "reboot reminder dismissed")
        await pg.evaluate("setEdit('roll_p', val('roll_p') + 1)")
        await pg.keyboard.press("Control+s")
        await pg.wait_for_timeout(800)
        check(await pg.evaluate("!Object.keys(S.edits).length && !PRO.unsaved") and not await pg.is_visible("#applyBar"), "Ctrl+S applies and saves")

        # parameters tab: search first, presets / profiles / history on their own pages
        await pg.click("nav button[data-tab=params]")
        await pg.wait_for_timeout(300)
        pp = await pg.evaluate("[...document.querySelectorAll('#tab-params > .jump a')].map(a => a.textContent)")
        check(pp[:3] == ["Tất cả tham số", "Preset & hồ sơ", "Lịch sử & so sánh"] and await pg.is_visible("#pSearch")
              and not await pg.is_visible("#presetList"), "parameters: search is the first thing on the tab %s" % pp)
        check(await pg.evaluate("$('profPanel').parentElement.id === 'profHost' && $('cmpPanel').parentElement.id === 'paramTools' && $('histList').closest('#paramTools') !== null"),
              "parameters: profiles with presets, history and compare together")
        # pin map: Ports tab page, filters, search, a pin used twice, links from other tabs
        await pg.click("nav button[data-tab=ports]")
        await pg.wait_for_timeout(300)
        await pg.click("#tab-ports > .jump a:text-is('Sơ đồ chân')")
        await pg.wait_for_timeout(200)
        await pg.evaluate("setEdit('serial1_fn', 1); setEdit('serial2_fn', 2)")  # the demo drone starts with no port functions
        await pg.wait_for_timeout(100)
        rows = await pg.evaluate("[...document.querySelectorAll('#pinTable tr.pin-r')].map(r => r.textContent.replace(/\\s+/g, ' '))")
        check(await pg.is_visible("#pinTable") and any("PD9" in r and "SERIAL1 RX" in r and "Receiver" in r for r in rows)
              and any("PA0" in r and "motor 1" in r for r in rows), "pin map: pins with what they are wired to and what uses them (%d rows)" % len(rows))
        await pg.click("#pinSeg [data-f=motor]")
        n = await pg.evaluate("document.querySelectorAll('#pinTable tr.pin-r').length")
        check(n == 4, "pin map: 'Motor' filter shows the 4 motor outputs (%d)" % n)
        await pg.click("#pinSeg [data-f=all]")
        await pg.fill("#pinQ", "gps")
        rows = await pg.evaluate("[...document.querySelectorAll('#pinTable tr.pin-r')].map(r => r.dataset.pin)")
        check(rows == ["PE7", "PE8"], "pin map: search 'gps' finds the GPS port pins %s" % rows)
        await pg.fill("#pinQ", "")
        await pg.evaluate("setEdit('serial4_fn', 2)")
        await pg.wait_for_timeout(100)
        use = await pg.evaluate("document.querySelector('#pinTable tr[data-pin=PA10]').innerText")
        check("GPS" in use, "pin map follows pending edits (SERIAL4 -> GPS: %s)" % use.replace("\n", " "))
        await pg.evaluate("S.pinMap.pins.push(['PD9', 'led', 0, 'LED']); buildPinMap()")
        clash = await pg.inner_text("#pinClash")
        check(await pg.is_visible("#pinClash") and "PD9" in clash, "pin map: a pin used twice is flagged (%s)" % clash)
        await pg.evaluate("S.pinMap.pins.pop(); $('discardBtn').click(); buildPinMap()")
        await pg.screenshot(path=os.path.join(SHOTS, "ux-pins.png"))
        await pg.click("nav button[data-tab=motors]")
        await pg.wait_for_timeout(200)
        await pg.click("#tab-motors .pin-link")
        await pg.wait_for_timeout(300)
        check(await pg.is_visible("#pinTable") and await pg.evaluate("S.pinFilter") == "motor", "motor tab: 'Xem chân motor' opens the pin map on the motors")

        # flow / range module sending the wrong protocol: the Ports tab traffic table says what to do
        await pg.click("nav button[data-tab=ports]")
        await pg.wait_for_timeout(200)
        await pg.click("#tab-ports > .jump a:text-is('Tất cả')")
        note = await pg.evaluate("""(() => { PS.last = null; PS.base = null; PS.ref = null;
            portStatsRender({ ports: [[0, 900, 0, 0, 0, 0, 0]], snsl: [0, 900, 0, 0, 14] }); return $('psNote').textContent; })()""")
        check("MAVLink" in note and "MicoAssistant" in note, "Ports tab: a flow module sending MAVLink is explained (%s)" % note[:80])
        await pg.evaluate("PS.last = null; PS.base = null; PS.ref = null; S.status && portStatsRender(S.status)")

        # readiness: being connected is information, not a red blocker
        await pg.click("nav button[data-tab=overview]")
        await pg.wait_for_timeout(600)
        rd = await pg.evaluate("[$('readyTitle').textContent, document.querySelectorAll('#blockerChips .blk.info').length, $('blockers').className]")
        check(rd[0] == "Sẵn sàng arm" and rd[1] == 1 and "ready" in rd[2], "overview: the configurator link shows as information, not a blocker %s" % rd)
        todo = await pg.evaluate("[...document.querySelectorAll('#todoPanel .it span')].map(e => e.textContent)")
        check("Dung lượng pin" in todo and "Bật blackbox" in todo, "overview: 'Cấu hình còn thiếu' lists what is not set up %s" % todo)
        await pg.evaluate("setEdit('acc_calibrated', 0)")
        await pg.wait_for_timeout(700)
        need = await pg.evaluate("[$('todoDet').open, [...document.querySelectorAll('#todoPanel .it.need span')].map(e => e.textContent)]")
        check(need[0] and "Hiệu chuẩn accel" in need[1], "overview: a missing accel calibration is a must-do item, card open %s" % need)
        await pg.evaluate("$('discardBtn').click()")
        alt = await pg.evaluate("document.querySelector('#tl-pos .v').textContent")
        check(not alt.startswith("-0.0"), "overview: no '-0.0 m' (%s)" % alt)

        # receiver tab: channels, travel check, channel order wizard
        await pg.click("nav button[data-tab=rx]")
        await pg.wait_for_timeout(800)
        n = await pg.evaluate("document.querySelectorAll('#rxChs .rxch').length")
        check(n == 8, "receiver tab: 4 sticks + 4 AUX shown (%d)" % n)
        fake = """((rc) => { const st = Object.assign({}, S.status, { rc: rc.concat(Array(16 - rc.length).fill(1000)), rxok: true }); (window.__ts || toolsStatus)(st); })"""
        await pg.evaluate(fake + "([1000, 1500, 1000, 1500, 1000, 1000, 1000, 1000])")
        await pg.evaluate(fake + "([2000, 1500, 2000, 1500, 1000, 1000, 1000, 1000])")
        await pg.evaluate(fake + "([1500, 1100, 1000, 1500, 1000, 1000, 1000, 1000])")
        trav = await pg.evaluate("$('rxTravel').textContent")
        check("chỉnh endpoint" in trav or "tốt" in trav, "receiver tab: travel check comments on each stick")
        # wizard on a TAER receiver: throttle comes on raw channel 1, roll 2, pitch 3, yaw 4
        await pg.evaluate("window.__ts = toolsStatus; toolsStatus = () => {}")  # the demo drone's own frames paused
        await pg.evaluate("$('rxWizBtn').click()")
        steps = [[1000, 1500, 1500, 1500], [2000, 1500, 1500, 1500],  # GA: raw ch1
                 [1000, 1500, 1500, 1500], [1000, 2000, 1500, 1500],  # ROLL: raw ch2
                 [1000, 1500, 1500, 1500], [1000, 1500, 2000, 1500],  # PITCH: raw ch3
                 [1000, 1500, 1500, 1500], [1000, 1500, 1500, 2000]]  # YAW: raw ch4
        for k in range(0, 8, 2):
            await pg.evaluate("RX.wiz && (RX.wiz.waitUntil = 0)")
            await pg.evaluate(fake + "(%s)" % steps[k])
            await pg.evaluate(fake + "(%s)" % steps[k + 1])
            await pg.wait_for_timeout(650)
            await pg.evaluate(fake + "(%s)" % steps[k + 1])
        wiz = await pg.evaluate("$('rxWiz').textContent")
        m = await pg.evaluate("[val('rc_ch_roll'), val('rc_ch_pitch'), val('rc_ch_thr'), val('rc_ch_yaw')]")
        check(m == [2, 3, 1, 4] and "Xong" in wiz, "receiver tab: 'move each stick' finds a TAER order %s (%s)" % (m, wiz[:60]))
        await pg.screenshot(path=os.path.join(SHOTS, "ux-rx.png"))
        await pg.evaluate("$('discardBtn').click(); RX.wiz = null; rxWizShow()")

        # modes: pick a switch channel by flipping it
        await pg.click("nav button[data-tab=modes]")
        await pg.wait_for_timeout(300)
        await pg.evaluate("document.querySelector('#modeList .mrow .mr-learn').click()")
        await pg.evaluate("S.chLearn.base = [1500, 1500, 1000, 1500].concat(Array(12).fill(1000))")
        await pg.evaluate(fake + "([1500, 1500, 1000, 1500, 1000, 1000, 2000, 1000])")
        i = await pg.evaluate("+document.querySelector('#modeList .mrow').dataset.i")
        ch = await pg.evaluate("val('mr%d_ch')" % i)
        check(ch == 6, "modes: 'Dò' picks the switch that was flipped (AUX3 = channel index 6, got %s)" % ch)
        await pg.evaluate("$('discardBtn').click(); toolsStatus = window.__ts; window.__ts = null")

        # battery / current sensing helper: divider calculator, Betaflight current scale, calibration
        await pg.click("nav button[data-tab=modes]")
        await pg.wait_for_timeout(500)
        ok = await pg.evaluate("""(() => { BAT.how = 'div'; buildBatHelper();
            $('batR1').value = '10'; $('batR2').value = '1'; $('batCells').value = '6'; $('batR1').oninput();
            const a = $('batDivOut').textContent;
            $('batR1').value = '3'; $('batR1').oninput();
            const b = $('batDivOut').textContent;
            $('batCurT').value = 'bf'; $('batCurV').value = '400'; $('batCurO').value = '0'; $('batCurV').oninput();
            $('batCurOut').querySelector('button').click();
            return [a.includes('11.00') && a.includes('✓'), b.includes('quá'), val('cur_scale')]; })()""")
        check(ok[0] and ok[1] and abs(ok[2] - 25) < 1e-6, "battery helper: 10k/1k = 11 fits 6S, 3k/1k flagged, Betaflight 400 -> 25 A/V %s" % ok)
        cal = await pg.evaluate("""(() => { const pin = S.status.bat.pin; $('batMeter').value = (pin * 11.2).toFixed(3); $('batMeterGo').click();
            BAT.cbuf = [0.10, 0.11, 0.12]; $('batZero').click();
            return [Math.abs(val('vbat_scale') - 11.2) < 0.01, Math.abs(val('cur_offset') - 0.11) < 1e-6]; })()""")
        check(cal == [True, True], "battery helper: multimeter calibration sets vbat_scale, zero sets cur_offset %s" % cal)
        await pg.evaluate("$('discardBtn').click()")

        # diagnostic bundle
        await pg.click("nav button[data-tab=system]")
        await pg.wait_for_timeout(300)
        txt = await pg.evaluate("diagBundle()")
        check(all(h in txt for h in ["## selftest", "## diff", "## pins", "## status", "Board:"]) and "allquad123" not in txt and '"ssid"' not in txt,
              "system: diagnostic bundle has selftest / diff / pins / status, no password or WiFi name (%d bytes)" % len(txt))
        async with pg.expect_download() as dl:
            await pg.evaluate("$('diagSave').click()")
        d = await dl.value
        check(d.suggested_filename.startswith("allquad-chan-doan-") and d.suggested_filename.endswith(".txt"), "system: bundle saved as %s" % d.suggested_filename)

        check(not errs, "no page errors %s" % errs[:3])
        await ctx.close()

        # ---------------- phone
        ctx, pg, errs = await demo(b, {"width": 390, "height": 844})
        await pg.click("#demoBtn")
        await pg.wait_for_function("Object.keys(S.params).length > 200", timeout=20000)
        await pg.wait_for_timeout(1000)
        vis = await pg.evaluate("[...document.querySelectorAll('nav.tabs button')].filter(b => b.offsetParent).map(b => b.dataset.tab || b.id)")
        check(vis == ["overview", "setup", "pid", "modes", "navMore"], "phone: bottom bar keeps 4 main tabs + Thêm %s" % vis)
        await pg.click("#navMore")
        await pg.wait_for_timeout(200)
        await pg.screenshot(path=os.path.join(SHOTS, "ux-phone-sheet.png"))
        n = await pg.evaluate("[...document.querySelectorAll('nav.tabs button[data-tab]')].filter(b => b.offsetParent).length")
        check(n >= 14, "phone: 'Thêm' opens a sheet with every tab (%d)" % n)
        await pg.click("nav button[data-tab=system]")
        await pg.wait_for_timeout(300)
        vis = await pg.evaluate("[...document.querySelectorAll('nav.tabs button')].filter(b => b.offsetParent).map(b => b.dataset.tab || b.id)")
        check(not await pg.evaluate("document.body.classList.contains('navsheet')") and await pg.is_visible("#tab-system") and "system" in vis,
              "phone: picking a tab closes the sheet, the open tab shows in the bar %s" % vis)
        await pg.screenshot(path=os.path.join(SHOTS, "ux-phone-system.png"))
        check(not errs, "phone: no page errors %s" % errs[:3])
        await ctx.close()
        await b.close()
    print("\n" + ("all ux checks passed" if not fails else "%d FAILED" % len(fails)))
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    asyncio.run(main())
