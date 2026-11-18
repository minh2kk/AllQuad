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
    ctx = await b.new_context(viewport=vp)
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
