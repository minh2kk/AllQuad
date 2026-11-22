#!/usr/bin/env python3
"""Guidance layer of the configurator (src/web/ui/guide.js) on the in-browser demo drone:
first-flight guide (steps, live tilt test, live failsafe test), arm-blocker deep links, RPM filter state,
optical-flow alignment fit, flight summary of a blackbox log, glossary, "raise / lower" lines, simple-view tabs.
Needs build/web/AllQuad-Configurator.html (tools/embed_web.py with the WebAssembly demo) and Playwright.
    python3 test/web/test_guide.py [screenshot_dir]
"""
import asyncio, os, sys
from playwright.async_api import async_playwright

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
PAGE = os.path.join(ROOT, "build", "web", "AllQuad-Configurator.html")
SHOTS = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "build", "web", "shots")
LOG = os.path.join(ROOT, "build", "web", "normal.bin")
fails = []


def check(ok, msg):
    print("[%s] %s" % ("PASS" if ok else "FAIL", msg))
    if not ok:
        fails.append(msg)


# status frames fed to the guide by hand (the demo drone's own frames are paused meanwhile)
FEED = """((o) => { const st = Object.assign({}, S.status, o); ffLiveTick(st); ffRender(st); return GD.live ? [GD.live.phase, GD.live.fail || ''] : null; })"""


async def main():
    os.makedirs(SHOTS, exist_ok=True)
    async with async_playwright() as p:
        exe = os.environ.get("CHROMIUM")
        b = await p.chromium.launch(**({"executable_path": exe} if exe else {}))
        ctx = await b.new_context(viewport={"width": 1366, "height": 900})
        pg = await ctx.new_page()
        errs = []
        pg.on("pageerror", lambda e: errs.append(str(e)))
        pg.on("dialog", lambda d: asyncio.ensure_future(d.accept()))
        await pg.goto("file://" + PAGE)
        await pg.wait_for_selector("#demoBtn:not([hidden])", timeout=5000)
        await pg.click("#demoBtn")
        await pg.wait_for_function("Object.keys(S.params).length > 200 && S.status", timeout=20000)
        await pg.wait_for_timeout(1500)

        # ---- first-flight guide
        await pg.click("nav button[data-tab=preflight]")
        await pg.wait_for_timeout(400)
        n = await pg.locator("#ffPanel ol.ff > li").count()
        check(n == 9, "guide: 9 steps in the pre-flight tab (%d)" % n)
        first = await pg.inner_text("#ffPanel li.ff-now b")
        check(first.startswith("1."), "guide: first open step highlighted with its reason (%s)" % first)
        await pg.click("#ffPanel [data-ff-done=wire]")
        await pg.wait_for_timeout(200)
        check(await pg.evaluate("!!ffMarks().wire") and "ok" in await pg.get_attribute("#ffPanel li:nth-child(1)", "class"),
              "guide: 'Đã xong' ticks a manual step (kept in the browser)")
        todo = await pg.evaluate("(() => { setupTodo(S.status); return $('todoPanel').textContent; })()")
        check("Hướng dẫn bay lần đầu" in todo, "overview to-do card suggests the guide until it is finished")

        await pg.evaluate("window.__ts = toolsStatus; toolsStatus = () => {}")
        # tilt test: flat -> right -> nose up passes
        await pg.evaluate("ffLive('orient')")
        r = None
        for att in ([0, 0, 0],) * 3 + ([30, 2, 0],) * 3 + ([0, 0, 0], [1, 30, 0], [1, 30, 0], [1, 30, 0]):
            r = await pg.evaluate(FEED, {"att": att})
            await pg.wait_for_timeout(350)
        check(r is None and await pg.evaluate("!!ffMarks().orient"), "tilt test: right roll and nose-up seen with the right signs -> step done")
        # a board turned 90 degrees: tilting right shows up as pitch
        await pg.evaluate("ffLive('orient')")
        for att in ([0, 0, 0], [0, 0, 0], [0, 0, 0], [2, 28, 0]):
            r = await pg.evaluate(FEED, {"att": att})
            await pg.wait_for_timeout(350)
        check(r and "lệch 90°" in r[1] and "imu_align" in r[1], "tilt test: board turned 90° is named, with imu_align (%s)" % (r and r[1][:50]))
        await pg.screenshot(path=os.path.join(SHOTS, "guide-tilt-fail.png"))
        await pg.evaluate("ffLiveStop()")
        # failsafe: radio off is noticed, radio back on passes
        await pg.evaluate("ffLive('fs')")
        r1 = await pg.evaluate(FEED, {"rxok": True, "failsafe": False})
        r2 = await pg.evaluate(FEED, {"rxok": False})
        r3 = await pg.evaluate(FEED, {"rxok": True, "failsafe": False, "blockers": []})
        check(r1 == [0, ""] and r2 == [1, ""] and r3 is None and await pg.evaluate("!!ffMarks().fs"),
              "failsafe test: waits for the radio to go off, then back on -> step done")
        # a receiver that keeps sending channels with the radio off ("hold"): named after 20 s
        await pg.evaluate("ffLive('fs'); GD.live.t0 -= 21000")
        r = await pg.evaluate(FEED, {"rxok": True, "failsafe": False})
        check(r and "No pulses" in r[1], "failsafe test: receiver still sending after 20 s -> told to set 'No pulses'")
        await pg.evaluate("ffLiveStop(); toolsStatus = window.__ts")
        await pg.screenshot(path=os.path.join(SHOTS, "guide-preflight.png"), full_page=True)

        # ---- arm blockers: deep links
        await pg.evaluate("gotoTab('modes:arm_max_angle')")
        await pg.wait_for_timeout(500)
        check(await pg.is_visible("#tab-modes") and await pg.evaluate("document.querySelector('#tab-modes .prow[data-name=arm_max_angle]').classList.contains('flash')"),
              "gotoTab('modes:arm_max_angle') opens the tab and highlights the row")
        await pg.evaluate("gotoTab('modes#batHelp')")
        await pg.wait_for_timeout(500)
        check(await pg.evaluate("$('batHelp') && $('batHelp').classList.contains('flash')"), "gotoTab('modes#batHelp') scrolls to the battery helper")
        blk = await pg.evaluate("BLOCK_HELP.NO_RX[2] + ' ' + BLOCK_HELP.ANGLE[2]")
        check(blk == "rx modes:arm_max_angle", "arm blockers point at the receiver tab / the limit itself (%s)" % blk)

        # ---- RPM filter state
        states = await pg.evaluate("""(() => { const e = Object.assign({}, S.edits), out = [];
            const run = (h, bd, er) => { S.edits.rpm_harmonics = h; S.edits.dshot_bidir = bd; S.edits.esc_telemetry = 0; S.edits.motor_protocol = 3; out.push(rpmState({ erpm: er })[0]); };
            run(0, 1, null); run(3, 0, null); run(3, 1, [9000, 9100, 9050, 9000]); run(3, 1, [9000, -1, 9050, 9000]); run(3, 1, null);
            S.edits = e; return out; })()""")
        check(states == ["off", "warn", "ok", "warn", "info"], "RPM filter state: off / no source / running / one ESC missing / waiting %s" % states)
        await pg.click("nav button[data-tab=pid]")
        await pg.wait_for_timeout(500)
        check(await pg.evaluate("!!$('filterParamsRpm') && /RPM filter/.test($('filterParamsRpm').textContent)"), "PID & lọc: RPM filter state shown above the filter settings")
        fx = await pg.evaluate("[...document.querySelectorAll('#tab-pid .prow[data-name=gyro_lpf_hz] .fx span')].map(s => s.textContent)")
        check(len(fx) == 2 and fx[0].startswith("↑") and fx[1].startswith("↓"), "gyro_lpf_hz: what raising / lowering does (%s)" % fx)

        # ---- optical flow fit: sensor mounted at every alignment -> the fit finds it back
        ok = await pg.evaluate("""(() => {
            const bad = [];
            for (let align = 0; align < 4; align++) for (let flip = 0; flip < 4; flip++) {
              const s = [];
              for (let k = 0; k < 120; k++) {
                const p = Math.sin(k / 7) * (k % 40 < 20 ? 1.5 : 0), q = Math.cos(k / 5) * (k % 40 >= 20 ? 1.5 : 0);
                let ix = -q, iy = p;                        // what the estimator expects after alignment
                if (flip & 2) iy = -iy;                     // undo flip, then rotate back (inverse of the firmware)
                if (flip & 1) ix = -ix;
                for (let i = 0; i < align; i++) { const t = iy; iy = -ix; ix = t; }
                s.push([ix * 0.9, iy * 0.9, p, q]);
              }
              const b = flowFit(s);
              if (flowXf(b.align, b.flip) !== flowXf(align, flip) || Math.abs(b.kx - 0.9) > 0.05 || Math.abs(b.ky - 0.9) > 0.05) bad.push([align, flip, b.align, b.flip]);
            }
            return bad; })()""")
        check(ok == [], "optical flow: all 16 alignments found back (same transform) from hand-rolled samples, scale 0.9 measured %s" % ok)
        await pg.click("nav button[data-tab=sensors]")
        await pg.wait_for_timeout(800)
        vis, txt = await pg.evaluate("(() => { flowAdd({ gyro: [0, 0, 0], flow: [0, 0], flowq: -1, range: 0.82 }); return [!$('flowPanel').hidden, $('flowRes').textContent]; })()")
        check(vis and "chỉ đo khoảng cách" in txt, "sensors: rangefinder-only module explained in the flow panel")

        # ---- flight summary
        if os.path.exists(LOG):
            await pg.click("nav button[data-tab=blackbox]")
            await pg.set_input_files("#bbFile", LOG)
            await pg.wait_for_selector("#bbReport", timeout=20000)
            tiles = await pg.locator("#bbReport .rep-tiles > div").count()
            notes = await pg.locator("#bbReport .rep-n").count()
            t = await pg.inner_text("#bbReport")
            check(tiles >= 6 and notes >= 1 and "Thời gian bay" in t and "mAh" in t, "blackbox: flight summary with %d tiles and %d notes" % (tiles, notes))
            await pg.locator("#bbReport").screenshot(path=os.path.join(SHOTS, "guide-report.png"))
            # high-rate gyro: none in this log -> how to turn it on; a capture -> spectrum to 2 kHz with the peak named
            t0 = await pg.inner_text("#hfPanel")
            check("bb_gyro_s" in t0, "blackbox: no high-rate capture in the log -> explains bb_gyro_s")
            pk = await pg.evaluate("""(() => { const n = 8000, r = 4000, mk = () => [0, 1, 2].map(() => new Float32Array(n)), raw = mk(), filt = mk();
                for (let k = 0; k < n; k++) { raw[0][k] = 30 * Math.sin(2 * Math.PI * 520 * k / r) + Math.random(); filt[0][k] = 2 * Math.sin(2 * Math.PI * 520 * k / r); }
                S.flight.hf = [{ rate: r, t0: 3, n, duration: 2, raw, filt }]; S.specAxis = 0; drawBlackbox(); return $('hfPanel').textContent; })()""")
            check("520 Hz" in pk and "4000 Hz" in pk and "dB" in pk, "blackbox: high-rate capture shows the 520 Hz motor line and the filter attenuation")
            await pg.locator("#hfPanel").screenshot(path=os.path.join(SHOTS, "guide-hf.png"))
        else:
            print("(no build/web/normal.bin: flight summary skipped)")

        # ---- glossary
        await pg.click("nav button[data-tab=pid]")
        await pg.wait_for_timeout(700)
        q = await pg.locator("#tab-pid h2 .gl-q").count()
        check(q >= 2, "glossary: '?' next to PID-tab headings with a known term (%d)" % q)
        h = await pg.evaluate("document.querySelector('#tab-pid h2').textContent")
        check(h == "PID", "glossary button adds no text to the heading (%r)" % h)
        await pg.evaluate("document.querySelector('#tab-pid h2 .gl-q').click()")
        await pg.wait_for_timeout(300)
        check(await pg.is_visible("#glossDlg") and await pg.evaluate("!!document.querySelector('#glList dt.on')"), "glossary: dialog opens on the term")
        await pg.fill("#glIn", "notch")
        n = await pg.locator("#glList dt").count()
        check(2 <= n <= 5, "glossary: search narrows the list (%d terms for 'notch')" % n)
        await pg.screenshot(path=os.path.join(SHOTS, "guide-glossary.png"))
        await pg.click("#glossDlg .pro-x")
        await pg.keyboard.press("Control+k")
        await pg.fill("#gsIn", "thuat ngu")
        await pg.wait_for_timeout(200)
        await pg.click("#gsRes .gs-it")
        await pg.wait_for_timeout(200)
        check(await pg.is_visible("#glossDlg"), "search: 'thuật ngữ' opens the glossary")
        await pg.click("#glossDlg .pro-x")

        # ---- simple view: Tham số / CLI folded under "Công cụ"
        await pg.click("#viewBtn")
        await pg.wait_for_timeout(200)
        hid = await pg.evaluate("[...document.querySelectorAll('nav.tabs button[data-tab]')].filter(b => !b.offsetParent).map(b => b.dataset.tab)")
        check(hid == ["params", "cli"], "simple view: expert tabs folded %s" % hid)
        await pg.click("nav.tabs .tgroup.tg-x")
        await pg.wait_for_timeout(200)
        hid2 = await pg.evaluate("[...document.querySelectorAll('nav.tabs button[data-tab]')].filter(b => !b.offsetParent).length")
        check(hid2 == 0, "simple view: the 'Công cụ' label unfolds them")
        await pg.click("#viewBtn")

        # ---- English view: header switch, nav / headings / hints translated, numbers kept, back to Vietnamese
        await pg.click("#langBtn")
        await pg.evaluate("gotoTab('overview')")
        await pg.wait_for_timeout(600)
        nav = await pg.evaluate("[...document.querySelectorAll('nav.tabs button[data-tab]')].slice(0, 4).map(b => b.textContent.trim())")
        check(nav == ["Overview", "Setup", "Pre-flight", "PID & filters"], "English: tab names %s" % nav)
        tot = [0, 0]
        for t in ("overview", "pid", "modes", "sensors"):
            await pg.evaluate("t => gotoTab(t)", t)
            await pg.wait_for_timeout(500)
            r = await pg.evaluate("""(() => { const VI = /[àáảãạăằắẳẵặâầấẩẫậèéẻẽẹêềếểễệìíỉĩịòóỏõọôồốổỗộơờớởỡợùúủũụưừứửữựỳýỷỹỵđ]/i; let a = 0, v = 0;
                const w = document.createTreeWalker(document.querySelector('main > section:not([hidden])'), NodeFilter.SHOW_TEXT); let n;
                while ((n = w.nextNode())) { const x = n.nodeValue.trim(); if (!x || !n.parentElement.offsetParent) continue; a += x.length; if (VI.test(x)) v += x.length; }
                return [a, v]; })()""")
            tot[0] += r[0]; tot[1] += r[1]
        left = tot[1] / max(1, tot[0])
        check(left < 0.02, "English: %.1f%% of the visible text on 4 tabs still Vietnamese" % (left * 100))
        sat = await pg.evaluate("i18nText('14 vệ tinh, sai số 1.2 m')")
        check(sat == "14 satellites, error 1.2 m", "English: numbers carried into the translation (%s)" % sat)
        await pg.click("#langBtn")
        await pg.wait_for_timeout(200)
        vi = await pg.evaluate("document.querySelector('nav.tabs button[data-tab=pid]').textContent.trim()")
        check(vi == "PID & lọc" and await pg.evaluate("localStorage.getItem('aq-lang')") == "vi", "English off: Vietnamese text restored")
        check(not errs, "no page errors %s" % errs[:3])
        await ctx.close()
        await b.close()
    print("\n" + ("all guide checks passed" if not fails else "%d FAILED" % len(fails)))
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    asyncio.run(main())
