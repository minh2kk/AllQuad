#!/usr/bin/env python3
"""Configurator extras end to end on the in-browser demo (firmware compiled to WebAssembly):
setup wizard, parameter history + undo, VTX table, EKF panel, OpenStreetMap tiles (served by a
route, no Internet), blackbox compare + CSV export, warning chips.
Needs build/web/AllQuad-Configurator.html with the demo (tools/build_wasm.sh, tools/embed_web.py),
build/web/normal.bin (node test/web/test_analysis.js) and Playwright + Chromium.
    python3 test/web/test_extras.py [screenshot_dir]
"""
import asyncio, io, zipfile, base64, os, sys, zlib, struct
from playwright.async_api import async_playwright

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SHOTS = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "build", "web", "shots")
PAGE = os.path.join(ROOT, "build", "web", "AllQuad-Configurator.html")
LOG = os.path.join(ROOT, "build", "web", "normal.bin")
fails = []


def check(ok, msg):
    print("[%s] %s" % ("PASS" if ok else "FAIL", msg))
    if not ok:
        fails.append(msg)


def png(color):  # 256x256 single colour tile
    raw = b"".join(b"\x00" + bytes(color) * 256 for _ in range(256))
    chunk = lambda t, d: struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)
    return b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", 256, 256, 8, 2, 0, 0, 0)) + chunk(b"IDAT", zlib.compress(raw)) + chunk(b"IEND", b"")


async def tab(pg, name):
    await pg.click("nav button[data-tab=%s]" % name)
    await pg.wait_for_timeout(300)


async def main():
    os.makedirs(SHOTS, exist_ok=True)
    async with async_playwright() as p:
        exe = os.environ.get("CHROMIUM")
        b = await p.chromium.launch(**({"executable_path": exe} if exe else {}))
        ctx = await b.new_context(viewport={"width": 1280, "height": 900}, accept_downloads=True)
        tiles = []
        tile = png((90, 160, 90))

        async def on_tile(route):
            tiles.append(route.request.url)
            await route.fulfill(status=200, content_type="image/png", body=tile, headers={"Access-Control-Allow-Origin": "*"})
        await ctx.route("https://tile.openstreetmap.org/**", on_tile)
        pg = await ctx.new_page()
        errs = []
        pg.on("pageerror", lambda e: errs.append(str(e)))
        pg.on("dialog", lambda d: asyncio.ensure_future(d.accept()))
        await pg.goto("file://" + PAGE)

        # ---- demo: the firmware runs in a Web Worker
        await pg.wait_for_selector("#demoBtn:not([hidden])", timeout=5000)
        check(True, "connect screen: demo button shown (wasm embedded)")
        await pg.click("#demoBtn")
        await pg.wait_for_function("Object.keys(S.params).length > 200", timeout=20000)
        await pg.wait_for_timeout(2500)
        sub = await pg.inner_text("#sSub")
        check("Demo" in sub and await pg.evaluate("LINK.kind") == "demo", "demo connected: " + sub)
        await pg.screenshot(path=os.path.join(SHOTS, "extras-demo.png"))

        # ---- new parameters in the UI
        await tab(pg, "pid")
        check(await pg.locator("#filterParams [data-name=rpm_w3]").count() == 1 and
              await pg.locator("#motorParams [data-name=dshot_bidir]").count() == 1 and
              await pg.locator("#pidExtra [data-name=at_steps]").count() == 1 and
              await pg.locator("#pidExtra [data-name=launch_angle]").count() == 1, "PID tab: RPM weights, at_steps, launch_* rows; dshot_bidir lives in Motor & ESC")
        await tab(pg, "modes")
        opts = await pg.evaluate("Array.from(document.querySelector('#modeList select').options).map(o => o.textContent)")
        check("GPS RESCUE" in opts and "LAUNCH" in opts, "modes: GPS RESCUE and LAUNCH boxes")

        # ---- EKF panel
        await tab(pg, "nav")
        await pg.wait_for_function("document.getElementById('ekfBody').textContent.includes('Độ bất định')", timeout=8000)
        body = await pg.inner_text("#ekfBody")
        check("Vị trí GPS" in body and "Bias gyro" in body, "nav: EKF panel with test ratios, std and biases")
        check(await pg.locator("#navParams [data-name=est_type]").count() == 1 and await pg.locator("#navParams [data-name=rth_sanity_s]").count() == 1,
              "nav params: est_type, rth_sanity_s")

        # ---- OpenStreetMap tiles (home given, tiles routed)
        await pg.evaluate("S.mission = {home: true, lat: 21.0285, lon: 105.8542, items: [], rally: [], fence: 0}; drawNav()")
        await pg.click("#mapTiles")
        await pg.wait_for_timeout(1500)
        z = [u.split("/")[3] for u in tiles]
        drawn = await pg.evaluate("""(() => { const c = document.getElementById('navMap'), g = c.getContext('2d');
            const d = g.getImageData(c.width / 4, c.height / 4, 1, 1).data; return [d[0], d[1], d[2]]; })()""")
        check(len(tiles) >= 1 and drawn[1] > drawn[0] + 20, "map tiles: %d requested (zoom %s), drawn under the map %s" % (len(tiles), sorted(set(z)), drawn))
        check(await pg.evaluate("localStorage.getItem('aq-tiles')") == "true", "map tiles: choice remembered")
        await pg.screenshot(path=os.path.join(SHOTS, "extras-tiles.png"))

        # ---- offline map: viewed tiles kept in IndexedDB, ZIP import, coarser zoom as fallback
        px = """(() => { const c = document.getElementById('navMap'), g = c.getContext('2d');
            const d = g.getImageData(c.width / 4, c.height / 4, 1, 1).data; return [d[0], d[1], d[2]]; })()"""
        await pg.wait_for_timeout(500)
        st = await pg.evaluate("tileStats()")
        check(st and st["n"] >= 1, "offline map: viewed tiles stored (%s)" % st)
        offline = {"on": True}

        async def no_net(route):
            await route.abort()
        await ctx.unroute("https://tile.openstreetmap.org/**")
        await ctx.route("https://tile.openstreetmap.org/**", no_net)
        n0 = len(tiles)
        await pg.evaluate("TILES.cache.clear(); drawNav()")
        await pg.wait_for_timeout(800)
        await pg.evaluate("drawNav()")
        drawn = await pg.evaluate(px)
        check(drawn[1] > drawn[0] + 20, "offline map: no Internet, the stored tiles still draw %s" % drawn)
        await pg.click("#mapOffline")
        await pg.wait_for_timeout(300)
        info = await pg.inner_text("#tileInfo")
        check(await pg.is_visible("#tileBox") and "Đã lưu" in info, "offline map: panel shows what is stored (%s)" % info)
        await pg.click("#tileClear")  # confirm() accepted by the dialog handler
        await pg.wait_for_timeout(400)
        check((await pg.evaluate("tileStats()"))["n"] == 0, "offline map: clear empties the store")
        zbuf = io.BytesIO()
        with zipfile.ZipFile(zbuf, "w", zipfile.ZIP_DEFLATED) as zf:
            zf.writestr("Hanoi/3/6/3.png", png((200, 40, 40)))   # zoom 3 only: covers all of northern Vietnam
            zf.writestr("readme.txt", "not a tile")
        zp = os.path.join(SHOTS, "tiles-test.zip")
        open(zp, "wb").write(zbuf.getvalue())
        await pg.set_input_files("#tileZip", zp)
        await pg.wait_for_timeout(1200)
        await pg.evaluate("drawNav()")
        await pg.wait_for_timeout(300)
        await pg.evaluate("drawNav()")
        drawn = await pg.evaluate(px)
        st = await pg.evaluate("tileStats()")
        info = await pg.inner_text("#tileInfo")
        check(st["zip"] == 1 and st["n"] == 1 and "ZIP" in info, "offline map: ZIP import keeps the z/x/y tile, skips other files (%s)" % info)
        check(drawn[0] > drawn[1] + 60, "offline map: missing zoom drawn from the imported zoom-3 tile %s" % drawn)
        check(len(tiles) == n0, "offline map: nothing reached the network while offline")
        await pg.screenshot(path=os.path.join(SHOTS, "extras-tiles-offline.png"))
        await pg.click("#tileClose")

        # ---- VTX table
        await tab(pg, "ports")
        await pg.evaluate("setEdit('vtxt_powers', 3); setEdit('vtxt_mw1', 25); setEdit('vtxt_mw2', 200); setEdit('vtxt_mw3', 800)")
        await pg.wait_for_timeout(300)
        po = await pg.evaluate("Array.from(document.querySelector('[data-p=vtx_power]').options).map(o => o.textContent)")
        rows = await pg.locator("#vtxTable tr[data-lv]:not([hidden])").count()
        check(po == ["25 mW", "200 mW", "800 mW"] and rows == 3, "VTX table: 3 custom power levels %s, %d rows" % (po, rows))
        await pg.select_option("[data-p=vtx_band]", "7")
        await pg.evaluate("setEdit('vtxt_band1', 5333); setEdit('vtx_channel', 1); setEdit('vtx_freq', 0); updateVtx()")
        fq = await pg.inner_text("#vtxFreq")
        check("5333" in fq, "VTX band U: frequency from the table (%s)" % fq.strip())

        # ---- history: apply, record, undo
        await pg.evaluate("localStorage.removeItem('aq-history')")
        await pg.click("#applyBtn")
        await pg.wait_for_timeout(800)
        await tab(pg, "params")
        h = await pg.evaluate("JSON.parse(localStorage.getItem('aq-history'))")
        ok = h and len(h) == 1 and any(c[0] == "vtxt_mw3" and c[2] == 800 for c in h[0]["c"])
        check(ok, "history: applied change recorded with old and new values (%d changes)" % (len(h[0]["c"]) if h else 0))
        await pg.click("#histList [data-h='0']")
        await pg.wait_for_timeout(200)
        check(await pg.evaluate("S.edits.vtxt_powers === 0 && S.edits.vtxt_mw3 !== undefined"), "history: undo puts the old values into the pending edits")
        async with pg.expect_download() as dl:
            await pg.click("#histCsv")
        d = await dl.value
        txt = open(await d.path(), encoding="utf-8").read()
        check(txt.startswith("time,tag,board,param,old,new") and "vtxt_mw3" in txt, "history: CSV export")
        await pg.click("#applyBtn")
        await pg.wait_for_timeout(800)
        h = await pg.evaluate("JSON.parse(localStorage.getItem('aq-history'))")
        check(len(h) == 2 and h[0]["tag"].startswith("Hoàn tác"), "history: the undo itself is recorded (%s)" % h[0]["tag"])
        await pg.screenshot(path=os.path.join(SHOTS, "extras-history.png"))

        # ---- setup wizard
        await tab(pg, "setup")
        steps = await pg.locator("#wzSteps li").count()
        check(steps == 9, "wizard: %d steps" % steps)
        await pg.click("#wzNext")
        # FC mounted rotated 90° clockwise: the board reports the raw axes of that mounting
        await pg.evaluate("""(() => {
            const real = { level: [0, 0, -9.8], nose: [9.8, 0, 0] };
            window.WZ_POSE = 'level';
            wzSensors = async () => ({ acc: unAlign(real[window.WZ_POSE], 1).map((v) => v + (Math.random() - 0.5) * 0.1), baro: { ok: false } });
        })()""")
        await pg.click("#wzCapL")
        await pg.wait_for_timeout(900)
        await pg.evaluate("window.WZ_POSE = 'nose'")
        await pg.click("#wzCapN")
        await pg.wait_for_timeout(900)
        t = await pg.inner_text("#wzBody")
        check("90°" in t and await pg.locator("#wzUseAlign").count() == 1, "wizard: orientation solved from two poses (90°)")
        await pg.click("#wzUseAlign")
        check(await pg.evaluate("S.edits.imu_align") == 1, "wizard: imu_align = 1 put into the pending edits")
        await pg.wait_for_timeout(400)
        px = await pg.evaluate("""(() => { const c = $('wz3d'); const d = c.getContext('2d').getImageData(0, 0, c.width, c.height).data;
            let n = 0; for (let i = 3; i < d.length; i += 4) if (d[i] > 0) n++; return n; })()""")
        check(px > 2000, "wizard: 3D model drawn live on the mounting step (%d px)" % px)
        await pg.locator(".wz3d").screenshot(path=os.path.join(SHOTS, "extras-setup-3d.png"))
        await pg.click("#wzSteps button[data-i='6']")
        await pg.wait_for_timeout(600)
        check(await pg.locator("#wzRx li").count() >= 6, "wizard: receiver checklist follows the live channels")
        await pg.click("#wzSteps button[data-i='7']")
        check(await pg.locator("#wzBody [data-m]:disabled").count() == 4, "wizard: motor buttons locked until props-off is ticked")
        await pg.check("#wzProps")
        check(await pg.locator("#wzBody [data-m]:disabled").count() == 0, "wizard: motor buttons unlocked")
        await pg.click("#wzBody [data-m='2']")
        await pg.wait_for_timeout(400)
        await pg.wait_for_function("S.status && S.status.mtest && S.status.mtest[2] > 0", timeout=4000)
        mot = await pg.evaluate("S.status.mtest")
        check(mot == [0, 0, 0.06, 0], "wizard: only motor 3 spins in the firmware %s" % mot)
        await pg.screenshot(path=os.path.join(SHOTS, "extras-wizard.png"))
        await pg.wait_for_function("S.status && !S.status.mtest", timeout=5000)
        check(True, "wizard: motor stops by itself after 2 s")
        await pg.click("#wzSteps button[data-i='8']")
        check("Tóm tắt" in await pg.text_content("#wzBody"), "wizard: summary step")  # (headings are shown upper-case)
        await pg.click("#discardBtn")

        # ---- blackbox compare + CSV
        if os.path.exists(LOG):
            await tab(pg, "blackbox")
            await pg.set_input_files("#bbFile", LOG)
            await pg.wait_for_selector("#bbResults:not([hidden])", timeout=20000)
            await pg.click("#bbPin")
            await pg.set_input_files("#bbFile", LOG)
            await pg.wait_for_timeout(1500)
            rows = await pg.locator("#bbCmp tr").count()
            check(rows == 7 and await pg.evaluate("bbRefSeries().length") >= 2, "blackbox: pinned reference, comparison table (%d rows), dashed overlay" % rows)
            async with pg.expect_download() as dl:
                await pg.click("#bbCsv")
            d = await dl.value
            lines = open(await d.path(), encoding="utf-8").read().splitlines()
            n = await pg.evaluate("S.flight.n")
            check(lines[0].startswith("t_s,throttle,roll_setpoint") and len(lines) == n + 1, "blackbox: CSV export %d rows x %d columns" % (len(lines) - 1, len(lines[0].split(","))))
            # exports for Blackbox Explorer / PIDtoolbox
            async with pg.expect_download() as dl:
                await pg.click("#bbExpBbl")
            d = await dl.value
            raw = open(await d.path(), "rb").read()
            check(d.suggested_filename.endswith(".bbl") and raw.startswith(b"H Product:Blackbox flight data recorder") and b"Field I name:loopIteration,time," in raw
                  and raw.count(b"\nI") >= 1 and raw.endswith(b"End of log\x00"), "blackbox: .bbl export (%d bytes, %s)" % (len(raw), d.suggested_filename))
            async with pg.expect_download() as dl:
                await pg.click("#bbExpCsv")
            d = await dl.value
            lines = open(await d.path(), encoding="utf-8").read().splitlines()
            check(lines[0].startswith('"loopIteration","time (us)","axisP[0]"') and "gyroADC[0]" in lines[0] and len(lines) == n + 1,
                  "blackbox: Betaflight-style CSV export (%d rows)" % (len(lines) - 1))
            await pg.screenshot(path=os.path.join(SHOTS, "extras-blackbox.png"))  # viewport: a full-page shot of the long analysis page can stall headless Chromium
        else:
            print("(build/web/normal.bin missing: blackbox compare skipped)")

        # ---- navigation: survey generator, polygon fence editor, upload through the CLI
        sq = await pg.evaluate("surveyPath([[0,0],[100,0],[100,100],[0,100]], 20, 0)")
        check(len(sq) == 10 and all(-0.01 <= p[0] <= 100.01 and -0.01 <= p[1] <= 100.01 for p in sq),
              "survey: 100 m square, 20 m lanes -> 5 lanes, 10 waypoints inside the area")
        rot = await pg.evaluate("surveyPath([[0,0],[100,0],[100,100],[0,100]], 20, 90)")
        check(len(rot) == 10 and abs(rot[0][0] - rot[1][0]) < 0.01, "survey: 90 degree lanes run east-west")
        await tab(pg, "nav")
        await pg.wait_for_function("S.mission && S.mission.home !== undefined", timeout=8000)
        if await pg.evaluate("!!(S.mission && S.mission.home) || !!(S.status && S.status.gps && S.status.gps.fix)"):
            await pg.click("#wpEditBtn")
            await pg.click('#wpLayer [data-l="survey"]')
            box = await pg.locator("#navMap").bounding_box()
            cx, cy = box["x"] + box["width"] / 2, box["y"] + box["height"] / 2
            for dx, dy in ((-120, -90), (120, -90), (120, 90), (-120, 90)):
                await pg.mouse.click(cx + dx, cy + dy)
            await pg.fill("#svSpace", "10")
            await pg.dispatch_event("#svSpace", "change")
            await pg.click("#svMake")
            n_wp = await pg.evaluate("S.wp.items.length")
            check(n_wp > 6 and await pg.evaluate("S.wp.items[S.wp.items.length-1].cmd") == 20,
                  "survey: area drawn on the map becomes %d waypoints + return home" % n_wp)
            await pg.click('#wpLayer [data-l="fence"]')
            for dx, dy in ((-200, -140), (200, -140), (200, 140), (-200, 140)):
                await pg.mouse.click(cx + dx, cy + dy)
            await pg.check("#fcOn")
            info = await pg.text_content("#fcInfo")
            check(info.startswith("4 / 16") and "ngoài" not in info, "fence: 4-vertex polygon around home (%s)" % info)
            await pg.click("#wpUpload")
            await pg.wait_for_function("S.wp === null", timeout=10000)
            await pg.wait_for_timeout(800)
            m = await pg.evaluate("S.mission")
            check(len(m["poly"]) == 4 and m["polyOn"] and len(m["items"]) == n_wp,
                  "upload: mission (%d) and polygon fence (%d vertices, enabled) stored in the drone" % (len(m["items"]), len(m["poly"])))
            await pg.screenshot(path=os.path.join(SHOTS, "nav-survey-fence.png"))
        else:
            check(True, "navigation editor skipped: demo has no home position")

        # ---- overview card, PID sliders, mission files, backup compare, CLI completion, motor vibration
        await tab(pg, "overview")
        await pg.wait_for_selector("#pcPanel", timeout=5000)
        await pg.wait_for_function("document.getElementById('pcLoadT').textContent.includes('%')", timeout=8000)
        check(True, "overview: loop load card (%s)" % (await pg.text_content("#pcLoadT")))
        await tab(pg, "pid")
        await pg.wait_for_selector("#slPanel", timeout=5000)
        base = await pg.evaluate("SL.base.roll_p")
        await pg.evaluate("(() => { const el = document.querySelector('[data-sl=master]'); el.value = 1.2; el.dispatchEvent(new Event('input')); })()")
        got = await pg.evaluate("S.edits.roll_p")
        check(got == round(base * 1.2), "PID sliders: master x1.2 -> roll P %s -> %s" % (base, got))
        await pg.click('#profSeg [data-pf="1"]')
        check(await pg.locator('#pidMatrix [data-p="p2_roll_p"]').count() == 1 and await pg.locator('#rateMatrix [data-p="p2_yaw_max"]').count() == 1,
              "PID: Profile 2 is edited in the same matrix (p2_*)")
        await pg.click('#profSeg [data-pf="0"]')
        await pg.click("#slReset")
        check(await pg.evaluate("!('roll_p' in S.edits) || S.edits.roll_p == SL.base.roll_p"), "PID sliders: back to the reference")
        if await pg.locator("#discardBtn").is_visible():
            await pg.click("#discardBtn")
        wpl = "QGC WPL 110\n0\t1\t0\t16\t0\t0\t0\t0\t21.0285\t105.8542\t0\t1\n1\t0\t3\t22\t0\t0\t0\t0\t0\t0\t10\t1\n" + \
              "2\t0\t3\t16\t2\t0\t0\t0\t21.0290000\t105.8550000\t25\t1\n3\t0\t3\t20\t0\t0\t0\t0\t0\t0\t0\t1\n"
        items = await pg.evaluate("t => wplParse(t)", wpl)
        check(len(items) == 3 and items[1]["la"] == 210290000 and items[1]["p1"] == 2 and items[1]["alt"] == 25,
              "mission file: QGC WPL 110 parsed (home line skipped, 3 items)")
        await tab(pg, "params")
        await pg.wait_for_selector("#cmpPanel", timeout=5000)
        bk = os.path.join(SHOTS, "cmp-backup.txt")
        rp = await pg.evaluate("val('roll_p')")
        open(bk, "w").write("# test\nset roll_p = %d\nset pitch_p = %s\nset nope = 3\n" % (rp + 7, await pg.evaluate("val('pitch_p')")))
        await pg.set_input_files("#cmpFile", bk)
        await pg.wait_for_function("document.getElementById('cmpMsg').textContent.length > 0", timeout=5000)
        msg = await pg.text_content("#cmpMsg")
        check("2 tham số, 1 khác" in msg and await pg.locator("#cmpOut tr").count() == 2, "params: backup compare lists the one difference (%s)" % msg)
        await tab(pg, "cli")
        await pg.fill("#cliIn", "set roll_")
        await pg.press("#cliIn", "Tab")
        cli_val = await pg.input_value("#cliIn")
        check(cli_val.startswith("set roll_") and "roll_p" in await pg.text_content("#cliOut"), "CLI: Tab completes parameter names (%s)" % cli_val)
        await tab(pg, "motors")
        await pg.wait_for_selector("#vbPanel", timeout=5000)
        await pg.check("#mtArm")
        await pg.click("#vbGo")
        await pg.wait_for_function("document.querySelectorAll('#vbOut .mbar').length === 4", timeout=40000)
        check(True, "motors: per-motor vibration test ran on 4 motors (%s)" % (await pg.text_content("#vbMsg")))
        await pg.uncheck("#mtArm")

        # ---- ports: logic-condition editor (formats CLI 'logic' commands)
        await tab(pg, "ports")
        await pg.wait_for_function("document.querySelectorAll('#lgPanel tr[data-row]').length === 1", timeout=5000)
        await pg.click("#lgMore")
        check(await pg.locator("#lgPanel tr[data-row]").count() == 2, "ports: logic editor shows rules in use + one empty slot, '+ Thêm rule' opens another")
        await pg.locator('#lgPanel [data-lg="0"][data-k="en"]').check()
        await pg.locator('#lgPanel [data-lg="0"][data-k="op"]').select_option("2")
        await pg.locator('#lgPanel [data-lg="0"][data-k="at"]').select_option("2")
        await pg.locator('#lgPanel [data-lg="0"][data-k="av"]').select_option("3")
        await pg.locator('#lgPanel [data-lg="0"][data-k="bv"]').fill("500")
        await pg.locator('#lgPanel [data-lg="0"][data-k="bv"]').dispatch_event("change")
        await pg.locator('#lgPanel [data-lg="0"][data-k="ac"]').select_option("1")
        await pg.locator("#lgApply").click()
        await pg.wait_for_function("document.getElementById('lgMsg').textContent.length > 0", timeout=5000)
        out = await pg.evaluate("api('/api/cli', {method: 'POST', body: 'logic', text: true})")
        l1 = next((l.strip() for l in out.splitlines() if l.startswith("logic 1 ")), "")
        check(l1.startswith("logic 1 1 2 3 2 0 500 1 0 0"),
              "ports: logic rule 1 'home distance > 500 m -> beep' written through the CLI (%s)" % l1)
        # UART4/5 rows only where the board has them (status 'ports' bitmask)
        n4 = await pg.locator("#portTable [data-p=serial4_fn]").count()
        mask = await pg.evaluate("S.portMask")
        check((n4 > 0) == bool((mask >> 5) & 1), "ports: UART4 row follows the board's port mask (mask %s)" % mask)
        n7 = await pg.locator("#portTable [data-p=serial7_fn]").count()
        check((n7 > 0) == bool((mask >> 8) & 1), "ports: UART6 / UART7 rows (H743: 7 UARTs) follow the port mask")
        # wiring hint under each port: the UART instance and its pins, as the firmware reports them
        txt = await pg.evaluate("S.portPins = {'0': 'USART3 RX PD9 TX PD8', '8': 'USART6 RX PC7 TX PC6'}; buildPorts(); $('portTable').innerText")
        check("USART3 RX PD9 TX PD8" in txt and "USART6 RX PC7 TX PC6" in txt, "ports: UART pins shown under each port")
        await pg.locator("#portTable").screenshot(path=os.path.join(SHOTS, "extras-ports-pins.png"))
        await pg.evaluate("S.portPins = null; buildPorts()")

        # ---- sensors: calibration + mounting
        await tab(pg, "sensors")
        check(await pg.locator("#calPanel select[data-p=imu_align] option").count() == 8 and
              await pg.locator("#calBoard [data-p=board_yaw]").count() == 1 and await pg.locator("#calPanel [data-cal]").count() == 5,
              "sensors: calibration buttons (gyro, accel, accel 6-side, compass, compassmot), IMU / compass mounting (8 each), fine board alignment")
        await pg.select_option("#calPanel select[data-p=mag_align]", "2")
        await pg.click("#calBoard [aria-label='Tăng board_yaw']")
        check(await pg.evaluate("S.edits.mag_align === 2 && S.edits.board_yaw === 1"), "sensors: mounting changes go to the pending edits")
        await pg.click("#applyBtn")
        await pg.wait_for_timeout(600)
        back = await pg.evaluate("api('/api/cli', { method: 'POST', body: 'get board_yaw', text: true })")
        check("board_yaw = 1" in back, "sensors: board_yaw applied in the firmware (%s)" % back.strip())
        await pg.click("#calPanel [data-cal=calib_gyro]")
        await pg.wait_for_timeout(400)
        check("gyro" in (await pg.inner_text("#calMsg")).lower(), "sensors: gyro calibration started from the panel")
        await pg.screenshot(path=os.path.join(SHOTS, "extras-calibration.png"), full_page=True)

        # ---- motors: DShot ESC commands (the demo runs DShot300)
        await tab(pg, "motors")
        check(await pg.locator("#escCalPanel [data-beep]").count() == 5 and await pg.locator("#escCalPanel [data-dir]").count() == 8,
              "motors: DShot -> beep and spin-direction buttons, no throttle calibration")
        await pg.check("#mtArm")
        await pg.click("#escCalPanel [data-dir='3_1']")
        await pg.wait_for_timeout(500)
        check(await pg.evaluate("document.getElementById('toast').textContent.includes('Motor 3')"), "motors: spin direction of motor 3 saved through the firmware")
        # expected response preview (same signs as the firmware mixer)
        r = await pg.evaluate("""(() => { const b = Object.assign({}, S.status);
            const t = (att, rc) => { rxPreview(Object.assign({}, b, { att, rc })); return rxPreview.last.map(v => Math.sign(Math.round(v * 100))); };
            return [t([20, 0, 0], [1500, 1500, 1000, 1500]), t([0, -20, 0], [1500, 1500, 1000, 1500]),
                    t([0, 0, 0], [2000, 1500, 1000, 1500]), t([0, 0, 0], [1500, 2000, 1000, 1500])]; })()""")
        # M1 rear-right, M2 front-right, M3 rear-left, M4 front-left
        check(r == [[1, 1, -1, -1], [-1, 1, -1, 1], [-1, -1, 1, 1], [1, -1, 1, -1]],
              "motors: preview - tilt right raises the right motors, nose down the front ones, roll stick right the left ones, pitch forward the rear ones %s" % r)
        # motor order: the wizard spins outputs 1..4, the user clicks where the motor is
        check(await pg.locator("#motOrderPanel select[data-p]").count() == 4 and await pg.locator("#moProps button").count() == 2,
              "motors: output per position (4) and props-in / props-out")
        await pg.click("#moStart")
        answer = {1: 4, 2: 2, 3: 1, 4: 3}   # output -> mixer motor at that position (front-left, front-right, rear-right, rear-left)
        spun = []
        for k in (1, 2, 3, 4):
            await pg.wait_for_function("S.status && S.status.mtest && S.status.mtest.some(v => v > 0)", timeout=4000)
            spun.append(await pg.evaluate("S.status.mtest.findIndex(v => v > 0) + 1"))
            await pg.click("#moStepBox [data-pos='%d']" % answer[k])
            await pg.wait_for_timeout(400)
        got = await pg.evaluate("[1,2,3,4].map(i => S.edits['motor' + i + '_out'] ?? S.params['motor' + i + '_out'].v)")
        check(spun == [1, 2, 3, 4] and got == [3, 2, 4, 1], "motors: order found by clicking (outputs spun %s) -> motor1..4_out %s" % (spun, got))
        await pg.click("#moProps button[data-v='1']")
        await pg.click("#applyBtn")
        await pg.wait_for_timeout(600)
        back = await pg.evaluate("api('/api/cli', { method: 'POST', body: 'get motor', text: true })")
        check("motor4_out = 1" in back and "motor1_out = 3" in back, "motors: new order applied in the firmware")
        check(await pg.evaluate("S.params.yaw_reversed.v") == 1, "motors: props-out applied")
        await pg.screenshot(path=os.path.join(SHOTS, "extras-motor-order.png"), full_page=True)
        # bench test with RPM telemetry: stub the sensor reading (the demo has no bidirectional DShot);
        # motor 3 "does not spin"
        await pg.evaluate("""window._wzs = wzSensors; wzSensors = async () => ({ rpm: (S.status.mtest || [0, 0, 0, 0]).map((v, i) => i === 2 ? 0 : Math.round(v * 30000)) })""")
        await pg.click("#btStart")
        await pg.wait_for_function("document.getElementById('btTable').rows.length === 5 && !BT.run", timeout=20000)
        grades = await pg.evaluate("[...document.querySelectorAll('#btTable tr td:last-child')].map(td => td.innerText)")
        note = await pg.inner_text("#btNote")
        check(grades[0] == "Đạt" and grades[1] == "Đạt" and grades[3] == "Đạt" and "Không quay" in grades[2] and "M3" in note,
              "motors: RPM bench test - M1/M2/M4 pass, M3 reported as not spinning %s" % grades)
        g = await pg.evaluate("[btGrade({m: 0, max: [2000, 0, 0, 0], seen: [true, true, true, true]}), btGrade({m: 1, max: [0, 1800, 900, 0], seen: [true, true, true, true]}), btGrade({m: 2, max: [0, 0, 0, 0], seen: [true, true, false, true]})]")
        check(g == ["ok", "cross", "noTelem"], "motors: bench test grading ok / cross / no telemetry %s" % g)
        await pg.screenshot(path=os.path.join(SHOTS, "extras-bench-test.png"), full_page=True)
        await pg.evaluate("wzSensors = window._wzs")
        # PWM ESCs: the throttle calibration steps instead
        await pg.evaluate("S.params.motor_protocol.v = 0; escBuild()")
        check(await pg.locator("#escSteps li").count() == 5 and await pg.is_disabled("#escCalStart"), "motors: PWM -> 5-step throttle calibration, locked until props off + battery out")
        await pg.check("#escCalOk")
        check(await pg.is_enabled("#escCalStart"), "motors: calibration unlocked after the confirmation")
        await pg.screenshot(path=os.path.join(SHOTS, "extras-esc.png"), full_page=True)
        await pg.evaluate("S.params.motor_protocol.v = 2; escBuild()")
        await pg.uncheck("#mtArm")

        # ---- warning chips
        await tab(pg, "overview")
        # set and read in one step: the live status stream would redraw the chips in between
        chips = await pg.evaluate("(() => { const s = Object.assign({}, S.status, {warn: 512 | 1024, vibe: 41}); extrasStatus(s); return $('xWarn').innerText; })()")
        check("Rung mạnh" in chips and "EKF" in chips, "overview: vibration and EKF warning chips")
        # attitude as a 3D model instead of the horizon (remembered)
        await pg.click("#ovView button[data-v='3d']")
        await pg.wait_for_timeout(500)
        v3 = await pg.evaluate("""(() => { const c = $('ov3d'); const d = c.getContext('2d').getImageData(0, 0, c.width, c.height).data;
            let n = 0; for (let i = 3; i < d.length; i += 4) if (d[i] > 0) n++; return [n, $('ovHorizon').hidden, localStorage.getItem('aq.ovview')]; })()""")
        check(v3[0] > 1500 and v3[1] and v3[2] == "3d", "overview: 3D attitude model instead of the horizon %s" % v3)
        q = await pg.evaluate("qEuler(30, -20, 90).map(v => +v.toFixed(4))")
        R = await pg.evaluate("(() => { const R = qMat(qEuler(30, -20, 90)); return [Math.atan2(R[2][1], R[2][2]) * 180 / Math.PI, -Math.asin(R[2][0]) * 180 / Math.PI, Math.atan2(R[1][0], R[0][0]) * 180 / Math.PI].map(v => Math.round(v)); })()")
        check(R == [30, -20, 90], "overview: euler -> quaternion round trip %s %s" % (R, q))
        await pg.locator(".ov-att").screenshot(path=os.path.join(SHOTS, "extras-ov-3d.png"))
        await pg.click("#ovView button[data-v='h']")
        # motor card: disarmed -> protocol + ESC telemetry per position + test shortcut; armed -> bars
        mc = await pg.evaluate("""(() => { const b = Object.assign({}, S.status, {armed: false, mtest: undefined, erpm: [1200, -1, 900, 1500], etmp: [33, -1, 30, 35]});
            renderOvMotors(b); const idle = [!$('motIdle').hidden, $('motorBars').hidden, $('moProto').textContent, $('moQuad').innerText];
            renderOvMotors(Object.assign({}, b, {armed: true})); const armed = [$('motIdle').hidden, !$('motorBars').hidden, $('motorBars').children.length];
            return [idle, armed]; })()""")
        check(mc[0][0] and mc[0][1] and "DShot" in mc[0][2] and "1200 rpm" in mc[0][3] and "33 °C" in mc[0][3] and mc[1] == [True, True, 4],
              "overview: motor card shows protocol / RPM / temperature when disarmed, bars when armed %s" % mc)
        await pg.evaluate("renderOvMotors(Object.assign({}, S.status, {armed: false, erpm: [2100, 2050, 2200, 2080], etmp: [31, 30, 32, 31]}))")
        await pg.locator(".ov-mot").screenshot(path=os.path.join(SHOTS, "extras-ov-motor.png"))
        await pg.evaluate("document.getElementById('comClose').click()")
        await pg.wait_for_timeout(300)
        check(await pg.evaluate("LINK.worker === null || LINK.worker === undefined") and await pg.is_visible("#connect"), "demo: disconnect stops the worker")
        check(not errs, "no page errors %s" % errs[:3])
        await b.close()
    print("\n" + ("all extras checks passed" if not fails else "%d FAILED" % len(fails)))
    sys.exit(1 if fails else 0)


asyncio.run(main())
