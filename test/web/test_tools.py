#!/usr/bin/env python3
"""Laptop tools end to end against the real firmware (SITL through tools/configurator.py):
CLI, sensors + 3D model, motor test, ports / VTX / LED, OSD layout, mission editor, presets.
Needs build/sim (tools/build_host.sh) and Playwright + Chromium.
    python3 test/web/test_tools.py [screenshot_dir]
"""
import asyncio, os, subprocess, sys, time
from playwright.async_api import async_playwright

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SHOTS = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "build", "web", "shots")
PORT = 8182
fails = []


def check(ok, msg):
    print("[%s] %s" % ("PASS" if ok else "FAIL", msg))
    if not ok:
        fails.append(msg)


async def tab(pg, name):
    await pg.click("nav button[data-tab=%s]" % name)
    await pg.wait_for_timeout(300)


async def main():
    os.makedirs(SHOTS, exist_ok=True)
    srv = subprocess.Popen([sys.executable, os.path.join(ROOT, "tools", "configurator.py"), "--port", str(PORT), "--no-browser"],
                           stdout=subprocess.DEVNULL)
    time.sleep(1.0)
    try:
        async with async_playwright() as p:
            exe = os.environ.get("CHROMIUM")
            b = await p.chromium.launch(**({"executable_path": exe} if exe else {}))
            pg = await b.new_page(viewport={"width": 1280, "height": 860})
            errs = []
            pg.on("pageerror", lambda e: errs.append(str(e)))
            pg.on("dialog", lambda d: asyncio.ensure_future(d.accept()))
            await pg.goto("http://127.0.0.1:%d/?usb" % PORT)
            await pg.wait_for_selector("#sitlBtn:not([hidden])", timeout=5000)
            await pg.click("#sitlBtn")
            await pg.wait_for_function("Object.keys(S.params).length > 200", timeout=10000)
            await pg.wait_for_timeout(800)

            # ---- CLI
            await tab(pg, "cli")
            await pg.fill("#cliIn", "get roll_p")
            await pg.press("#cliIn", "Enter")
            await pg.wait_for_function("document.getElementById('cliOut').textContent.includes('roll_p')", timeout=5000)
            await pg.fill("#cliIn", "set roll_p 52")
            await pg.press("#cliIn", "Enter")
            await pg.wait_for_function("S.params.roll_p && S.params.roll_p.v === 52", timeout=5000)
            check(True, "CLI: get / set run in the firmware, parameters reload")
            await pg.click("#cliQuick [data-c=dump]")
            await pg.wait_for_function("document.getElementById('cliOut').textContent.includes('set led_count')", timeout=8000)
            await pg.press("#cliIn", "ArrowUp")
            hist = await pg.input_value("#cliIn")
            check(hist == "dump", "CLI: long dump read in pages, history with the up arrow (%s)" % hist)
            await pg.screenshot(path=os.path.join(SHOTS, "tools-cli.png"))

            # ---- sensors + 3D
            await tab(pg, "sensors")
            await pg.wait_for_function("SENS.gyro[0].length > 15", timeout=6000)
            att = await pg.inner_text("#modelAtt")
            kv = await pg.inner_text("#sensKv")
            check("roll" in att and "Baro" in kv and "hPa" in kv, "Sensors: live graphs at 20 Hz, 3D model, baro (%s)" % att)
            await pg.screenshot(path=os.path.join(SHOTS, "tools-sensors.png"))

            # ---- motor test
            await tab(pg, "motors")
            locked = await pg.evaluate("document.getElementById('mtLock').disabled")
            await pg.check("#mtArm")
            await pg.evaluate("""(() => { const r = document.querySelectorAll('#mtSliders input')[2];
                                  r.value = 30; r.dispatchEvent(new Event('input')); })()""")
            await pg.wait_for_function("S.status && S.status.mtest && S.status.mtest[1] === 0.3", timeout=4000)
            m = await pg.evaluate("S.status.mtest")
            await pg.wait_for_timeout(1500)  # keep-alive must hold it
            still = await pg.evaluate("S.status.mtest && S.status.mtest[1] === 0.3")
            blocked = await pg.evaluate("S.status.blockers.includes('CLI') || S.status.blockers.length > 0")
            check(locked and m == [0, 0.3, 0, 0] and still and blocked,
                  "Motor test: locked until 'props off', M2 at 30 %% in the firmware, kept alive, arming blocked (%s)" % m)
            await pg.screenshot(path=os.path.join(SHOTS, "tools-motors.png"))
            await tab(pg, "overview")
            await pg.wait_for_function("S.status && !S.status.mtest", timeout=4000)
            check(True, "Motor test stops when the tab is left")

            # ---- ports, VTX, LED
            await tab(pg, "ports")
            await pg.select_option("select[data-p=serial3_fn]", "9")
            await pg.select_option("select[data-p=vtx_band]", "4")
            await pg.select_option("select[data-p=vtx_channel]", "5")
            freq = await pg.inner_text("#vtxFreq")
            src = await pg.inner_text("#vtxSrc")
            leds = await pg.evaluate("document.querySelectorAll('#ledPreview i').length")
            check(freq == "5820 MHz" and "SmartAudio" in src and leds == 8,
                  "Ports: UART3 -> VTX SmartAudio, band F ch 5 = %s, LED preview (%d)" % (freq, leds))
            await pg.screenshot(path=os.path.join(SHOTS, "tools-ports.png"), full_page=True)

            # ---- OSD drag
            await tab(pg, "osd")
            before = await pg.evaluate("val('osd_vbat_pos')")
            await pg.evaluate("window.scrollTo(0, document.getElementById('osdCanvas').getBoundingClientRect().top + scrollY + 250)")
            await pg.wait_for_timeout(150)
            box = await pg.locator("#osdCanvas").bounding_box()
            cw = box["width"] / 30
            ch = cw * 1.45
            row, col = (before & 0x7FFF) // 32, (before & 0x7FFF) % 32
            sx, sy = box["x"] + (col + 1.5) * cw, box["y"] + (row + 0.5) * ch
            await pg.mouse.move(sx, sy)
            await pg.mouse.down()
            await pg.mouse.move(sx + 5 * cw, sy - 6 * ch, steps=6)
            await pg.mouse.up()
            after = await pg.evaluate("val('osd_vbat_pos')")
            check(after & 0x8000 and (after & 0x7FFF) == (row - 6) * 32 + col + 5,
                  "OSD: battery voltage dragged from row %d col %d to row %d col %d" % (row, col, ((after & 0x7FFF) // 32), (after & 0x7FFF) % 32))
            await pg.click("#osdList input[data-osd=osd_coord_pos]")
            coord = await pg.evaluate("val('osd_coord_pos') & 0x8000")
            check(coord != 0, "OSD: hidden element switched on from the list")
            await pg.screenshot(path=os.path.join(SHOTS, "tools-osd.png"))
            await pg.click("#discardBtn")

            # ---- mission editor
            await tab(pg, "nav")
            await pg.wait_for_timeout(500)
            await pg.click("#wpEditBtn")
            await pg.wait_for_selector("#wpPanel:not([hidden])", timeout=3000)
            mb = await pg.locator("#navMap").bounding_box()
            pts = [(0.62, 0.30), (0.80, 0.55), (0.40, 0.75)]
            for fx, fy in pts:
                await pg.mouse.click(mb["x"] + mb["width"] * fx, mb["y"] + mb["height"] * fy)
                await pg.wait_for_timeout(120)
            n = await pg.evaluate("S.wp.items.length")
            # drag the second point
            T = await pg.evaluate("S.navT")
            it = await pg.evaluate("S.wp.items[1]")
            px = mb["x"] + T["w"] / 2 + (it["e"] - T["ce"]) * T["k"]
            py = mb["y"] + T["h"] / 2 - (it["n"] - T["cn"]) * T["k"]
            await pg.mouse.move(px, py)
            await pg.mouse.down()
            await pg.mouse.move(px + 40, py + 30, steps=5)
            await pg.mouse.up()
            moved = await pg.evaluate("S.wp.items[1]")
            await pg.fill("#wpList .wp:nth-child(3) input[data-k=alt]", "35")
            await pg.press("#wpList .wp:nth-child(3) input[data-k=alt]", "Tab")
            await pg.click("#wpRtl")
            await pg.screenshot(path=os.path.join(SHOTS, "tools-mission.png"))
            await pg.click("#wpUpload")
            await pg.wait_for_selector("#wpPanel[hidden]", state="attached", timeout=8000)
            m = await pg.evaluate("api('/api/mission')")
            cmds = [i["cmd"] for i in m["items"]]
            check(n == 3 and abs(moved["e"] - it["e"]) > 1 and cmds == [16, 16, 16, 20] and m["items"][2]["alt"] == 35,
                  "Mission editor: 3 taps, one dragged, altitude edited, RTL added, sent and read back %s" % cmds)

            # ---- presets
            await tab(pg, "params")
            await pg.click("#presetList .preset:nth-child(4) button")
            ed = await pg.evaluate("S.edits")
            check(ed.get("roll_p") == 38 and ed.get("gyro_lpf_hz") == 100, "Preset '7 inch' staged its changes (%d)" % len(ed))
            await pg.click("#discardBtn")
            check(not errs, "no JavaScript errors %s" % errs[:3])
            await b.close()
    finally:
        srv.terminate()
    print("\n" + ("all tool checks passed" if not fails else "%d tool checks failed" % len(fails)))
    sys.exit(1 if fails else 0)


asyncio.run(main())
