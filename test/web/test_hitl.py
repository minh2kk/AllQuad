#!/usr/bin/env python3
"""Hardware in the loop from the configurator (src/web/ui/hitl.js) against the demo drone: the firmware
compiled to WebAssembly takes the page's sensor frames exactly like a board on USB, flies the check flight
(arm, ALTHOLD, POSHOLD, LAND) and the page grades it.
Needs build/web/AllQuad-Configurator.html with the demo (tools/build_wasm.sh, tools/embed_web.py)."""
import asyncio, os, sys
from playwright.async_api import async_playwright

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SHOTS = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "build", "web", "shots")
PAGE = os.path.join(ROOT, "build", "web", "AllQuad-Configurator.html")
fails = []


def check(ok, msg):
    print("[%s] %s" % ("PASS" if ok else "FAIL", msg))
    if not ok:
        fails.append(msg)


async def main():
    os.makedirs(SHOTS, exist_ok=True)
    async with async_playwright() as p:
        exe = os.environ.get("CHROMIUM")
        b = await p.chromium.launch(**({"executable_path": exe} if exe else {}))
        pg = await b.new_page(viewport={"width": 1280, "height": 900})
        errs = []
        pg.on("pageerror", lambda e: errs.append(str(e)))
        await pg.goto("file://" + PAGE)
        await pg.wait_for_selector("#demoBtn:not([hidden])", timeout=5000)
        await pg.click("#demoBtn")
        await pg.wait_for_function("Object.keys(S.params).length > 200", timeout=20000)
        # blackbox: the navigation frames of the simulator's EKF fault flight (node test/web/test_analysis.js)
        log = os.path.join(ROOT, "build", "web", "ekf_faults.bin")
        if os.path.exists(log):
            await pg.click("nav button[data-tab=blackbox]")
            await pg.set_input_files("#bbFile", log)
            await pg.wait_for_selector("#bbNav", timeout=15000)
            await pg.wait_for_timeout(500)
            ev = await pg.inner_text("#bbNavEv")
            leg = await pg.inner_text("#bbNavLeg")
            check("baro bị loại" in ev and "la bàn lệch" in ev and "La bàn" in leg,
                  "blackbox: EKF chart, events of the fault flight (%s)" % ev.replace("\n", " | ")[:160])
            await pg.locator("#bbNav").screenshot(path=os.path.join(SHOTS, "bb-nav.png"))
        else:
            print("(build/web/ekf_faults.bin missing: run node test/web/test_analysis.js; EKF chart check skipped)")
        await pg.click("nav button[data-tab=system]")
        await pg.wait_for_selector("#hitlPanel:not([hidden])", timeout=3000)
        check(True, "HITL panel shown on a USB / demo link")
        await pg.click("#hitlAuto")
        await pg.wait_for_timeout(20000)
        await pg.screenshot(path=os.path.join(SHOTS, "hitl-flying.png"))
        mid = await pg.evaluate("({on: HITL.on, armed: HITL.armed, got: HITL.got, mode: HITL.mode})")
        check(mid["on"] and mid["armed"] and mid["got"] > 1000, "flying: FC answers, armed (%s)" % mid)
        await pg.wait_for_function("window.AQ_HITL_RESULT", timeout=45000)
        res = await pg.evaluate("window.AQ_HITL_RESULT")
        for name, ok, detail in res["items"]:
            check(ok, "%s: %s" % (name, detail))
        await pg.screenshot(path=os.path.join(SHOTS, "hitl-result.png"))
        await pg.wait_for_selector("#connect:not([hidden])", timeout=5000)
        check(True, "after HITL: back on the connect screen (the FC restarts)")
        check(not errs, "no page errors %s" % errs)
        await b.close()
    print("\n%s" % ("all HITL page checks passed" if not fails else "%d HITL page check(s) FAILED" % len(fails)))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
