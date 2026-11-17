#!/usr/bin/env python3
"""Installable app (GitHub Pages copy) end to end:
tools/make_pages.py builds the site into a temp folder, a local web server stands in for GitHub Pages.
Checks: manifest + icons + service worker, the app opens offline from the cache, a newer build on the
site is loaded at once when no FC is connected, and waits (bar + auto-apply on disconnect) when one is.
Needs build/web/AllQuad-Configurator.html (tools/embed_web.py) and Playwright + Chromium.
    python3 test/web/test_pwa.py [screenshot_dir]
"""
import asyncio, functools, http.server, json, os, re, shutil, socketserver, subprocess, sys, tempfile, threading
from playwright.async_api import async_playwright

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SHOTS = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "build", "web", "shots")
fails = []


def check(ok, msg):
    print("[%s] %s" % ("PASS" if ok else "FAIL", msg))
    if not ok:
        fails.append(msg)


class Quiet(http.server.SimpleHTTPRequestHandler):
    def log_message(self, *a):
        pass


def publish(out, build, version):
    """what a new push to main does: new page + new version.json on the site"""
    p = os.path.join(out, "index.html")
    s = open(p, encoding="utf-8").read()
    s = re.sub(r'<meta name="aq-build" content="[^"]*">', '<meta name="aq-build" content="%s">' % build, s)
    s = re.sub(r'<meta name="aq-version" content="[^"]*">', '<meta name="aq-version" content="%s">' % version, s)
    open(p, "w", encoding="utf-8").write(s)
    json.dump({"version": version, "build": build, "date": "2026-11-17 20:00"}, open(os.path.join(out, "version.json"), "w"))


async def main():
    os.makedirs(SHOTS, exist_ok=True)
    tmp = tempfile.mkdtemp(prefix="aq_pages_")
    out = os.path.join(tmp, "AllQuad")  # served under /AllQuad/ like https://<user>.github.io/AllQuad/
    r = subprocess.run([sys.executable, os.path.join(ROOT, "tools", "make_pages.py"), out], capture_output=True, text=True)
    check(r.returncode == 0, "make_pages: " + (r.stdout.strip().splitlines() or [r.stderr])[0])
    names = set(os.listdir(out))
    check({"index.html", "sw.js", "manifest.webmanifest", "version.json", "icon-192.png", "icon-512.png"} <= names, "site has the app files %s" % sorted(names))
    man = json.load(open(os.path.join(out, "manifest.webmanifest"), encoding="utf-8"))
    ver = json.load(open(os.path.join(out, "version.json")))
    page = open(os.path.join(out, "index.html"), encoding="utf-8").read()
    check(man["display"] == "standalone" and len(man["icons"]) == 2 and man["start_url"] == "./", "manifest: standalone app, 2 icons")
    check('<meta name="aq-build" content="%s">' % ver["build"] in page and 'rel="manifest"' in page, "page carries the build id of version.json (%s)" % ver["build"])
    src = open(os.path.join(ROOT, "build", "web", "AllQuad-Configurator.html"), encoding="utf-8").read()
    check('<meta name="aq-build"' not in src and 'rel="manifest"' not in src, "the page embedded in the firmware stays without app tags")

    httpd = socketserver.ThreadingTCPServer(("127.0.0.1", 0), functools.partial(Quiet, directory=tmp))
    httpd.daemon_threads = True
    port = httpd.server_address[1]
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    url = "http://127.0.0.1:%d/AllQuad/" % port
    try:
        async with async_playwright() as p:
            exe = os.environ.get("CHROMIUM")
            b = await p.chromium.launch(**({"executable_path": exe} if exe else {}))
            ctx = await b.new_context(viewport={"width": 1100, "height": 800})
            pg = await ctx.new_page()
            errs = []
            pg.on("pageerror", lambda e: errs.append(str(e)))
            pg.on("dialog", lambda d: asyncio.ensure_future(d.accept()))
            await pg.goto(url)
            await pg.wait_for_function("navigator.serviceWorker && navigator.serviceWorker.controller", timeout=10000)
            check(True, "service worker installed and controls the page")
            info = await pg.inner_text("#pwaInfo")
            check(ver["build"][:7] in info and "tự cập nhật" in info, "connect screen shows the app build: " + info)

            # ---- offline: the app still opens (from the cache)
            await ctx.set_offline(True)
            await pg.reload()
            await pg.wait_for_selector("#comPick", timeout=8000)
            check(await pg.evaluate("PWA.build") == ver["build"] and "offline" in await pg.inner_text("#pwaInfo"),
                  "offline: app opens from the cache (%s)" % await pg.inner_text("#pwaInfo"))
            await pg.screenshot(path=os.path.join(SHOTS, "pwa-offline.png"))
            await ctx.set_offline(False)

            # ---- a newer build published, no FC connected: updates at once
            publish(out, "b2b2b2b2b2b2", "9.1")
            await pg.evaluate("pwaCheck()")
            await pg.wait_for_function("typeof PWA !== 'undefined' && PWA.build === 'b2b2b2b2b2b2'", timeout=10000)
            check("?v=b2b2b2b2b2b2" in pg.url, "new build loaded at once when no FC is connected (%s)" % pg.url)

            # ---- connected: a bar waits, applied when the FC is disconnected
            await pg.wait_for_selector("#comPick")
            await pg.evaluate("$('connect').hidden = true")  # as when connected
            publish(out, "c3c3c3c3c3c3", "9.2")
            await pg.evaluate("pwaCheck()")
            await pg.wait_for_selector("#pwaBar:not([hidden]) #pwaNow", timeout=5000)
            bar = await pg.inner_text("#pwaBar")
            check("9.2" in bar and await pg.evaluate("PWA.build") == "b2b2b2b2b2b2", "connected: update offered in a bar, page kept: " + bar.replace("\n", " "))
            await pg.screenshot(path=os.path.join(SHOTS, "pwa-update-bar.png"))
            await pg.evaluate("$('connect').hidden = false")  # disconnected
            await pg.wait_for_function("typeof PWA !== 'undefined' && PWA.build === 'c3c3c3c3c3c3'", timeout=12000)
            check(True, "update applied by itself after the FC was disconnected")

            # ---- the page offline after the update is the new one
            await ctx.set_offline(True)
            await pg.goto(url)
            await pg.wait_for_selector("#comPick", timeout=8000)
            check(await pg.evaluate("PWA.build") == "c3c3c3c3c3c3", "offline copy is the updated build")
            await ctx.set_offline(False)
            check(not errs, "no page errors %s" % errs[:3])
            await b.close()
    finally:
        httpd.shutdown()
        shutil.rmtree(tmp, ignore_errors=True)
    print("\n" + ("all pwa checks passed" if not fails else "%d FAILED" % len(fails)))
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    asyncio.run(main())
