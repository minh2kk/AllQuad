#!/usr/bin/env python3
"""Bundle the web configurator into firmware.

src/web/ui/index.html + analysis.js + tools.js + extras.js + pro.js
  -> build/web/index.html            (single file, minified, open in a browser for preview)
  -> build/web/index.dev.html        (same, not minified)
  -> build/web/AllQuad-Configurator.html  (same page for a laptop: open in Chrome/Edge, connect USB;
                                          also carries build/web/allquad.wasm for the "drone ảo" demo)
  -> src/web/web_ui_data.h           (gzip bytes served by the ESP32 at "/")

Run after editing anything in src/web/ui/. PlatformIO also runs it automatically
through extra_scripts (tools/pio_embed_web.py).
"""
import base64
import gzip
import re
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
UI = os.path.join(ROOT, "src", "web", "ui")
OUT_H = os.path.join(ROOT, "src", "web", "web_ui_data.h")
OUT_HTML = os.path.join(ROOT, "build", "web", "index.html")


def bundle():
    with open(os.path.join(UI, "index.html"), encoding="utf-8") as f:
        html = f.read()
    with open(os.path.join(UI, "analysis.js"), encoding="utf-8") as f:
        js = f.read()
    with open(os.path.join(UI, "tools.js"), encoding="utf-8") as f:
        tools = f.read()
    with open(os.path.join(UI, "extras.js"), encoding="utf-8") as f:
        extras = f.read()
    with open(os.path.join(UI, "pro.js"), encoding="utf-8") as f:
        pro = f.read()
    for ph in ("/*ANALYSIS_JS*/", "/*TOOLS_JS*/", "/*EXTRAS_JS*/", "/*PRO_JS*/"):
        if ph not in html:
            sys.exit("index.html: placeholder %s missing" % ph)
    html = html.replace("/*ANALYSIS_JS*/", js.replace("</script", "<\\/script"))
    html = html.replace("/*TOOLS_JS*/", tools.replace("</script", "<\\/script"))
    html = html.replace("/*EXTRAS_JS*/", extras.replace("</script", "<\\/script"))
    return html.replace("/*PRO_JS*/", pro.replace("</script", "<\\/script"))


WASM = os.path.join(ROOT, "build", "web", "allquad.wasm")


def with_demo(html):
    """Laptop copies only: the firmware compiled to WebAssembly (tools/build_wasm.sh) as base64,
    for the in-browser demo. The ESP32 page leaves it out (flash space; the button stays hidden)."""
    if not os.path.exists(WASM):
        return html
    with open(WASM, "rb") as f:
        b64 = base64.b64encode(f.read()).decode("ascii")
    return html.replace("<script>", '<script>window.AQ_DEMO_WASM="%s";</script>\n<script>' % b64, 1)


def _strip_js(code):
    """Line-based and conservative: drops whole-line // comments, trailing ' // ' comments after a
    statement end and the indentation. Nothing inside a line is rewritten otherwise."""
    out = []
    for line in code.split("\n"):
        t = line.strip()
        if not t or t.startswith("//"):
            continue
        m = re.search(r"([;{},)])\s+// [^`'\"]*$", t)
        if m:
            t = t[:m.start() + 1]
        out.append(t)
    return "\n".join(out)


def _strip_css(css):
    css = re.sub(r"/\*.*?\*/", "", css, flags=re.S)
    return "\n".join(l.strip() for l in css.split("\n") if l.strip())


def _full_minify(js, css):
    """terser + csso through node (tools/minify_web.js); None when node or the packages are missing."""
    import json
    import shutil
    import subprocess
    node = shutil.which("node")
    if not node:
        return None
    try:
        r = subprocess.run([node, os.path.join(ROOT, "tools", "minify_web.js")], input=json.dumps({"js": js, "css": css}),
                           capture_output=True, text=True, timeout=120, cwd=ROOT)
        if r.returncode != 0:
            return None
        out = json.loads(r.stdout)
        return out["js"], out["css"]
    except (OSError, ValueError, subprocess.SubprocessError):
        return None


def minify(html):
    html = re.sub(r"<!--.*?-->", "", html, flags=re.S)
    js = re.findall(r"<script>(.*?)</script>", html, flags=re.S)
    css = re.findall(r"<style>(.*?)</style>", html, flags=re.S)
    full = _full_minify(js, css)
    if full:  # real minifier: ~25 % smaller after gzip
        it_js, it_css = iter(full[0]), iter(full[1])
        html = re.sub(r"(<style>)(.*?)(</style>)", lambda m: m.group(1) + next(it_css) + m.group(3), html, flags=re.S)
        html = re.sub(r"(<script>)(.*?)(</script>)", lambda m: m.group(1) + next(it_js).replace("</script", "<\\/script") + m.group(3), html, flags=re.S)
        print("embed_web: terser + csso")
    else:  # conservative line-based fallback (no node / packages)
        html = re.sub(r"(<style>)(.*?)(</style>)", lambda m: m.group(1) + _strip_css(m.group(2)) + m.group(3), html, flags=re.S)
        html = re.sub(r"(<script>)(.*?)(</script>)", lambda m: m.group(1) + _strip_js(m.group(2)) + m.group(3), html, flags=re.S)
    # markup outside scripts/styles: indentation only
    parts = re.split(r"(<script>.*?</script>|<style>.*?</style>)", html, flags=re.S)
    return "".join(p if p.startswith("<script>") or p.startswith("<style>") else
                   "\n".join(l.strip() for l in p.split("\n") if l.strip()) for p in parts)


def main():
    src = bundle()
    os.makedirs(os.path.dirname(OUT_HTML), exist_ok=True)
    with open(OUT_HTML.replace(".html", ".dev.html"), "w", encoding="utf-8") as f:
        f.write(src)  # readable copy for debugging
    html = minify(src)  # what the ESP32 serves (and the UI test runs against)
    os.makedirs(os.path.dirname(OUT_HTML), exist_ok=True)
    with open(OUT_HTML, "w", encoding="utf-8") as f:
        f.write(html)
    with open(os.path.join(os.path.dirname(OUT_HTML), "AllQuad-Configurator.html"), "w", encoding="utf-8") as f:
        f.write(with_demo(html))  # for the laptop (open, connect USB, or try the demo)
    gz = gzip.compress(html.encode("utf-8"), compresslevel=9, mtime=0)
    lines = []
    for i in range(0, len(gz), 20):
        lines.append("  " + ", ".join("0x%02x" % b for b in gz[i:i + 20]) + ",")
    header = (
        "// Generated by tools/embed_web.py from src/web/ui/. Do not edit.\n"
        "#pragma once\n#include <stddef.h>\n#include <stdint.h>\n"
        "#if defined(ARDUINO)\n#include <pgmspace.h>\n#else\n#define PROGMEM\n#endif\n\n"
        "static const size_t WEB_UI_GZ_LEN = %d;\n"
        "#define WEB_UI_ETAG \"\\\"%s\\\"\"\n"
        "static const uint8_t WEB_UI_GZ[] PROGMEM = {\n%s\n};\n" % (len(gz), __import__("hashlib").sha1(gz).hexdigest()[:16], "\n".join(lines))
    )
    old = None
    if os.path.exists(OUT_H):
        with open(OUT_H, encoding="utf-8") as f:
            old = f.read()
    if old != header:
        with open(OUT_H, "w", encoding="utf-8") as f:
            f.write(header)
    print("web ui: %d bytes source, %d minified, %d gzip" % (len(src), len(html), len(gz)))


if __name__ == "__main__":
    main()
