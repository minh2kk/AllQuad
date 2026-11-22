#!/usr/bin/env python3
"""Builds src/web/ui/i18n.js = tools/i18n/runtime.js + the table tools/i18n/en.json.

Adding text to the page: run  node tools/i18n/collect.js  (lists the Vietnamese strings on screen in the
demo drone, numbers as {n}) and add the new ones to en.json, then this script (embed_web.py runs it)."""
import json
import os

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
HERE = os.path.join(ROOT, "tools", "i18n")
OUT = os.path.join(ROOT, "src", "web", "ui", "i18n.js")


def main():
    with open(os.path.join(HERE, "en.json"), encoding="utf-8") as f:
        d = json.load(f)
    with open(os.path.join(HERE, "runtime.js"), encoding="utf-8") as f:
        rt = f.read()
    table = "const I18N_EN = " + json.dumps(d, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + ";\n"
    out = rt.replace('"use strict";\n', '"use strict";\n' + table, 1)
    old = open(OUT, encoding="utf-8").read() if os.path.exists(OUT) else ""
    if out != old:
        with open(OUT, "w", encoding="utf-8", newline="\n") as f:
            f.write(out)
    print("i18n: %d English entries" % len(d))


if __name__ == "__main__":
    main()
