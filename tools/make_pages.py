#!/usr/bin/env python3
"""Builds the folder to publish on GitHub Pages (or any static web host):
    pages/index.html                  the configurator (build/web/AllQuad-Configurator.html)
    pages/firmware/*.bin              firmware of every board that was built (.pio/build/<env>/)
    pages/firmware/manifest.json      list + size + SHA-256: the page's flasher offers these files
                                      ("Bản phát hành trên trang này") and checks them after download
Run after building:  tools/embed_web.py ; pio run -e <env> ... ; python3 tools/make_pages.py [out_dir]
Boards that were not built are left out (and listed)."""
import hashlib, json, os, re, shutil, sys, time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "pages")
PAGE = os.path.join(ROOT, "build", "web", "AllQuad-Configurator.html")

# env, image (app = firmware.bin, full = firmware.factory.bin), published name, target, label,
# regex on the board name the FC reports (pre-selects the entry), note shown after the download
BOARDS = [
    ("esp32s3_n16r8", "app", "AllQuad-esp32s3-n16r8-app.bin", "esp", "ESP32-S3 N16R8 (giữ cấu hình)", "ESP32-S3-N16R8", ""),
    ("esp32s3_n16r8", "full", "AllQuad-esp32s3-n16r8-full.bin", "esp", "ESP32-S3 N16R8, file gộp (board trống / đổi phân vùng)", "",
     "File gộp ghi cả bảng phân vùng: cấu hình về mặc định (nhớ sao lưu)."),
    ("esp32dev", "full", "AllQuad-esp32dev-full.bin", "esp", "ESP32 DevKit (WROOM-32), file gộp", "ESP32-DevKit",
     "Từ v2.1 ô firmware 4 MB lớn hơn: phải nạp file gộp này một lần, cấu hình về mặc định (nhớ sao lưu)."),
    ("esp32s3", "app", "AllQuad-esp32s3-app.bin", "esp", "ESP32-S3 DevKitC (giữ cấu hình)", "ESP32-S3-DevKitC", ""),
    ("esp32c6", "app", "AllQuad-esp32c6-app.bin", "esp", "ESP32-C6 DevKitC (giữ cấu hình)", "ESP32-C6", ""),
    ("esp32p4", "app", "AllQuad-esp32p4-app.bin", "esp", "ESP32-P4 (giữ cấu hình)", "ESP32-P4", ""),
    ("bridge_esp32c3", "full", "AllQuad-bridge-esp32c3-full.bin", "esp", "Cầu WiFi ESP32-C3 (không phải FC)", "", ""),
    ("stm32h743_devebox", "app", "AllQuad-stm32h743_devebox.bin", "stm", "STM32H743 DevEBox", "DevEBox", ""),
    ("stm32h743_weact", "app", "AllQuad-stm32h743_weact.bin", "stm", "STM32H743 WeAct", "STM32H743-WeAct", ""),
    ("stm32f405", "app", "AllQuad-stm32f405.bin", "stm", "STM32F405", "STM32F405", ""),
    ("stm32f722", "app", "AllQuad-stm32f722.bin", "stm", "STM32F722", "STM32F722", ""),
    ("blackpill_f411ce", "app", "AllQuad-blackpill_f411ce.bin", "stm", "STM32F411 Black Pill", "F411", ""),
]


def version():
    with open(os.path.join(ROOT, "src", "app", "app.h"), encoding="utf-8") as f:
        m = re.search(r'VERSION = "([^"]+)"', f.read())
    return m.group(1) if m else "?"


def main():
    if not os.path.exists(PAGE):
        sys.exit("build/web/AllQuad-Configurator.html missing: run python3 tools/embed_web.py")
    fw_dir = os.path.join(OUT, "firmware")
    os.makedirs(fw_dir, exist_ok=True)
    shutil.copyfile(PAGE, os.path.join(OUT, "index.html"))
    files, missing = [], []
    for env, kind, name, target, label, match, note in BOARDS:
        src = os.path.join(ROOT, ".pio", "build", env, "firmware.factory.bin" if kind == "full" else "firmware.bin")
        if not os.path.exists(src):
            missing.append("%s (%s)" % (env, kind))
            continue
        data = open(src, "rb").read()
        if target == "stm" and len(data) > 0x60000:
            sys.exit("%s: %d bytes, larger than the flasher accepts" % (env, len(data)))
        with open(os.path.join(fw_dir, name), "wb") as f:
            f.write(data)
        e = {"file": name, "name": label, "target": target, "size": len(data), "sha256": hashlib.sha256(data).hexdigest()}
        if match:
            e["match"] = match
        if note:
            e["note"] = note
        files.append(e)
    manifest = {"version": version(), "date": time.strftime("%Y-%m-%d"), "files": files}
    with open(os.path.join(fw_dir, "manifest.json"), "w", encoding="utf-8") as f:
        json.dump(manifest, f, ensure_ascii=False, indent=1)
    # GitHub Pages: serve the files as they are (no Jekyll processing)
    open(os.path.join(OUT, ".nojekyll"), "w").close()
    print("pages in %s: index.html + %d firmware files (v%s)" % (OUT, len(files), manifest["version"]))
    if missing:
        print("not built, left out: " + ", ".join(missing))


if __name__ == "__main__":
    main()
