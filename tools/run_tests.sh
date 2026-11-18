#!/bin/sh
# Unit tests + MAVLink interoperability (pymavlink) + all simulator scenarios + WebAssembly demo
# + configurator UI tests (Playwright).
#   pip install pymavlink   (for the MAVLink check)
set -e
cd "$(dirname "$0")/.."
python3 tools/check_pins.py
tools/build_host.sh "$@"
./build/tests
if python3 -c "import pymavlink" 2>/dev/null; then python3 tools/test_mavlink.py; else echo "(pymavlink not installed: MAVLink check skipped)"; fi
./build/sim all
if [ -n "$WASI_SDK" ] || ls -d /opt/wasi/wasi-sdk-* >/dev/null 2>&1; then tools/build_wasm.sh; else echo "(wasi-sdk missing: demo build skipped)"; fi
python3 tools/embed_web.py
if command -v node >/dev/null; then node test/web/test_analysis.js && node test/web/test_wasm.js; else echo "(node missing: analyzer tests skipped)"; fi
if python3 -c "import playwright" 2>/dev/null; then
  # one per line: with set -e a failing suite stops the run (an && chain would not)
  python3 test/web/test_ui.py
  python3 test/web/test_configurator.py
  python3 test/web/test_tools.py
  python3 test/web/test_extras.py
  python3 test/web/test_pro.py
  python3 test/web/test_pwa.py
  if [ -f .pio/build/esp32dev/firmware.bin ] && [ -f .pio/build/blackpill_f411ce/firmware.bin ] && python3 -c "import esptool" 2>/dev/null; then
    python3 test/web/test_flasher.py
  else echo "(flasher test needs pio builds + pip install esptool)"; fi
else echo "(playwright missing: UI test skipped)"; fi
