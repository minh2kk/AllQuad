#!/bin/sh
# Builds the firmware + simulator as WebAssembly (build/web/allquad.wasm) for the configurator's
# in-browser demo. Needs wasi-sdk (https://github.com/WebAssembly/wasi-sdk), e.g. in /opt/wasi:
#   WASI_SDK=/opt/wasi/wasi-sdk-25.0-x86_64-linux tools/build_wasm.sh
set -e
cd "$(dirname "$0")/.."
W=${WASI_SDK:-$(ls -d /opt/wasi/wasi-sdk-* 2>/dev/null | head -1)}
[ -x "$W/bin/clang++" ] || { echo "wasi-sdk not found (set WASI_SDK)"; exit 1; }
mkdir -p build/web
SRC=$(find src -name '*.cpp' ! -path 'src/bridge/*' ! -path 'src/hal/hal_arduino_common.cpp' ! -path 'src/hal/hal_esp32.cpp' \
      ! -path 'src/hal/hal_stm32.cpp' ! -path 'src/os/os_freertos.cpp' ! -path 'src/sim/*' ! -path 'src/main.cpp')
"$W/bin/clang++" --target=wasm32-wasip1 --sysroot="$W/share/wasi-sysroot" -std=c++17 -O2 -fno-exceptions \
  -DALLQUAD_HOST -DAQ_WASM -D_WASI_EMULATED_SIGNAL -Wno-unused-command-line-argument \
  $SRC test/host/hal_host.cpp src/sim/sim_main.cpp -o build/web/allquad.wasm \
  -mexec-model=reactor -Wl,--strip-all -Wl,-z,stack-size=1048576
ls -l build/web/allquad.wasm
