#!/bin/sh
# Protocol fuzzer under AddressSanitizer + UndefinedBehaviorSanitizer (test/host/fuzz_main.cpp).
# Usage: tools/fuzz.sh [iterations per target, default 20000] [seed] [target]
#   targets: rx msp micolink mavlink gps sensors aqlink cli fourway config
# A long run before a release: tools/fuzz.sh 500000 $(date +%s)
set -e
cd "$(dirname "$0")/.."
mkdir -p build
SRC=$(find src -name '*.cpp' ! -path 'src/bridge/*' ! -path 'src/hal/hal_arduino_common.cpp' ! -path 'src/hal/hal_esp32.cpp' \
      ! -path 'src/hal/hal_stm32.cpp' ! -path 'src/os/os_freertos.cpp' ! -path 'src/sim/*' ! -path 'src/main.cpp')
if [ ! -x build/fuzz ] || [ -n "$(find src test/host/fuzz_main.cpp test/host/hal_host.cpp -newer build/fuzz -name '*.[ch]*' | head -1)" ]; then
  g++ -std=c++17 -O1 -g -fno-omit-frame-pointer -fsanitize=address,undefined -fno-sanitize-recover=undefined \
      -Wall -Wno-unused-parameter -Wno-missing-field-initializers -DALLQUAD_HOST $SRC test/host/hal_host.cpp test/host/fuzz_main.cpp -o build/fuzz
fi
ASAN_OPTIONS=detect_leaks=0:abort_on_error=1 UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1 ./build/fuzz "$@"
