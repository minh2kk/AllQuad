#!/bin/sh
# Builds the PC simulator (build/sim) and unit tests (build/tests).
# Usage: tools/build_host.sh [extra compiler flags, e.g. -fsanitize=address,undefined]
set -e
cd "$(dirname "$0")/.."
mkdir -p build
SRC=$(find src -name '*.cpp' ! -path 'src/hal/hal_arduino_common.cpp' ! -path 'src/hal/hal_esp32.cpp' \
      ! -path 'src/hal/hal_stm32.cpp' ! -path 'src/os/os_freertos.cpp' ! -path 'src/sim/*' ! -path 'src/main.cpp')
FLAGS="-std=c++17 -O2 -g -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -DALLQUAD_HOST $*"
g++ $FLAGS $SRC test/host/hal_host.cpp src/sim/sim_main.cpp -o build/sim
if [ -f test/host/test_main.cpp ]; then
  g++ $FLAGS $SRC test/host/hal_host.cpp test/host/test_main.cpp -o build/tests
fi
