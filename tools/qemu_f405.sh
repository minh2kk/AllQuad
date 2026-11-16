#!/bin/sh
# Boot test of the STM32 firmware (STM32Cube build) in QEMU: the F405 image runs on the
# olimex-stm32-h405 machine with the console on USART1 (QEMU serial0), the CLI must answer.
#   apt install qemu-system-arm ; pio run -e stm32f405_qemu ; tools/qemu_f405.sh
# QEMU models no RCC, USB, I2C, GPIO or DMA: no sensors are found, the boot takes ~20 s (I2C timeouts).
# Second part: "crash test" faults on purpose; after the reset "crash" must show the record kept in RAM.
set -e
cd "$(dirname "$0")/.."
ELF=.pio/build/stm32f405_qemu/firmware.elf
[ -f "$ELF" ] || pio run -e stm32f405_qemu
OUT=$(mktemp)
(sleep 25; printf '\r\nversion\r\n'; sleep 4; printf 'status\r\n'; sleep 4; printf 'crash test\r\n'; sleep 28
 printf '\r\ncrash\r\n'; sleep 4) |
  timeout 75 qemu-system-arm -M olimex-stm32-h405 -kernel "$ELF" -display none -serial stdio -monitor none >"$OUT" 2>&1 || true
cat "$OUT"
if grep -q "AllQuad .* parameters" "$OUT" && grep -q "arming blocked" "$OUT" &&
   grep -q "hard fault (caused the last reset)" "$OUT"; then
  echo "QEMU boot + crash record test: OK"
  rm -f "$OUT"
else
  echo "QEMU boot test: FAILED (no CLI answer)"
  rm -f "$OUT"
  exit 1
fi
