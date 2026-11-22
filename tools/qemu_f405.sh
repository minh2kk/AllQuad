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
(sleep 25; printf '\r\nversion\r\n'; sleep 4; printf 'status\r\n'; sleep 4; printf 'selftest\r\n'; sleep 4; printf 'pins uart\r\n'; sleep 3; printf 'tasks\r\n'; sleep 3; printf 'crash test\r\n'; sleep 28
 printf '\r\ncrash\r\n'; sleep 4) |
  timeout 88 qemu-system-arm -M olimex-stm32-h405 -kernel "$ELF" -display none -serial stdio -monitor none >"$OUT" 2>&1 || true
cat "$OUT"
# stack headroom per task (the "tasks" table, high-water mark measured on request): every task needs
# >= STACK_MIN bytes free, the report goes to the CI log (and the job summary when there is one)
STACK_MIN=${STACK_MIN:-256}
STK=$(tr -d '\r' <"$OUT" | awk '/task +rate_hz +last_us/{on=1;next} on&&NF==7&&$7~/^[0-9]+$/{print $1, $7; next} on{on=0}')
echo "== stack free per task (bytes, minimum $STACK_MIN) =="
echo "$STK"
[ -n "$GITHUB_STEP_SUMMARY" ] && { echo "### STM32F405 (QEMU) stack free per task"; echo; echo "| task | free (B) |"; echo "|---|---|";
  echo "$STK" | awk '{print "| " $1 " | " $2 " |"}'; } >>"$GITHUB_STEP_SUMMARY"
if [ -z "$STK" ]; then echo "QEMU stack report: FAILED (no tasks table)"; rm -f "$OUT"; exit 1; fi
LOW=$(echo "$STK" | awk -v m="$STACK_MIN" '$2 < m {print}')
if [ -n "$LOW" ]; then echo "QEMU stack report: FAILED, too little stack:"; echo "$LOW"; rm -f "$OUT"; exit 1; fi
if grep -q "AllQuad .* parameters" "$OUT" && grep -q "arming blocked" "$OUT" &&
   grep -q "SERIAL1 RX USART2" "$OUT" &&
   grep -q "hard fault (caused the last reset)" "$OUT"; then
  echo "QEMU boot + crash record test: OK"
  rm -f "$OUT"
else
  echo "QEMU boot test: FAILED (no CLI answer)"
  rm -f "$OUT"
  exit 1
fi
