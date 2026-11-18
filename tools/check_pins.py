#!/usr/bin/env python3
"""Pin map check for every board (src/hal/board.h + src/hal/stm32/board_stm32.h), run by run_tests.sh / CI:
  - no pin used twice (motors, SPI, I2C, serial ports, LED, buzzer, ADC, SD card, LED strip ...)
  - no pin the board or the chip keeps for itself (AQ_PINS_RESERVED: USB, SWD, on-board flash / SD slot / PSRAM)
  - STM32: every SERIALn RX / TX pin really is that UART's RX / TX on the alternate function used
A board added or re-wired in board.h fails here instead of on the bench.
    python3 tools/check_pins.py          (exit code 1 on a problem)
"""
import os, re, shutil, subprocess, sys, tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOARDS = ["BOARD_ESP32_DEVKIT", "BOARD_ESP32S3_N16R8", "BOARD_ESP32S3_DEVKIT", "BOARD_ESP32C6_DEVKIT", "BOARD_ESP32P4",
          "BOARD_STM32F411_BLACKPILL", "BOARD_STM32F405", "BOARD_STM32F722", "BOARD_STM32H743_WEACT", "BOARD_STM32H743_DEVEBOX"]
FAMILY = {"BOARD_STM32F411_BLACKPILL": "F4", "BOARD_STM32F405": "F4", "BOARD_STM32F722": "F7",
          "BOARD_STM32H743_WEACT": "H7", "BOARD_STM32H743_DEVEBOX": "H7"}

# UART pins: (instance, role) -> {pin: AF}. From the datasheets' alternate function tables (the pins these
# packages have; extend when a board uses another one).
_F4 = {
    ("USART1", "TX"): {"PA9": 7, "PB6": 7}, ("USART1", "RX"): {"PA10": 7, "PB7": 7},
    ("USART2", "TX"): {"PA2": 7, "PD5": 7}, ("USART2", "RX"): {"PA3": 7, "PD6": 7},
    ("USART3", "TX"): {"PB10": 7, "PC10": 7, "PD8": 7}, ("USART3", "RX"): {"PB11": 7, "PC11": 7, "PD9": 7},
    ("UART4", "TX"): {"PA0": 8, "PC10": 8}, ("UART4", "RX"): {"PA1": 8, "PC11": 8},
    ("UART5", "TX"): {"PC12": 8}, ("UART5", "RX"): {"PD2": 8},
    ("USART6", "TX"): {"PC6": 8, "PG14": 8}, ("USART6", "RX"): {"PC7": 8, "PG9": 8},
}
_F7 = dict(_F4)
_F7.update({("UART7", "TX"): {"PE8": 8, "PF7": 8}, ("UART7", "RX"): {"PE7": 8, "PF6": 8},
            ("UART8", "TX"): {"PE1": 8}, ("UART8", "RX"): {"PE0": 8}})
_H7 = {
    ("USART1", "TX"): {"PA9": 7, "PB6": 7, "PB14": 4}, ("USART1", "RX"): {"PA10": 7, "PB7": 7, "PB15": 4},
    ("USART2", "TX"): {"PA2": 7, "PD5": 7}, ("USART2", "RX"): {"PA3": 7, "PD6": 7},
    ("USART3", "TX"): {"PB10": 7, "PC10": 7, "PD8": 7}, ("USART3", "RX"): {"PB11": 7, "PC11": 7, "PD9": 7},
    ("UART4", "TX"): {"PA0": 8, "PC10": 8, "PD1": 8, "PB9": 8, "PA12": 6},
    ("UART4", "RX"): {"PA1": 8, "PC11": 8, "PD0": 8, "PB8": 8, "PA11": 6},
    ("UART5", "TX"): {"PC12": 8, "PB6": 14, "PB13": 14}, ("UART5", "RX"): {"PD2": 8, "PB5": 14, "PB12": 14},
    ("USART6", "TX"): {"PC6": 7, "PG14": 7}, ("USART6", "RX"): {"PC7": 7, "PG9": 7},
    ("UART7", "TX"): {"PE8": 7, "PF7": 7, "PA15": 11, "PB4": 11}, ("UART7", "RX"): {"PE7": 7, "PF6": 7, "PA8": 11, "PB3": 11},
    ("UART8", "TX"): {"PE1": 8}, ("UART8", "RX"): {"PE0": 8},
}
AF = {"F4": _F4, "F7": _F7, "H7": _H7}


def macros(board, header, extra_dir=None):
    args = ["gcc", "-E", "-dM", "-x", "c++", "-D" + board, header]
    if extra_dir:
        args[1:1] = ["-I", extra_dir]
    out = subprocess.run(args, capture_output=True, text=True, check=True).stdout
    m = {}
    for line in out.splitlines():
        mm = re.match(r"#define (\w+) (.*)$", line)
        if mm:
            m[mm.group(1)] = mm.group(2).strip()
    return m


def value(v):
    v = v.strip()
    if re.fullmatch(r"-?\d+", v):
        return None if int(v) < 0 else v
    if re.fullmatch(r"P[A-K]\d{1,2}", v):
        return v
    return None


def check(board):
    errs = []
    m = macros(board, os.path.join(ROOT, "src", "hal", "board.h"))
    pins = {}  # pin -> list of users
    osd_on = value(m.get("PIN_OSD_CS", "-1")) is not None
    for k, v in sorted(m.items()):
        if not k.startswith("PIN_") or k.endswith("_HINT"):
            continue
        if k.startswith("PIN_OSD_") and not osd_on:
            continue  # no OSD chip fitted: its bus pins are free (often reused for the SD card)
        p = value(v)
        if p is not None:
            pins.setdefault(p, []).append(k)
    for p, users in pins.items():
        if len(users) > 1:
            errs.append("pin %s used by %s" % (p, ", ".join(users)))
    def strmacro(v, depth=0):  # "a" "b" OTHER_MACRO -> "a b ..."
        out = []
        for q, ident in re.findall(r'"([^"]*)"|([A-Za-z_]\w*)', v):
            out.append(q if not ident else strmacro(m.get(ident, ""), depth + 1) if depth < 4 else "")
        return " ".join(out)
    reserved = strmacro(m.get("AQ_PINS_RESERVED", "")).split()
    for p in reserved:
        if p in pins:
            errs.append("pin %s is kept by the board / chip (AQ_PINS_RESERVED) but used by %s" % (p, ", ".join(pins[p])))
    fam = FAMILY.get(board)
    ports = 0
    if fam:
        tmp = tempfile.mkdtemp()
        try:
            shutil.copy(os.path.join(ROOT, "src", "hal", "stm32", "board_stm32.h"), tmp)
            open(os.path.join(tmp, "stm32_mcu.h"), "w").close()  # pin names only, no HAL
            s = macros(board, os.path.join(tmp, "board_stm32.h"))
        finally:
            shutil.rmtree(tmp)
        for n in range(1, 8):
            rx, tx = value(m.get("PIN_SERIAL%d_RX" % n, "-1")), value(m.get("PIN_SERIAL%d_TX" % n, "-1"))
            inst, af = s.get("AQ_SERIAL%d_UART" % n), s.get("AQ_SERIAL%d_AF" % n)
            if (rx or tx) and not inst:
                errs.append("SERIAL%d has pins but no AQ_SERIAL%d_UART in board_stm32.h" % (n, n))
                continue
            if not (rx and tx) or not inst:
                continue
            ports += 1
            for role, pin in (("RX", rx), ("TX", tx)):
                ok = AF[fam].get((inst, role), {})
                if pin not in ok:
                    errs.append("SERIAL%d: %s is not a %s %s pin (%s)" % (n, pin, inst, role, ", ".join(sorted(ok)) or "unknown UART"))
                elif ok[pin] != int(af):
                    errs.append("SERIAL%d: %s %s on %s is AF%d, board_stm32.h says AF%s" % (n, inst, role, pin, ok[pin], af))
        used = [s.get("AQ_SERIAL%d_UART" % n) for n in range(1, 8)
                if s.get("AQ_SERIAL%d_UART" % n) and value(m.get("PIN_SERIAL%d_RX" % n, "-1"))]
        for u in set(used):
            if used.count(u) > 1:
                errs.append("%s drives two serial ports" % u)
    return errs, len(pins), ports, len(reserved)


def main():
    bad = 0
    for b in BOARDS:
        errs, n, ports, nres = check(b)
        print("[%s] %-26s %d pins, %d reserved%s" % ("FAIL" if errs else "PASS", b, n, nres, ", %d UARTs checked" % ports if ports else ""))
        for e in errs:
            print("       " + e)
        bad += len(errs)
    print("\n" + ("pin maps OK" if not bad else "%d problem(s)" % bad))
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
