#!/usr/bin/env python3
"""Decode an AllQuad blackbox log (serial OpenLog file or ESP32 /blackbox.bin) to CSV.

usage: tools/blackbox_decode.py LOG.bin [out.csv]
Each flight starts with text header lines ("H ...", ending with "H end"), followed by
full records (0xA5, see src/logging/blackbox.h) and, since "H encoding delta1", delta frames
(0xA6) in between, and since "H esc_frame 23" ESC telemetry frames (0xA7: RPM, temperature,
voltage, current per motor, ~50 Hz) which go to a second file LOG_esc.csv. Corrupt records are skipped.
"""
import csv
import struct
import sys

FMT = "<BBI" + "3h" * 3 + "12h" + "4H" + "H" + "4H" + "3h" + "ihii" + "Hh" + "BBBB"
SIZE = struct.calcsize(FMT)
FIELDS = (["time_us"] + [f"gyro_{a}" for a in "rpy"] + [f"gyro_raw_{a}" for a in "rpy"] +
          [f"setpoint_{a}" for a in "rpy"] + [f"{t}_{a}" for t in ("P", "I", "D", "F") for a in "rpy"] +
          [f"motor{i}" for i in range(1, 5)] + ["throttle"] + [f"rc{i}" for i in range(1, 5)] +
          ["roll", "pitch", "yaw", "alt_m", "vz_ms", "pos_n_m", "pos_e_m", "vbat", "current", "mode", "nav_phase",
           "armed", "failsafe", "landed"])
SCALE = {**{f: 0.1 for f in FIELDS if f.startswith(("gyro", "setpoint", "P_", "I_", "D_", "F_"))},
         "roll": 0.1, "pitch": 0.1, "yaw": 0.1, "alt_m": 0.01, "vz_ms": 0.01, "pos_n_m": 0.01, "pos_e_m": 0.01,
         "vbat": 0.01, "current": 0.01, "throttle": 0.001, **{f"motor{i}": 0.001 for i in range(1, 5)}}


# ESC frames (0xA7): sync, len, time_us, rpm/10 x4, temperature x4, voltage x100, current x10, xor
ESC_FMT = "<BBI4H4BHhB"
ESC_SIZE = struct.calcsize(ESC_FMT)
ESC_FIELDS = (["time_us"] + [f"rpm{i}" for i in range(1, 5)] + [f"temp{i}" for i in range(1, 5)] +
              ["esc_voltage", "esc_current"])


def _esc_row(v):
    rpm = ["" if r == 0xFFFF else r * 10 for r in v[3:7]]
    tmp = ["" if t == 255 else t for t in v[7:11]]
    return [v[2]] + rpm + tmp + ["" if not v[11] else v[11] / 100, "" if v[12] == -32768 else v[12] / 10]


# delta frames (0xA6, "H encoding delta1"): changed-field mask + zig-zag varints, see src/logging/blackbox.cpp
FIELD_FMT = ["I"] + ["h"] * 21 + ["H"] * 4 + ["H"] + ["H"] * 4 + ["h"] * 3 + ["i", "h", "i", "i", "H", "h", "B", "B", "B"]
MASK = (len(FIELD_FMT) + 7) // 8
WRAP = {"I": 1 << 32, "H": 1 << 16, "B": 1 << 8}


def expand_delta(data, i, prev, prev_crc):
    """prev: list of field values, prev_crc: checksum byte of that record. Returns (values, crc, length)."""
    if i + 1 >= len(data) or data[i + 1] != prev_crc:
        return None  # built on a record we do not have
    j = i + 2 + MASK
    if j > len(data):
        return None
    vals = list(prev)
    for f, t in enumerate(FIELD_FMT):
        if not data[i + 2 + (f >> 3)] & (1 << (f & 7)):
            continue
        z, shift = 0, 0
        while True:
            if j >= len(data) or shift > 63:
                return None
            b = data[j]
            j += 1
            z |= (b & 0x7F) << shift
            shift += 7
            if not b & 0x80:
                break
        d = (z >> 1) ^ -(z & 1)
        v = vals[f] + d
        vals[f] = v % WRAP[t] if t in WRAP else v
    if j >= len(data):
        return None
    x = 0
    for b in data[i:j]:
        x ^= b
    if x != data[j]:
        return None
    rec = struct.pack(FMT, 0xA5, SIZE, *vals, 0)
    c = 0
    for b in rec[:-1]:
        c ^= b
    return vals, c, j + 1 - i


def decode(data, esc_out=None):
    """flights: [(headers, rows)]; esc_out (list) collects [(flight number, ESC row)]."""
    flights, rows, headers, i, prev, prev_crc = [], None, [], 0, None, 0
    while i < len(data):
        if data.startswith(b"H ", i):
            end = data.find(b"\n", i)
            if end < 0:
                break
            line = data[i + 2:end].decode(errors="replace")
            if line.startswith("AllQuad blackbox"):
                rows = []
                headers = []
                prev = None
                flights.append((headers, rows))
            headers.append(line)
            i = end + 1
            continue
        if data[i] == 0xA5 and i + SIZE <= len(data) and data[i + 1] == SIZE:
            rec = data[i:i + SIZE]
            x = 0
            for b in rec[:-1]:
                x ^= b
            if x == rec[-1] and rows is not None:
                v = struct.unpack(FMT, rec)
                prev = list(v[2:-1])  # fields (sync / len / crc dropped)
                prev_crc = rec[-1]
                rows.append(_row(prev))
                i += SIZE
                continue
        if data[i] == 0xA7 and rows is not None and i + ESC_SIZE <= len(data) and data[i + 1] == ESC_SIZE:
            rec = data[i:i + ESC_SIZE]
            x = 0
            for b in rec[:-1]:
                x ^= b
            if x == rec[-1]:
                if esc_out is not None:
                    esc_out.append((len(flights), _esc_row(struct.unpack(ESC_FMT, rec))))
                i += ESC_SIZE
                continue
        if data[i] == 0xA6 and rows is not None and prev is not None:
            r = expand_delta(data, i, prev, prev_crc)
            if r:
                prev, prev_crc = r[0], r[1]
                rows.append(_row(prev))
                i += r[2]
                continue
            prev = None  # damaged: wait for the next full record
        i += 1
    return flights


def _row(fields):
    vals = list(fields[:-1])  # flags expanded below
    flags = fields[-1]
    return vals + [flags & 1, (flags >> 1) & 1, (flags >> 2) & 1]


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    data = open(sys.argv[1], "rb").read()
    esc = []
    flights = decode(data, esc)
    out = sys.argv[2] if len(sys.argv) > 2 else sys.argv[1].rsplit(".", 1)[0] + ".csv"
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["flight"] + FIELDS)
        for n, (hdr, rows) in enumerate(flights, 1):
            for r in rows:
                w.writerow([n] + [round(val * SCALE.get(name, 1), 4) for name, val in zip(FIELDS, r)])
    total = sum(len(r) for _, r in flights)
    print(f"{len(flights)} flight(s), {total} records -> {out}")
    if esc:
        eout = out.rsplit(".", 1)[0] + "_esc.csv"
        with open(eout, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["flight"] + ESC_FIELDS)
            for n, row in esc:
                w.writerow([n] + row)
        print(f"{len(esc)} ESC frames (RPM, temperature) -> {eout}")
    for n, (hdr, rows) in enumerate(flights, 1):
        dur = (rows[-1][0] - rows[0][0]) / 1e6 if len(rows) > 1 else 0
        print(f"  flight {n}: {len(rows)} records, {dur:.1f} s, " + "; ".join(h for h in hdr if h.startswith("pid")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
