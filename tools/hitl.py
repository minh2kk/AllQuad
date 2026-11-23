#!/usr/bin/env python3
"""AllQuad hardware in the loop: the real flight controller flies a simulated quad.

The board runs its real firmware (tasks, timing, filters, estimators, failsafes) while this script
simulates the craft on the PC and feeds the board gyro, accelerometer, barometer, compass, GPS (and
optionally the rangefinder, RC channels and battery) over USB. The board answers with its motor
outputs; the motors themselves stay stopped (nothing spins: props can stay on, no battery needed).

usage:
  python tools/hitl.py COM5                  fly with your own radio (real receiver on the board)
  python tools/hitl.py COM5 --auto           scripted check flight with RC from the PC: arm, climb,
                                             ALTHOLD, POSHOLD, LAND, disarm -> PASS / FAIL
  python tools/hitl.py --exec "build/sim serve" --auto     the same against the PC build (CI test)
options: --wind 2 (m/s), --log flight.csv, --seconds 60, --mass / --tmax (frame), --home LAT,LON
The default switches of AllQuad are used by --auto (AUX1 arm, AUX2 ANGLE / ALTHOLD, AUX3 POSHOLD,
AUX4 LAND). The board restarts by itself 1 s after this script stops (disarmed): back to real sensors.
needs: pip install pyserial (not for --exec)
"""
import argparse
import math
import os
import random
import struct
import subprocess
import sys
import time

CMD = 0x4160
G = 9.80665
MODES = ["ACRO", "ANGLE", "HORIZON", "ALTHOLD", "POSHOLD", "RTH", "MISSION", "LAND", "FAILSAFE", "TURTLE",
         "CRUISE", "AUTOTUNE", "GUIDED", "SMART_RTL"]
F_GPS, F_RANGE, F_RC, F_BATTERY, F_EXIT = 1, 2, 4, 8, 0x80


# ------------------------------------------------------------------ MSP v2
def crc8(data):
    c = 0
    for b in data:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0xD5) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def msp2(cmd, payload):
    body = struct.pack("<BHH", 0, cmd, len(payload)) + payload
    return b"$X<" + body + bytes([crc8(body)])


class Msp2Reader:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(b"$X")
            if i < 0:
                self.buf = self.buf[-1:]
                return out
            del self.buf[:i]
            if len(self.buf) < 8:
                return out
            ln = self.buf[6] | (self.buf[7] << 8)
            if ln > 512:
                del self.buf[:2]
                continue
            if len(self.buf) < 9 + ln:
                return out
            body = bytes(self.buf[3:8 + ln])
            ok = crc8(body) == self.buf[8 + ln]
            direction = chr(self.buf[2])
            cmd = self.buf[4] | (self.buf[5] << 8)
            payload = bytes(self.buf[8:8 + ln])
            del self.buf[:9 + ln]
            if ok:
                out.append((direction, cmd, payload))


# ------------------------------------------------------------------ math
def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)


def qrot(q, v):  # body -> earth
    w, x, y, z = q
    tx, ty, tz = 2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0])
    return (v[0] + w * tx + y * tz - z * ty, v[1] + w * ty + z * tx - x * tz, v[2] + w * tz + x * ty - y * tx)


def qinv(q):
    return (q[0], -q[1], -q[2], -q[3])


def euler(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


# ------------------------------------------------------------------ the craft (same model as src/sim)
class Quad:
    X, Y, D = (-1, 1, -1, 1), (1, 1, -1, -1), (-1, 1, 1, -1)  # mixer order: rear-right, front-right, rear-left, front-left

    def __init__(self, mass=0.55, tmax=3.6, arm=0.08, yaw0=30.0):
        self.mass, self.tmax, self.arm, self.kyaw = mass, tmax, arm, 0.016
        self.inertia = (0.0026 * mass / 0.55, 0.0028 * mass / 0.55, 0.0048 * mass / 0.55)
        self.tau, self.drag = 0.03, 0.25 * mass / 0.55
        self.pos, self.vel, self.w, self.acc = [0.0] * 3, [0.0] * 3, [0.0] * 3, [0.0, 0.0, 0.0]
        h = math.radians(yaw0) / 2
        self.q = (math.cos(h), 0.0, 0.0, math.sin(h))
        self.m = [0.0] * 4
        self.ground = True
        self.wind = (0.0, 0.0)

    def step(self, cmd, armed, dt):
        thrust, tx, ty, tz = 0.0, 0.0, 0.0, 0.0
        for i in range(4):
            c = max(0.0, min(1.0, cmd[i])) if armed else 0.0
            self.m[i] += (c - self.m[i]) * (dt / (self.tau + dt))
            sp = self.m[i]
            t = self.tmax * (sp * sp * 0.6 + sp * 0.4)
            thrust += t
            tx += -self.Y[i] * self.arm * t
            ty += self.X[i] * self.arm * t
            tz += self.D[i] * self.kyaw * t
        I, w = self.inertia, self.w
        Iw = (I[0] * w[0], I[1] * w[1], I[2] * w[2])
        cr = (w[1] * Iw[2] - w[2] * Iw[1], w[2] * Iw[0] - w[0] * Iw[2], w[0] * Iw[1] - w[1] * Iw[0])
        for k, tq in enumerate((tx, ty, tz)):
            w[k] += (tq - cr[k]) / I[k] * dt
        qd = qmul(self.q, (0.0, w[0], w[1], w[2]))
        q = tuple(self.q[k] + 0.5 * qd[k] * dt for k in range(4))
        n = math.sqrt(sum(c * c for c in q))
        self.q = tuple(c / n for c in q)
        f = qrot(self.q, (0.0, 0.0, -thrust))
        wind = (0.0, 0.0) if self.ground else self.wind
        F = [f[0] - self.vel[0] * self.drag + wind[0] * self.drag, f[1] - self.vel[1] * self.drag + wind[1] * self.drag,
             f[2] + self.mass * G - self.vel[2] * self.drag]
        self.acc = [F[k] / self.mass for k in range(3)]
        for k in range(3):
            self.vel[k] += self.acc[k] * dt
            self.pos[k] += self.vel[k] * dt
        self.ground = self.pos[2] >= 0
        if self.ground:
            self.pos[2] = 0.0
            if self.vel[2] > 0:
                self.vel = [0.0, 0.0, 0.0]
            self.vel[0] *= 0.9
            self.vel[1] *= 0.9
            if self.acc[2] > 0:
                self.acc = [0.0, 0.0, 0.0]
            self.w = [c * 0.9 for c in self.w]

    def specific_force_body(self):
        return qrot(qinv(self.q), (self.acc[0], self.acc[1], self.acc[2] - G))


# ------------------------------------------------------------------ link
class Link:
    def __init__(self, port=None, cmd=None):
        self.proc = self.ser = None
        if cmd:
            self.proc = subprocess.Popen(cmd, shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
            os.set_blocking(self.proc.stdout.fileno(), False)
        else:
            import serial  # pip install pyserial
            self.ser = serial.Serial(port, 921600, timeout=0)

    def write(self, b):
        if self.proc:
            self.proc.stdin.write(b)
        else:
            self.ser.write(b)

    def read(self):
        if self.proc:
            try:
                return self.proc.stdout.read(65536) or b""
            except BlockingIOError:
                return b""
        return self.ser.read(self.ser.in_waiting or 1)

    def close(self):
        if self.proc:
            self.proc.stdin.close()
            self.proc.wait(timeout=5)
        else:
            self.ser.close()


def ned_to_lla(n, e, lat0, lon0):
    lat = lat0 + math.degrees(n / 6378137.0)
    lon = lon0 + math.degrees(e / (6378137.0 * math.cos(math.radians(lat0))))
    return lat, lon


# ------------------------------------------------------------------ the flight
def main():
    ap = argparse.ArgumentParser(description="AllQuad hardware in the loop", formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__)
    ap.add_argument("port", nargs="?", help="serial port of the board (COM5, /dev/ttyACM0)")
    ap.add_argument("--exec", dest="cmd", help="run this program instead of a serial port (its stdin/stdout is the console)")
    ap.add_argument("--auto", action="store_true", help="scripted check flight, RC from the PC")
    ap.add_argument("--rc", action="store_true", help="RC from the PC (sticks centred, disarmed) instead of the board's receiver")
    ap.add_argument("--seconds", type=float, default=0, help="stop after this long (default: --auto 50 s, else until Ctrl+C)")
    ap.add_argument("--wind", type=float, default=0.0, help="steady wind, m/s from the west")
    ap.add_argument("--mass", type=float, default=0.55)
    ap.add_argument("--tmax", type=float, default=3.6, help="total static thrust at full throttle per motor (N)")
    ap.add_argument("--home", default="21.0285,105.8542")
    ap.add_argument("--rate", type=float, default=500.0, help="sensor frames per second")
    ap.add_argument("--log", help="CSV of the flight (truth and the board's estimate)")
    a = ap.parse_args()
    if not a.port and not a.cmd:
        ap.error("give the serial port of the board, or --exec")
    lat0, lon0 = (float(v) for v in a.home.split(","))
    link = Link(a.port, a.cmd)
    quad = Quad(a.mass, a.tmax)
    quad.wind = (0.0, a.wind)
    mag_e = (0.22, 0.0, 0.42)
    rnd = random.Random(1)
    reader = Msp2Reader()
    rc = [1500, 1500, 1000, 1500, 1000, 1500, 1000, 1000]  # AETR: roll, pitch, throttle, yaw, AUX1..4
    use_rc = a.rc or a.auto
    duration = a.seconds or (50.0 if a.auto else 1e9)
    motors, armed, mode, nav, fs = [0.0] * 4, False, 0, 0, 0
    est = (0.0, 0.0, 0.0, 0.0)
    loop_hz = 0
    sent = got = 0
    lat_sum, lat_n = 0.0, 0
    sent_at = {}
    seq = 0
    t0 = time.perf_counter()
    t_phys = 0.0
    next_gps = 0.0
    log = open(a.log, "w") if a.log else None
    if log:
        log.write("t,armed,mode,alt,n,e,roll,pitch,yaw,est_roll,est_pitch,est_yaw,est_alt,m1,m2,m3,m4\n")
    # check-flight measurements
    hold_alt, hold_n, hold_e = [], None, None
    att_err = 0.0
    drift = 0.0
    landed_t = None
    armed_seen = False
    last_print = -1.0
    started = False
    try:
        while True:
            now = time.perf_counter() - t0
            if now > duration:
                break
            # ---- answers ----
            for d, cmd, p in reader.feed(link.read()):
                if cmd != CMD or d != ">" or len(p) < 36:
                    if cmd == CMD and d == "!":
                        print("the board refused HITL (armed?)")
                        return 2
                    continue
                started = True
                got += 1
                s = struct.unpack_from("<I", p)[0]
                if s in sent_at:
                    lat_sum += now - sent_at.pop(s)
                    lat_n += 1
                motors = list(struct.unpack_from("<4f", p, 4))
                armed, mode, nav, fs = bool(p[20]), p[21], p[22], p[23] & 1
                r, pi, y = struct.unpack_from("<3h", p, 24)
                alt_cm = struct.unpack_from("<i", p, 30)[0]
                loop_hz = struct.unpack_from("<H", p, 34)[0]
                est = (r / 10, pi / 10, y / 10, alt_cm / 100)
            # ---- physics up to now (1 kHz) ----
            while t_phys < now:
                quad.step(motors, armed, 0.001)
                t_phys += 0.001
            # ---- the pilot (--auto) ----
            if a.auto:
                t = now
                rc[4] = 2000 if 4.0 <= t < 44.0 else 1000            # AUX1 arm
                rc[2] = 1000 if t < 4.5 else 1580 if t < 7.0 else 1500  # throttle: climb, then centre (ALTHOLD)
                rc[5] = 2000 if t >= 7.0 else 1500                    # AUX2 ALTHOLD
                rc[6] = 1500 if t >= 10.0 else 1000                   # AUX3 POSHOLD
                rc[7] = 2000 if t >= 28.0 else 1000                   # AUX4 LAND
                rc[1] = 1650 if 18.0 <= t < 19.5 else 1500            # a short pitch push in POSHOLD
            # ---- sensors to the board ----
            seq += 1
            g = [quad.w[k] + rnd.gauss(0, 0.004) for k in range(3)]
            fb = quad.specific_force_body()
            acc = [fb[k] + rnd.gauss(0, 0.3) for k in range(3)]
            alt = -quad.pos[2]
            pa = 101325.0 * (1 - (alt + rnd.gauss(0, 0.08)) / 44330.0) ** 5.255
            mb = qrot(qinv(quad.q), mag_e)
            mb = [mb[k] + rnd.gauss(0, 0.003) for k in range(3)]
            flags = F_BATTERY | (F_RC if use_rc else 0)
            lat = lon = 0.0
            if now >= next_gps:
                next_gps = now + 0.1
                flags |= F_GPS
                lat, lon = ned_to_lla(quad.pos[0] + rnd.gauss(0, 0.3), quad.pos[1] + rnd.gauss(0, 0.3), lat0, lon0)
            vbat = 16.4 - 0.004 * now
            cur = 0.5 + 18.0 * sum(quad.m) / 4
            frame = struct.pack("<I3f3ff3fBBH", seq, *g, *acc, pa, *mb, flags, 14, 0)
            frame += struct.pack("<ii5ff", int(lat * 1e7), int(lon * 1e7), 10.0 + alt, quad.vel[0] + rnd.gauss(0, 0.1),
                                 quad.vel[1] + rnd.gauss(0, 0.1), quad.vel[2], 1.2, 0.0)
            frame += struct.pack("<8H", *rc) + struct.pack("<ffI", vbat, cur, 0)
            link.write(msp2(CMD, frame))
            sent += 1
            sent_at[seq] = now
            if len(sent_at) > 2000:
                sent_at.clear()
            # ---- measurements / display ----
            tr, tp, ty = (math.degrees(v) for v in euler(quad.q))
            if armed:
                armed_seen = True
            if started and armed and now > 8:
                att_err = max(att_err, abs(est[0] - tr), abs(est[1] - tp))
            if a.auto and 12.0 <= now < 18.0:
                hold_alt.append(alt)
            if a.auto and abs(now - 12.0) < 0.003 and hold_n is None:
                hold_n, hold_e = quad.pos[0], quad.pos[1]
            if a.auto and hold_n is not None and 12.0 <= now < 28.0:
                drift = max(drift, math.hypot(quad.pos[0] - hold_n, quad.pos[1] - hold_e))
            if a.auto and now > 28 and landed_t is None and not armed and armed_seen:
                landed_t = now
            if log and seq % 10 == 0:
                log.write(f"{now:.3f},{int(armed)},{mode},{alt:.2f},{quad.pos[0]:.2f},{quad.pos[1]:.2f},{tr:.1f},{tp:.1f},{ty:.1f},"
                          f"{est[0]:.1f},{est[1]:.1f},{est[2]:.1f},{est[3]:.2f}," + ",".join(f"{m:.3f}" for m in motors) + "\n")
            if now - last_print >= 1.0:
                last_print = now
                rate = got / max(now, 1e-3)
                lat_ms = lat_sum / lat_n * 1000 if lat_n else 0
                print(f"t={now:5.1f}  {MODES[mode] if mode < len(MODES) else mode:8s} {'ARMED' if armed else 'disarmed':8s}"
                      f" alt {alt:5.1f} m (est {est[3]:5.1f})  N {quad.pos[0]:6.1f} E {quad.pos[1]:6.1f}  roll {tr:5.1f} (est {est[0]:5.1f})"
                      f"  link {rate:4.0f}/s {lat_ms:4.1f} ms  fc loop {loop_hz} Hz" + ("  FAILSAFE" if fs else ""), flush=True)
            # ---- pace ----
            nxt = seq / a.rate
            d = nxt - (time.perf_counter() - t0)
            if d > 0:
                time.sleep(d)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            link.write(msp2(CMD, struct.pack("<I", seq + 1) + bytes(40) + bytes([F_EXIT]) + bytes(63)))  # leave HITL (restart)
        except OSError:
            pass
        link.close()
        if log:
            log.close()
    if not a.auto:
        return 0
    # ---- verdict ----
    rate = got / duration
    lat_ms = lat_sum / lat_n * 1000 if lat_n else 999
    alt_sd = 0.0
    if hold_alt:
        m = sum(hold_alt) / len(hold_alt)
        alt_sd = math.sqrt(sum((v - m) ** 2 for v in hold_alt) / len(hold_alt))
    checks = [
        ("board answers HITL frames", rate > 0.6 * a.rate, f"{rate:.0f} answers/s, latency {lat_ms:.1f} ms"),
        ("armed and flew", armed_seen and bool(hold_alt) and max(hold_alt) > 1.0, f"hover height {sum(hold_alt) / max(len(hold_alt), 1):.1f} m"),
        ("ALTHOLD steady", bool(hold_alt) and alt_sd < 0.5, f"height spread {alt_sd:.2f} m"),
        ("POSHOLD kept its place (with a stick push)", drift < 4.0, f"max {drift:.1f} m"),
        ("attitude estimate follows the simulated craft", att_err < 5.0, f"max error {att_err:.1f} deg"),
        ("LAND: down and disarmed", landed_t is not None, f"at {landed_t:.1f} s" if landed_t else "not disarmed"),
    ]
    for name, ok, detail in checks:
        print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}")
    bad = sum(1 for _, ok, _ in checks if not ok)
    print(f"{len(checks) - bad}/{len(checks)} HITL checks passed")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
