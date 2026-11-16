#!/usr/bin/env python3
"""Cross-checks the firmware MAVLink implementation against pymavlink (the reference
implementation used by QGroundControl / ArduPilot tooling).

A ground station built with pymavlink sends: PARAM_REQUEST_LIST, PARAM_SET, a full mission
upload (MISSION_COUNT + MISSION_ITEM_INT x N) and COMMAND_LONG arm. The firmware link runs on
the host HAL; every byte it emits is decoded by pymavlink (CRC, CRC_EXTRA, field layout).
Requires: pip install pymavlink ; build/tests (tools/build_host.sh)
"""
import os
import subprocess
import sys
import tempfile

from pymavlink.dialects.v20 import common as mavlink2

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TESTS = os.path.join(ROOT, "build", "tests")


class Sink:
    def __init__(self):
        self.buf = bytearray()

    def write(self, b):
        self.buf += b


def main():
    sink = Sink()
    gcs = mavlink2.MAVLink(sink, srcSystem=255, srcComponent=190)
    gcs.robust_parsing = True

    gcs.heartbeat_send(mavlink2.MAV_TYPE_GCS, mavlink2.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    gcs.param_request_list_send(1, 1)
    gcs.param_set_send(1, 1, b"roll_p", 52.0, mavlink2.MAV_PARAM_TYPE_REAL32)
    gcs.command_long_send(1, 1, mavlink2.MAV_CMD_REQUEST_MESSAGE, 0, 148, 0, 0, 0, 0, 0, 0)
    items = [
        (mavlink2.MAV_CMD_NAV_TAKEOFF, 0, 0, 10.0, 0),
        (mavlink2.MAV_CMD_NAV_WAYPOINT, 210287000, 1058545000, 15.0, 2.0),
        (mavlink2.MAV_CMD_NAV_WAYPOINT, 210290000, 1058550000, 20.0, 0.0),
        (mavlink2.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0.0, 0),
    ]
    gcs.mission_count_send(1, 1, len(items), 0)
    for seq, (cmd, lat, lon, alt, p1) in enumerate(items):
        gcs.mission_item_int_send(1, 1, seq, mavlink2.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, cmd, 0, 1,
                                  p1, 0, 0, 0, lat, lon, alt, 0)
    # rally points (mission_type 2) as QGroundControl's Plan view uploads them, then read one back
    rally = [(210300000, 1058560000, 12.0), (210310000, 1058570000, 0.0)]
    gcs.mission_count_send(1, 1, len(rally), mavlink2.MAV_MISSION_TYPE_RALLY)
    for seq, (lat, lon, alt) in enumerate(rally):
        gcs.mission_item_int_send(1, 1, seq, mavlink2.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                  mavlink2.MAV_CMD_NAV_RALLY_POINT, 0, 1, 0, 0, 0, 0, lat, lon, alt,
                                  mavlink2.MAV_MISSION_TYPE_RALLY)
    gcs.mission_request_list_send(1, 1, mavlink2.MAV_MISSION_TYPE_RALLY)
    gcs.mission_request_int_send(1, 1, 1, mavlink2.MAV_MISSION_TYPE_RALLY)
    gcs.command_long_send(1, 1, mavlink2.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

    with tempfile.TemporaryDirectory() as d:
        fin, fout = os.path.join(d, "in.bin"), os.path.join(d, "out.bin")
        open(fin, "wb").write(bytes(sink.buf))
        res = subprocess.run([TESTS, "mavlink-io", fin, fout], capture_output=True, text=True, check=True)
        out = open(fout, "rb").read()
    state = res.stdout

    # decode everything the firmware sent
    rx = mavlink2.MAVLink(None)
    msgs = []
    bad = 0
    for b in out:
        try:
            m = rx.parse_char(bytes([b]))
        except mavlink2.MAVError:
            bad += 1
            continue
        if m is not None:
            if m.get_type() == "BAD_DATA":
                bad += 1
            else:
                msgs.append(m)
    by = {}
    for m in msgs:
        by.setdefault(m.get_type(), []).append(m)

    results = []

    def check(name, ok, detail=""):
        results.append((name, ok, detail))

    expected_params = int([l for l in state.splitlines() if l.startswith("param_count=")][0].split("=")[1])
    check("all frames decode with valid CRC", bad == 0 and len(msgs) > 0, f"{len(msgs)} msgs, {bad} bad")
    hb = by.get("HEARTBEAT", [])
    check("HEARTBEAT quadrotor", hb and hb[0].type == mavlink2.MAV_TYPE_QUADROTOR, f"{len(hb)} heartbeats")
    for t in ["SYS_STATUS", "ATTITUDE", "GLOBAL_POSITION_INT", "VFR_HUD", "MISSION_CURRENT"]:
        check(f"{t} stream", t in by, f"{len(by.get(t, []))}")
    pv = by.get("PARAM_VALUE", [])
    names = {p.param_id for p in pv}
    check("PARAM list complete", len(names) == expected_params and all(p.param_count == expected_params for p in pv),
          f"{len(names)}/{expected_params} unique")
    rp = [p for p in pv if p.param_id == "roll_p"]
    check("PARAM_SET applied & echoed", "roll_p=52.00" in state and any(abs(p.param_value - 52.0) < 1e-4 for p in rp))
    reqs = [m.seq for m in by.get("MISSION_REQUEST_INT", [])]
    check("mission items requested in order", reqs[:4] == [0, 1, 2, 3], str(reqs))
    acks = by.get("MISSION_ACK", [])
    check("MISSION_ACK accepted", acks and acks[-1].type == mavlink2.MAV_MISSION_ACCEPTED)
    check("mission stored", "mission_count=4" in state and "lat=210287000" in state and "p1=2.00" in state)
    rally_acks = [a for a in acks if a.mission_type == mavlink2.MAV_MISSION_TYPE_RALLY]
    check("rally points accepted & stored", rally_acks and rally_acks[-1].type == 0 and "rally_count=2" in state
          and "rally 1 lat=210310000" in state)
    rc = [c for c in by.get("MISSION_COUNT", []) if c.mission_type == mavlink2.MAV_MISSION_TYPE_RALLY]
    ri = [i for i in by.get("MISSION_ITEM_INT", []) if i.mission_type == mavlink2.MAV_MISSION_TYPE_RALLY]
    check("rally points download", rc and rc[0].count == 2 and ri and ri[0].seq == 1 and ri[0].x == 210310000,
          f"{len(rc)} counts, {len(ri)} items")
    av = by.get("AUTOPILOT_VERSION", [])
    check("AUTOPILOT_VERSION on request",
          av and av[0].capabilities & mavlink2.MAV_PROTOCOL_CAPABILITY_MISSION_INT and
          av[0].capabilities & mavlink2.MAV_PROTOCOL_CAPABILITY_MAVLINK2 and
          av[0].capabilities & mavlink2.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY)
    cacks = {(a.command, a.result) for a in by.get("COMMAND_ACK", [])}
    check("COMMAND_ACK arm accepted", (mavlink2.MAV_CMD_COMPONENT_ARM_DISARM, 0) in cacks, str(cacks))
    check("arm request forwarded to commander", "arm_request=1" in state)

    fails = 0
    for name, ok, detail in results:
        print(f"[{'PASS' if ok else 'FAIL'}] {name:40s} {detail}")
        fails += 0 if ok else 1
    print(f"\n{len(results) - fails}/{len(results)} MAVLink interoperability checks passed")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
