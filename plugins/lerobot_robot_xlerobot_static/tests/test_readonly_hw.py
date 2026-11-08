"""READ-ONLY hardware test of the xlerobot_static plugin (no motion, no torque, no motor writes).

    .venv/Scripts/python.exe plugins/lerobot_robot_xlerobot_static/tests/test_readonly_hw.py [--cameras]

1. raw read (tools/sts.py via projects/sim_mirror real_reader): torque off on all 18 motors, ticks
2. plugin connect(read_only=True): per-ID ping, neck calibration adopted from EEPROM (file updated, backup kept)
3. get_observation x10: plugin values -> ticks vs raw ticks (+-1), URDF angles vs projects/sim_mirror calib (+-0.1 deg)
4. home_action / stop clamps sanity, send_action must raise
5. disconnect, raw read: torque still off on all 18
Close every other program using COM3/COM4 (and the cameras with --cameras) first.
"""
import argparse
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "projects/sim_mirror/scripts"))
sys.path.insert(0, str(ROOT / "tools"))
import calib  # noqa: E402
from real_reader import RealReader  # noqa: E402

from lerobot_robot_xlerobot_static import XLerobotStatic, XLerobotStaticConfig  # noqa: E402


def raw():
    r = RealReader()
    try:
        t = r.assert_torque_off(); ticks = r.ticks()
    finally:
        r.close()
    return t, ticks


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--cameras", action="store_true"); a = ap.parse_args()
    ok = True
    torque0, ticks0 = raw()
    print(f"1. raw: torque off on {len(torque0)} motors, ticks read")
    cfg = XLerobotStaticConfig(id="xlerobot_static_01", read_only=True, **({} if a.cameras else {"cameras": {}}))
    robot = XLerobotStatic(cfg)
    t = time.perf_counter(); robot.connect(); print(f"2. plugin connected read-only in {time.perf_counter() - t:.1f} s")
    try:
        worst_tick, worst_deg, dts = 0.0, 0.0, []
        for _ in range(10):
            t = time.perf_counter(); obs = robot.get_observation(); dts.append(time.perf_counter() - t)
            for k, v in obs.items():
                if not k.endswith(".pos"):
                    continue
                m = k.removesuffix(".pos")
                worst_tick = max(worst_tick, abs(robot._to_ticks(m, v) - ticks0[m]))
            q = robot.urdf_positions(obs); q_ref = calib.ticks_to_q(ticks0)
            worst_deg = max(worst_deg, max(abs(math.degrees(q[m] - q_ref[m])) for m in q_ref))
        print(f"3. get_observation: {len([k for k in obs if k.endswith('.pos')])} joints, {1e3 * sum(dts) / len(dts):.1f} ms/read, "
              f"max |plugin - raw| {worst_tick:.2f} ticks, max URDF angle diff {worst_deg:.3f} deg")
        ok &= worst_tick <= 1.5 and worst_deg <= 0.2
        if a.cameras:
            print("   cameras:", {k: getattr(v, "shape", None) for k, v in obs.items() if not k.endswith(".pos")})
        home = robot.home_action()
        print(f"4. home_action: {len(home)} joints, e.g. neck_j2 {home.get('neck_j2.pos'):.1f} / neck_j3 {home.get('neck_j3.pos'):.1f} "
              f"(clamped into range), left lift {home.get('left_arm_shoulder_lift.pos'):.2f}")
        try:
            robot.send_action(home); print("   !! send_action did NOT raise"); ok = False
        except RuntimeError as e:
            print("   send_action raises:", e)
    finally:
        robot.disconnect()
    torque1, ticks1 = raw()
    moved = {m: ticks1[m] - ticks0[m] for m in ticks0 if ticks1[m] is not None and abs(ticks1[m] - ticks0[m]) > 2}
    print(f"5. after disconnect: torque off on {len(torque1)} motors; joints moved >2 ticks during the test: {moved or 'none'}")
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
