"""Set the arm zero_ticks in arm_calibration.json from the STANDARD REST POSE (READ-ONLY on the robot).

    .venv/Scripts/python.exe hardware/wallerobot_v0_1/calib_arm_rest.py              # reads the encoders now
    .venv/Scripts/python.exe hardware/wallerobot_v0_1/calib_arm_rest.py --pose p.json # or use a saved read ({"ticks": ...})

Put both arms in the standard rest pose first (user, 2026-11-05 photo projects/sim_mirror/outputs/photo_full_20261105.jpg):
lift and elbow folded onto their mechanical stops, upper arm lying backwards, gripper pointing forward-down in front of
the base, wrist camera on top, gripper closed. In this URDF that pose is REST_Q below (found by rendering against the
photo; lift 187 = same SO101 fold stop as neck J2 rest +187.6, elbow 180 = model upper/lower-arm contact 180.75,
pan 0 = arm facing forward (mount yaw fixed 2026-11-05), roll 90 = camera on top, as the upstream ManiSkill rest).
Then zero_tick = tick - sign * q * 4096 / 2pi.

SIGNS: lift -1 / elbow +1 from projects/handover_test/outputs/ik_calib (right arm: leaving the rest, lift ticks go up,
elbow ticks go down). pan / wrist_flex / wrist_roll +1 verified by the user in the mirror (2026-11-05).
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
ARM_CAL = HERE / "arm_calibration.json"
REST_Q = {"shoulder_pan": 0.0, "shoulder_lift": 187.0, "elbow_flex": 180.0,
          "wrist_flex": 50.0, "wrist_roll": 90.0}
GRIPPER_CLOSED_DEG = -22.0  # stock SO101 Moving_Jaw touches Fixed_Jaw at -22.5 deg in the model (upstream limit -21.5)
SIGN = {"shoulder_pan": 1, "shoulder_lift": -1, "elbow_flex": 1, "wrist_flex": 1, "wrist_roll": 1}
SIGN_STATUS = {"shoulder_pan": "user-verified (mirror)", "shoulder_lift": "handover_test data + mirror",
               "elbow_flex": "handover_test data + mirror", "wrist_flex": "user-verified (mirror)", "wrist_roll": "user-verified (mirror)"}


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--pose", type=Path); a = ap.parse_args()
    if a.pose:
        ticks = json.loads(a.pose.read_text())["ticks"]
    else:
        sys.path.insert(0, str(ROOT / "projects/sim_mirror/scripts"))
        from real_reader import RealReader
        r = RealReader()
        try:
            r.assert_torque_off(); ticks = r.ticks(); r.assert_torque_off()
        finally:
            r.close()
    d = json.loads(ARM_CAL.read_text(encoding="utf-8"))
    for side in ("left", "right"):
        for j, q in REST_Q.items():
            n = f"{side}_arm_{j}"; c = d["joints"][n]; t = ticks[n]
            q = q[side] if isinstance(q, dict) else q
            old = c["zero_tick"]
            c["sign"] = SIGN[j]; c["zero_tick"] = int(round(t - SIGN[j] * q * 4096 / 360))
            c["status"] = "rest_pose"; c["sign_status"] = SIGN_STATUS[j]
            c["source"] = f"calib_arm_rest.py {time.strftime('%Y-%m-%d %H:%M')}: rest tick {t} = {q:+.1f} deg (was zero {old})"
            print(f"  {n:24s} tick {t:5d} = {q:+7.1f} deg  sign {SIGN[j]:+d} ({SIGN_STATUS[j]})  zero {old} -> {c['zero_tick']}")
        # gripper: fully closed in the rest pose -> closed_tick = this reading, closed_rad = model finger contact
        n = f"{side}_arm_gripper"; c = d["joints"][n]; old = c["closed_tick"]
        c["closed_tick"] = ticks[n]; c["closed_rad"] = round(math.radians(GRIPPER_CLOSED_DEG), 4); c["status"] = "rest_pose"
        c["source"] = (f"calib_arm_rest.py {time.strftime('%Y-%m-%d %H:%M')}: fully closed tick {ticks[n]} (was {old}) = "
                       f"model jaw contact {GRIPPER_CLOSED_DEG} deg; open_tick = lerobot range_max -> {c['open_rad']} rad (placeholder)")
        print(f"  {n:24s} tick {ticks[n]:5d} = closed ({GRIPPER_CLOSED_DEG:+.1f} deg)  closed_tick {old} -> {ticks[n]}")
    d["rest_pose"] = {"date": time.strftime("%Y-%m-%d %H:%M:%S"), "ticks": {k: v for k, v in ticks.items() if "_arm_" in k},
                      "q_deg": REST_Q, "photo": "projects/sim_mirror/outputs/photo_full_20261105.jpg"}
    ARM_CAL.write_text(json.dumps(d, indent=2), encoding="utf-8")
    print("updated", ARM_CAL)


if __name__ == "__main__":
    main()
