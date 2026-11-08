"""Seed arm_calibration.json (raw tick -> URDF rad, same convention as neck_calibration.json) for the two SO101 arms.

    .venv/Scripts/python.exe hardware/wallerobot_v0_1/seed_arm_calibration.py [--force]

q_rad = sign * (Present_Position - zero_tick) * 2*pi/4096   (Present_Position includes the EEPROM Homing_Offset)
Grippers: q_rad = closed_rad + (tick - closed_tick) / (open_tick - closed_tick) * (open_rad - closed_rad)

Seeds (all UNVERIFIED until the sim-real mirror check, status "seed"):
- ranges / homing = lerobot calibration JSON of the xlerobot_static plugin (arm entries only)
- zero_tick = lerobot DEGREES midpoint + offset_deg from projects/sim_isaac/mapping.yaml
  (lerobot deg = (tick - mid) * 360/4095, sim_rad = sign * rad(deg - offset)). Lift/elbow offsets there were anchored
  on the rest pose (both on their mechanical stop); pan / wrist_flex / wrist_roll are 0 = range midpoint (guess).
"""
import argparse
import json
import time
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
LEROBOT_CAL = Path.home() / ".cache/huggingface/lerobot/calibration/robots/xlerobot_static/xlerobot_static_01.json"
MAPPING = ROOT / "projects/sim_isaac/mapping.yaml"
OUT = HERE / "arm_calibration.json"
GRIPPER = {"closed_rad": 0.0, "open_rad": 1.7}  # sim_isaac mapping.yaml (Jaw 0 .. 1.7 rad); real gripper is PARALLEL


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--force", action="store_true"); a = ap.parse_args()
    if OUT.exists() and not a.force:
        raise SystemExit(f"{OUT} exists (use --force to re-seed; it overwrites verified values)")
    lr = json.loads(LEROBOT_CAL.read_text(encoding="utf-8"))
    mp = yaml.safe_load(MAPPING.read_text(encoding="utf-8"))["joints"]
    joints = {}
    for side in ("left", "right"):
        for m in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"):
            n = f"{side}_arm_{m}"; c = lr[n]
            base = {"id": c["id"], "port": "COM3" if side == "left" else "COM4", "homing_offset": c["homing_offset"],
                    "range_min": c["range_min"], "range_max": c["range_max"], "status": "seed"}
            if m == "gripper":
                joints[n] = {**base, "type": "gripper", "closed_tick": c["range_min"], "open_tick": c["range_max"], **GRIPPER,
                             "source": "lerobot range (+ticks = open, hardware.yaml) -> sim_isaac Jaw 0..1.7 rad"}
                continue
            mid = (c["range_min"] + c["range_max"]) / 2
            off = mp[n].get("offset_deg", 0.0); sign = mp[n].get("sign", 1)
            joints[n] = {**base, "type": "revolute", "sign": sign, "zero_tick": int(round(mid + off * 4095 / 360)),
                         "source": f"lerobot midpoint {mid:.1f} + sim_isaac offset_deg {off}"
                                   + (" (rest-on-stop anchor)" if m in ("shoulder_lift", "elbow_flex") else " (guess)")}
    OUT.write_text(json.dumps({
        "robot": "wallerobot static (dual SO101 arms)",
        "convention": "q_rad = sign * (Present_Position - zero_tick) * 2*pi/4096; grippers linear closed_tick..open_tick "
                      "-> closed_rad..open_rad; Present_Position includes the EEPROM Homing_Offset",
        "sources": {"lerobot_calibration": "~/" + LEROBOT_CAL.relative_to(Path.home()).as_posix(), "mapping": "projects/sim_isaac/mapping.yaml"},
        "date": time.strftime("%Y-%m-%d %H:%M:%S"), "joints": joints}, indent=2), encoding="utf-8")
    print("wrote", OUT)
    for n, c in joints.items():
        print(f"  {n:26s} {c.get('zero_tick', '-')!s:>5} sign {c.get('sign', '-')!s:>2}  range {c['range_min']}..{c['range_max']}")


if __name__ == "__main__":
    main()
