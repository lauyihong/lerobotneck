"""Real raw ticks <-> URDF joint angles for the whole robot (18 joints), from the two calibration files:

    hardware/neck_v0_1/neck_calibration.json     neck_j1..6   (calibrated 2026-11-05, J3 zero fixed from the fold contact)
    hardware/wallerobot_v0_1/arm_calibration.json left/right_arm_*  (seeded, verify with the mirror)

q_rad = sign * (tick - zero_tick) * 2*pi/4096 ; grippers linear closed_tick..open_tick -> closed_rad..open_rad.
Ticks are Present_Position as read from the motor (Homing_Offset already applied).

Arm shoulder_lift / elbow_flex have no EEPROM limit (0..4095) and the encoder wrap (4095 -> 0) lies INSIDE their travel
(rest ~2050, wrap 180 deg away). Readings are unwrapped into a 4096-tick window that starts WRAP_TOL ticks past the
rest tick on the fold-stop side (the joint can't be further into the stop than that).
"""
import json
import math
from pathlib import Path

import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "hardware"))
import robot_config  # noqa: E402

_cfg = robot_config.load()       # calibration file paths: robot.yaml `calibration` (+ $WALLEROBOT_CONFIG override)
NECK_CAL = robot_config.path(_cfg["calibration"]["neck"])
ARM_CAL = robot_config.path(_cfg["calibration"]["arms"])
URDF = ROOT / "hardware/wallerobot_v0_1/urdf/wallerobot_static.urdf"
TICKS = 4096
WRAP_TOL = 150   # ticks past the rest tick (into the fold stop) still accepted as real
WRAP_JOINTS = ("shoulder_lift", "elbow_flex")


def load():
    """{joint: {id, port, type, sign, zero_tick, ... , range_min, range_max}} for all 18 joints, URDF joint names."""
    cal = {}
    for n, c in json.loads(NECK_CAL.read_text(encoding="utf-8"))["joints"].items():
        cal[n] = {**c, "port": "COM3", "type": "revolute"}
    arms = json.loads(ARM_CAL.read_text(encoding="utf-8"))
    cal.update(arms["joints"])
    rest = arms.get("rest_pose", {}).get("ticks", {})
    for n, c in cal.items():
        if n.endswith(WRAP_JOINTS) and rest.get(n) is not None:
            # fold stop = side where q grows past the rest angle: sign -1 -> lower ticks, sign +1 -> higher ticks
            c["wrap_lo"] = rest[n] - WRAP_TOL if c["sign"] < 0 else rest[n] + WRAP_TOL - (TICKS - 1)
    return cal


def unwrap(name, tick):
    c = CAL[name]
    return c["wrap_lo"] + (tick - c["wrap_lo"]) % TICKS if "wrap_lo" in c else tick


CAL = load()
ORDER = [n for p in ("COM3", "COM4") for n, c in sorted(CAL.items(), key=lambda kv: kv[1]["id"]) if c["port"] == p]


def tick_to_rad(name, tick):
    c = CAL[name]
    if c["type"] == "gripper":
        f = (tick - c["closed_tick"]) / (c["open_tick"] - c["closed_tick"])
        return c["closed_rad"] + f * (c["open_rad"] - c["closed_rad"])
    return c["sign"] * (unwrap(name, tick) - c["zero_tick"]) * 2 * math.pi / TICKS


def rad_to_tick(name, q):
    c = CAL[name]
    if c["type"] == "gripper":
        f = (q - c["closed_rad"]) / (c["open_rad"] - c["closed_rad"])
        return int(round(c["closed_tick"] + f * (c["open_tick"] - c["closed_tick"])))
    t = int(round(c["zero_tick"] + c["sign"] * q * TICKS / (2 * math.pi)))
    return t % TICKS if "wrap_lo" in c else t


def ticks_to_q(ticks: dict) -> dict:
    return {n: tick_to_rad(n, t) for n, t in ticks.items() if t is not None}
