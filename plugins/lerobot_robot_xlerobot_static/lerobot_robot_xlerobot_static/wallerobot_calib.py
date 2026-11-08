"""Joint-angle calibration of this robot (URDF angles, stop clamps, home pose) on top of lerobot's motor units.

Sources (in the xlerobot workspace this plugin is editable-installed from):
    hardware/neck_v0_1/neck_calibration.json      neck_j1..6: zero_tick/sign (J3 zero fixed from the fold contact,
                                                  file only), EEPROM ranges, poses.home (target_ticks)
    hardware/wallerobot_v0_1/arm_calibration.json arms: zero_tick/sign from the standard rest pose (user-verified),
                                                  rest_pose ticks; grippers closed/open ticks

URDF angle: q_rad = sign * (tick - zero_tick) * 2*pi/4096, ticks = Present_Position (Homing_Offset applied).
lerobot DEGREES unit: deg = (tick - mid) * 360/4095, mid = (range_min + range_max)/2 of the lerobot calibration.

ENCODER WRAP (found 2026-11-06): arm shoulder_lift / elbow_flex have range 0..4095 and the 4095 -> 0 wrap lies INSIDE
their travel (rest ~2050, wrap 180 deg away). Readings are unwrapped (wrap_lo window, same as projects/sim_mirror
calib.py). Position mode can't command across the wrap (the motor would turn the long way through the fold stop),
so wrap_problems() lists those joints and the plugin refuses arm torque until their Homing_Offset is re-centred.
"""
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
NECK_CAL = ROOT / "hardware/neck_v0_1/neck_calibration.json"
ARM_CAL = ROOT / "hardware/wallerobot_v0_1/arm_calibration.json"
TICKS = 4096
WRAP_TOL = 150
MAX_TRAVEL = 3000   # ticks (264 deg): generous upper bound of SO101 lift / elbow travel from the fold stop
WRAP_JOINTS = ("shoulder_lift", "elbow_flex")
NECK = [f"neck_j{i}" for i in range(1, 7)]


def load(neck_path: Path = NECK_CAL, arm_path: Path = ARM_CAL) -> dict:
    cal = {}
    if neck_path.is_file():
        neck = json.loads(neck_path.read_text(encoding="utf-8"))
        for n, c in neck["joints"].items():
            cal[n] = {**c, "type": "revolute"}
        home = neck.get("poses", {}).get("home", {})
        for n, t in home.get("target_ticks", {}).items():
            cal[n]["home_tick"] = t
    if arm_path.is_file():
        arms = json.loads(arm_path.read_text(encoding="utf-8"))
        rest = arms.get("rest_pose", {}).get("ticks", {})
        for n, c in arms["joints"].items():
            cal[n] = {**c, "rest_tick": rest.get(n)}
            if n.endswith(WRAP_JOINTS) and rest.get(n) is not None:
                cal[n]["wrap_lo"] = rest[n] - WRAP_TOL if c["sign"] < 0 else rest[n] + WRAP_TOL - (TICKS - 1)
    return cal


def unwrap(c: dict, tick: float) -> float:
    return c["wrap_lo"] + (tick - c["wrap_lo"]) % TICKS if "wrap_lo" in c else tick


def wrap_problems(cal: dict) -> list[str]:
    """Joints whose physical travel window crosses the encoder wrap (0/4095) -> position mode is unsafe for them."""
    # travel starts at wrap_lo (fold stop side) and is < MAX_TRAVEL ticks long -> safe iff it ends before 4095
    return [n for n, c in cal.items() if "wrap_lo" in c and not (0 <= c["wrap_lo"] <= TICKS - 1 - MAX_TRAVEL)]


def tick_to_rad(c: dict, tick: float) -> float:
    if c["type"] == "gripper":
        f = (tick - c["closed_tick"]) / (c["open_tick"] - c["closed_tick"])
        return c["closed_rad"] + f * (c["open_rad"] - c["closed_rad"])
    return c["sign"] * (unwrap(c, tick) - c["zero_tick"]) * 2 * math.pi / TICKS


def stop_tick_limits(name: str, c: dict) -> tuple[int | None, int | None]:
    """(min_tick, max_tick) a goal may have so it never pushes a joint resting on a mechanical stop further into it.

    neck: the calibrated EEPROM range (J2/J3 rest just past it; the motor clamps there too).
    arms shoulder_lift / elbow_flex (no EEPROM limit, 0..4095): the rest-pose tick on the stop side. The stop side
    is the side where the URDF angle grows past the rest angle (lift +187 deg / elbow +180 deg are their fold stops):
    lift sign -1 -> stop = lower ticks; elbow sign +1 -> stop = higher ticks.
    """
    if name in NECK:
        return c["range_min"], c["range_max"]
    if name.endswith(("shoulder_lift", "elbow_flex")) and c.get("rest_tick") is not None:
        return (c["rest_tick"], None) if c["sign"] < 0 else (None, c["rest_tick"])
    return None, None
