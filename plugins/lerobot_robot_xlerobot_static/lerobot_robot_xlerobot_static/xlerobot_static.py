"""XLeRobot (no base) of this workspace: two SO101 arms + 6-DoF SO101-based neck (D435i) on two Feetech buses.

    COM3 (port_left):  left_arm_{shoulder_pan,shoulder_lift,elbow_flex,wrist_flex,wrist_roll,gripper} IDs 1-6
                       neck_j1..neck_j6 IDs 7-12 (replaced the old 2-DoF head_motor_1/2 = IDs 7/8 on 2026-11-04)
    COM4 (port_right): right_arm_* IDs 1-6

18 motors -> 18-D action/observation (old head-based datasets/policies with 14-D features are NOT compatible).
Arm/neck motor values are lerobot units (DEGREES around the calibration range midpoint; grippers 0-100).
URDF joint angles (hardware/wallerobot_v0_1/urdf/wallerobot_static.urdf): `urdf_positions(obs)`.

Safety behaviour (see config):
- read_only=True: connect/disconnect send no write at all (no calibration, configuration or torque writes);
  send_action raises. Use for the sim-real mirror and hand-guided recording.
- torque_on_connect=False (default): configuration runs with torque off and torque STAYS off afterwards
  (lerobot's bus.torque_disabled() would re-enable it on exit).
- The neck EEPROM (homing / limits written by tools/calib_neck.py, Phase 92 on IDs 9-11, PID) is never written:
  neck calibration is adopted from the motors and the neck configuration is only verified.
- Goals are clamped so neck J2/J3 and arm shoulder_lift/elbow_flex are never pushed further into their mechanical
  stops (wallerobot_calib.stop_tick_limits), then max_relative_target (float or per-motor dict) applies.
- Connect pings every ID itself (the broadcast ping with 12 motors on COM3 loses replies).
"""

import logging
import time
from functools import cached_property

from lerobot.cameras import DepthCamera, make_cameras_from_configs
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode
from lerobot.robots.robot import Robot
from lerobot.robots.utils import ensure_safe_goal_position

from . import wallerobot_calib
from .config_xlerobot_static import XLerobotStaticConfig

logger = logging.getLogger(__name__)

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
NECK_JOINTS = [f"neck_j{i}" for i in range(1, 7)]  # IDs 7..12
STALE = ("head_motor_1", "head_motor_2")
MAX_RES = 4095  # lerobot DEGREES uses resolution - 1


class XLerobotStatic(Robot):
    config_class = XLerobotStaticConfig
    name = "xlerobot_static"

    def __init__(self, config: XLerobotStaticConfig):
        super().__init__(config)
        self.config = config
        body = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100

        def arm(side):
            return {
                f"{side}_arm_{j}": Motor(i + 1, "sts3215", MotorNormMode.RANGE_0_100 if j == "gripper" else body)
                for i, j in enumerate(ARM_JOINTS)
            }

        left_motors = arm("left") | {n: Motor(7 + i, "sts3215", body) for i, n in enumerate(NECK_JOINTS)}
        right_motors = arm("right")
        stale = [k for k in STALE if k in self.calibration]
        if stale:
            logger.warning(f"Ignoring stale old-head entries {stale} in {self.calibration_fpath} (IDs 7/8 are neck J1/J2)")
            self.calibration = {k: v for k, v in self.calibration.items() if k not in STALE}
        self.buses = {
            "left": FeetechMotorsBus(port=config.port_left, motors=left_motors,
                                     calibration={k: v for k, v in self.calibration.items() if k in left_motors}),
            "right": FeetechMotorsBus(port=config.port_right, motors=right_motors,
                                      calibration={k: v for k, v in self.calibration.items() if k in right_motors}),
        }
        self.joint_cal = wallerobot_calib.load()
        self.cameras = make_cameras_from_configs(config.cameras)

    # ---- features -------------------------------------------------------------------------------
    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{m}.pos": float for bus in self.buses.values() for m in bus.motors}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        ft = {}
        for key, cam in self.cameras.items():
            if getattr(cam, "use_rgb", True):
                ft[key] = (cam.height, cam.width, 3)
            if isinstance(cam, DepthCamera) and cam.use_depth:
                ft[f"{key}_depth"] = (cam.height, cam.width, 1)
        return ft

    @cached_property
    def observation_features(self) -> dict:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict:
        return self._motors_ft

    # ---- lifecycle ------------------------------------------------------------------------------
    @property
    def is_connected(self) -> bool:
        return all(b.is_connected for b in self.buses.values()) and all(
            c.is_connected for c in self.cameras.values()
        )

    @property
    def is_calibrated(self) -> bool:
        return all(b.is_calibrated for b in self.buses.values())

    def _ping_all(self, bus: FeetechMotorsBus) -> None:
        missing = [m for m in bus.motors if bus.ping(m, num_retry=self.config.num_ping_retries) is None]
        if missing:
            raise ConnectionError(f"{bus.port}: no answer from {missing} (power? cable? port used by another program?)")

    def connect(self, calibrate: bool = True) -> None:
        for bus in self.buses.values():
            bus.connect(handshake=False)
            self._ping_all(bus)
        if calibrate and not self.is_calibrated:
            self.calibrate()
        for cam in self.cameras.values():
            cam.connect()
        if self.config.read_only:
            logger.info(f"{self} connected READ-ONLY (no configuration / torque writes)")
        else:
            self.configure()
        logger.info(f"{self} connected.")

    def calibrate(self) -> None:
        """Neck: always adopt the motor EEPROM (tools/calib_neck.py owns it). Arms: the calibration file is written to
        the motors when it differs (previous behaviour), or adopted from EEPROM when there is no entry.
        read_only: nothing is written to the motors; EEPROM values are used in memory where they differ."""
        new, changed = dict(self.calibration), False
        for bus in self.buses.values():
            eeprom = bus.read_calibration()
            to_write = {}
            for m, cal in eeprom.items():
                file_cal = self.calibration.get(m)
                same = file_cal is not None and (file_cal.homing_offset, file_cal.range_min, file_cal.range_max) == \
                    (cal.homing_offset, cal.range_min, cal.range_max)
                if same:
                    continue
                if file_cal is None and not self.config.adopt_motor_calibration:
                    raise RuntimeError(f"{m}: no calibration file entry and adopt_motor_calibration=False")
                if m in NECK_JOINTS or file_cal is None or self.config.read_only:
                    if file_cal is not None:
                        logger.warning(f"{m}: calibration file differs from EEPROM -> using EEPROM "
                                       f"({file_cal} -> {cal}){' (read-only)' if self.config.read_only else ''}")
                    new[m] = cal; changed = True
                else:
                    to_write[m] = file_cal
            if to_write:
                logger.info(f"Writing calibration file entries {list(to_write)} to the motors on {bus.port}")
                bus.write_calibration(to_write, cache=False)
            bus.calibration = {m: new[m] for m in bus.motors}
        self.calibration = new
        if changed:
            backup = self.calibration_fpath.with_name(f"{self.calibration_fpath.stem}.bak-{time.strftime('%Y%m%d-%H%M%S')}.json")
            if self.calibration_fpath.is_file():
                backup.write_text(self.calibration_fpath.read_text())
                logger.info(f"Backed up the old calibration file to {backup}")
            self._save_calibration()
            logger.info(f"Calibration saved to {self.calibration_fpath}")

    def configure(self) -> None:
        """Arms: lerobot SO101 configuration (torque off while writing). Neck: verify only, never written.
        Torque is enabled afterwards only with torque_on_connect=True (goal = present position first)."""
        if self.config.torque_on_connect:
            self._refuse_wrapped(self.joint_cal.keys())
        for side, bus in self.buses.items():
            arm_motors = [m for m in bus.motors if m not in NECK_JOINTS]
            bus.disable_torque()
            for m in arm_motors:
                bus.write("Return_Delay_Time", m, 0)
                bus.write("Maximum_Acceleration", m, 254)
                bus.write("Acceleration", m, 254)
                bus.write("Operating_Mode", m, OperatingMode.POSITION.value)
                bus.write("P_Coefficient", m, self.config.position_p_coefficient)
                bus.write("I_Coefficient", m, self.config.position_i_coefficient)
                bus.write("D_Coefficient", m, self.config.position_d_coefficient)
                if m.endswith("gripper"):
                    bus.write("Max_Torque_Limit", m, 500)
                    bus.write("Protection_Current", m, 250)
                    bus.write("Overload_Torque", m, 25)
            for m in [m for m in bus.motors if m in NECK_JOINTS]:
                mode = bus.read("Operating_Mode", m, normalize=False)
                if mode != OperatingMode.POSITION.value:
                    raise RuntimeError(f"{m}: Operating_Mode {mode} != position mode - set it deliberately, the plugin "
                                       "does not write the neck EEPROM")
            if self.config.torque_on_connect:
                present = bus.sync_read("Present_Position", normalize=False, num_retry=self.config.num_read_retries)
                bus.sync_write("Goal_Position", present, normalize=False)
                bus.enable_torque()
                logger.warning(f"{bus.port}: torque ENABLED (holding the present position)")

    def setup_motors(self) -> None:
        raise NotImplementedError("Motor IDs are already set; use tools/set_motor_id.py if needed.")

    # ---- io -------------------------------------------------------------------------------------
    def get_observation(self) -> dict:
        obs = {}
        for bus in self.buses.values():
            pos = bus.sync_read("Present_Position", num_retry=self.config.num_read_retries)
            obs.update({f"{m}.pos": v for m, v in pos.items()})
        for key, cam in self.cameras.items():
            t = time.perf_counter()
            if getattr(cam, "use_rgb", True):
                obs[key] = cam.read_latest()
            if isinstance(cam, DepthCamera) and cam.use_depth:
                obs[f"{key}_depth"] = cam.read_latest_depth()
            logger.debug(f"{self} read {key}: {(time.perf_counter() - t) * 1e3:.1f}ms")
        return obs

    # unit conversions (lerobot unit of motor m <-> raw ticks), using the active calibration
    def _to_ticks(self, m: str, v: float) -> float:
        c, mode = self.calibration[m], self._mode(m)
        if mode is MotorNormMode.DEGREES:
            return v * MAX_RES / 360 + (c.range_min + c.range_max) / 2
        if mode is MotorNormMode.RANGE_0_100:
            return v / 100 * (c.range_max - c.range_min) + c.range_min
        return (v + 100) / 200 * (c.range_max - c.range_min) + c.range_min

    def _from_ticks(self, m: str, t: float) -> float:
        c, mode = self.calibration[m], self._mode(m)
        if mode is MotorNormMode.DEGREES:
            return (t - (c.range_min + c.range_max) / 2) * 360 / MAX_RES
        if mode is MotorNormMode.RANGE_0_100:
            return (t - c.range_min) / (c.range_max - c.range_min) * 100
        return (t - c.range_min) / (c.range_max - c.range_min) * 200 - 100

    def _mode(self, m: str) -> MotorNormMode:
        return next(b.motors[m].norm_mode for b in self.buses.values() if m in b.motors)

    def _refuse_wrapped(self, motors) -> None:
        """Position mode can't cross the encoder wrap: arm lift/elbow with range 0..4095 and the wrap inside their travel
        would turn the long way through the fold stop. Refuse torque / goals until their Homing_Offset is re-centred."""
        bad = sorted(set(motors) & set(wallerobot_calib.wrap_problems(self.joint_cal)))
        if bad:
            raise RuntimeError(f"encoder wrap inside the travel of {bad}: commanding them is unsafe (the motor could turn "
                               "the long way into the fold stop). Re-centre their Homing_Offset first.")

    def clamp_to_stops(self, goal: dict[str, float]) -> dict[str, float]:
        """Clamp lerobot-unit goals so no joint is pushed further into its mechanical stop."""
        out = {}
        for m, v in goal.items():
            lo, hi = wallerobot_calib.stop_tick_limits(m, self.joint_cal.get(m, {})) if m in self.joint_cal else (None, None)
            t = self._to_ticks(m, v)
            t2 = max(lo, t) if lo is not None else t
            t2 = min(hi, t2) if hi is not None else t2
            if t2 != t:
                logger.warning(f"{m}: goal {v:.2f} clamped at its mechanical stop -> {self._from_ticks(m, t2):.2f}")
            out[m] = self._from_ticks(m, t2) if t2 != t else v
        return out

    def send_action(self, action: dict) -> dict:
        if self.config.read_only:
            raise RuntimeError("xlerobot_static is connected read_only=True: send_action is disabled")
        self._refuse_wrapped(k.removesuffix(".pos") for k in action if k.endswith(".pos"))
        goal = self.clamp_to_stops({k.removesuffix(".pos"): v for k, v in action.items() if k.endswith(".pos")})
        sent = {}
        for bus in self.buses.values():
            g = {m: v for m, v in goal.items() if m in bus.motors}
            if not g:
                continue
            mrt = self.config.max_relative_target
            if mrt is not None:
                if isinstance(mrt, dict):
                    missing = [m for m in g if m not in mrt]
                    if missing:
                        raise ValueError(f"max_relative_target has no entry for {missing}")
                    mrt = {m: float(mrt[m]) for m in g}
                else:
                    mrt = float(mrt)
                present = bus.sync_read("Present_Position", list(g), num_retry=self.config.num_read_retries)
                g = ensure_safe_goal_position({m: (v, present[m]) for m, v in g.items()}, mrt)
            bus.sync_write("Goal_Position", g)
            sent.update(g)
        return {f"{m}.pos": v for m, v in sent.items()}

    # ---- helpers for this robot -----------------------------------------------------------------
    def urdf_positions(self, obs: dict) -> dict[str, float]:
        """lerobot observation -> URDF joint angles (rad) of wallerobot_static.urdf (MuJoCo twin joint names)."""
        out = {}
        for k, v in obs.items():
            m = k.removesuffix(".pos")
            if k.endswith(".pos") and m in self.joint_cal and m in self.calibration:
                out[m] = wallerobot_calib.tick_to_rad(self.joint_cal[m], self._to_ticks(m, v))
        return out

    def home_action(self) -> dict[str, float]:
        """Return / init position in lerobot units: neck `poses.home.target_ticks` (clamped into the limits) and the
        arms' standard rest pose (lift/elbow on their stops = the stop clamp itself, grippers closed)."""
        act = {}
        for m, c in self.joint_cal.items():
            if m not in self.calibration:
                continue
            t = c.get("home_tick") if m in NECK_JOINTS else (c.get("closed_tick") if c["type"] == "gripper" else c.get("rest_tick"))
            if t is not None:
                act[f"{m}.pos"] = self._from_ticks(m, t)
        return act

    def disconnect(self) -> None:
        for bus in self.buses.values():
            bus.disconnect(self.config.disable_torque_on_disconnect and not self.config.read_only)
        for cam in self.cameras.values():
            cam.disconnect()
        logger.info(f"{self} disconnected.")
