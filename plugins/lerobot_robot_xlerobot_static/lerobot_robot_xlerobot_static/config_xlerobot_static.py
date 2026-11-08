"""Config for XLeRobot with the mobile base removed + the 6-DoF neck v0.1 (this workspace's robot).

Defaults match the bench-verified wiring of this machine (see hardware/hardware.yaml):
  COM3 -> left arm (ids 1-6) + neck_j1..neck_j6 (ids 7-12)
  COM4 -> right arm (ids 1-6)
  OpenCV MSMF index 0 -> left wrist cam, 2 -> right wrist cam, 1 = D435i RGB via UVC (never open it here)
     (2026-11-05, user-confirmed; indices change after re-plugging -> projects/sim_mirror/scripts/probe_cams.py, read-only)
  RealSense D435i SN 261222077675 -> head (neck) RGB(-D)
"""

from dataclasses import dataclass, field

from lerobot.cameras.configs import CameraConfig, ColorMode, Cv2Rotation
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig
from lerobot.robots.config import RobotConfig


def default_cameras() -> dict[str, CameraConfig]:
    return {
        "left_wrist": OpenCVCameraConfig(index_or_path=0, fps=30, width=640, height=480),
        "right_wrist": OpenCVCameraConfig(index_or_path=2, fps=30, width=640, height=480),
        "head": RealSenseCameraConfig(
            serial_number_or_name="261222077675",
            fps=30,
            width=640,
            height=480,
            color_mode=ColorMode.RGB,
            rotation=Cv2Rotation.NO_ROTATION,
            use_depth=False,
        ),
    }


@RobotConfig.register_subclass("xlerobot_static")
@dataclass
class XLerobotStaticConfig(RobotConfig):
    port_left: str = "COM3"  # left arm + neck
    port_right: str = "COM4"  # right arm

    # READ-ONLY: no write of any kind to the motors (calibration, configuration, torque); send_action raises.
    read_only: bool = False
    # Enable torque at the end of connect (goal = present position first). False keeps every motor limp.
    torque_on_connect: bool = False
    disable_torque_on_disconnect: bool = True
    # Per-step clamp on goal position (same units as actions): float or {motor_name: value} for every commanded motor.
    max_relative_target: float | dict[str, float] | None = 10.0

    cameras: dict[str, CameraConfig] = field(default_factory=default_cameras)
    head_use_depth: bool = False  # also stream D435i depth as observation "head_depth"

    # DEGREES matches current lerobot SO101 defaults; set False for old XLeRobot datasets (RANGE_M100_100).
    use_degrees: bool = True

    # If the calibration file has no entry for a motor, adopt the offsets/limits burned into the motor EEPROM
    # (the neck is ALWAYS adopted: tools/calib_neck.py owns its EEPROM).
    adopt_motor_calibration: bool = True

    position_p_coefficient: int = 16
    position_i_coefficient: int = 0
    position_d_coefficient: int = 32
    num_read_retries: int = 2
    num_ping_retries: int = 3

    def __post_init__(self):
        super().__post_init__()
        head = self.cameras.get("head")
        if self.head_use_depth and isinstance(head, RealSenseCameraConfig):
            head.use_depth = True
