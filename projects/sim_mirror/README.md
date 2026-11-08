# sim_mirror — MuJoCo model, viewer and read-only sim-real mirror

The MuJoCo model in `models/` is **generated**: `python hardware/build_all.py` runs `scripts/build_mjcf.py`. Its drive
gains, cameras and desk height come from `hardware/wallerobot_v0_1/robot.yaml` (`sim`, `cameras`).

## Simulation only (no hardware)

| script | what it does |
|---|---|
| `scripts/view.py` | interactive MuJoCo viewer: `--pose rest\|home\|zero\|pose.json`, `--physics`, `--collision` (show hulls), `--snapshot x.png` (headless renders) |
| `scripts/check_mjcf.py` | checks the MJCF against the URDF with its own forward kinematics: joints, limits, frames, mesh bounds, cameras, mass |
| `scripts/sim_check.py` | headless smoke test: hold the rest pose under physics, sinusoid tracking, NaN check |
| `scripts/sim_robot.py` | `SimWallerobot`, a plugin-like interface (`get_observation`, `send_action`, camera renders) |
| `scripts/calib.py` | raw servo ticks ↔ URDF joint angles from the calibration files in `robot.yaml` |

```bash
python projects/sim_mirror/scripts/view.py --pose home
```

## With the real robot (read-only, never writes a register)

| script | what it does |
|---|---|
| `scripts/real_reader.py` | reads all 18 encoders (Present_Position only), checks that torque is off |
| `scripts/mirror.py` | real encoders drive the twin at ~70 Hz; the real D435i and wrist cameras are shown above the simulated ones |
| `scripts/probe_cams.py` | grabs one frame from each OpenCV camera index, to find the wrist cameras |
| `scripts/imu_check.py`, `scripts/imu_fit.py` | neck camera orientation from the D435i accelerometer vs the twin |

Hardware scripts need the full environment (LeRobot, pyrealsense2, opencv). They default to the reference robot's
ports and camera serial number. Change those for your machine.

`outputs/rest_pose_20261105.json` is the measured folded rest pose. The MJCF builder uses it to exclude contacts
between parts that already touch at rest.
