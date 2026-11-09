# wallerobot

**English** · [中文](README.zh-CN.md)

wallerobot is an open, **configurable** robot description for a stationary XLeRobot variant: two SO101 arms and a
6-DoF neck that carries an Intel RealSense D435i. It has **no mobile base**. One YAML file generates the URDF, the
MuJoCo model and a browser simulator from the same numbers. You change the robot by editing that file instead of
patching a simulator.

- **LeRobotNeck:** [showcase website](https://lauyihong.github.io/lerobotneck/): the 6-DoF neck on its own, with
  live demos, a neck simulator, and an exploded view of how one SO101 arm plus two new prints becomes the neck.
- **Try it:** [browser simulator](https://lauyihong.github.io/lerobotneck/sim/). It runs MuJoCo in the page, with
  nothing to install.
- **Docs:** [https://lauyihong.github.io/lerobotneck/docs/](https://lauyihong.github.io/lerobotneck/docs/), in English and 中文.

| | |
|---|---|
| Degrees of freedom | 18: 2 × SO101 arm (6 each, including the gripper) + 6-DoF neck |
| Actuators | 18 × Feetech STS3215 (12 V), on two 1 Mbps buses |
| Cameras | RealSense D435i on the neck (RGB-D + IMU), and one USB camera on each wrist |
| Model outputs | URDF, MuJoCo MJCF (position actuators, 3 cameras), and a web model |
| Software | LeRobot robot plugin `--robot.type=xlerobot_static` |

## How it differs from XLeRobot
[XLeRobot](https://github.com/Vector-Wangel/XLeRobot) is a mobile dual-arm robot. wallerobot keeps its body, tower
and SO101 arms, and changes three things:
- **No mobile base.** The body stands on a desk.
- **6-DoF neck instead of the 2-DoF head.** The neck is a modified SO101 with a new J3→J4 link that turns J4 into a
  side swing.
- **Arms face forward**, towards the main camera.

The model is generated from configuration. Arm mounts, neck geometry, cameras and simulation parameters are all in
[`hardware/wallerobot_v0_1/robot.yaml`](hardware/wallerobot_v0_1/robot.yaml).

## Build the model

Only Python is needed. The submodules and the full robot-learning stack are not.

```bash
pip install -r requirements-model.txt
python hardware/build_all.py              # URDF, MJCF, joint table and web model from robot.yaml
python hardware/build_all.py --check      # rebuild, then confirm nothing changed
```

Make a variant without touching the reference model:

```bash
python hardware/build_all.py --config hardware/variants/single_arm_left.yaml --out build/single_arm_left
```

| output | path |
|---|---|
| URDF + meshes | `hardware/wallerobot_v0_1/urdf/wallerobot_static.urdf` |
| MuJoCo model + scene | `projects/sim_mirror/models/scene.xml` |
| joint table | `hardware/wallerobot_v0_1/JOINTS.md` |
| browser simulator | `projects/web_sim/` (`npm ci && npm run dev` in `projects/web_sim/web`) |

## Repository layout
```
hardware/            robot description: robot.yaml, builders, calibration, CAD meshes, vendored upstream files
  wallerobot_v0_1/   the full robot (config, URDF, arm calibration, joint table)
  neck_v0_1/         the 6-DoF neck (new link STL, neck builder, neck calibration)
  upstream/          unmodified XLeRobot URDF + meshes the builders need (Apache-2.0)
  variants/          example override files
projects/sim_mirror/ MuJoCo model builder, viewer, read-only sim-real mirror
projects/web_sim/    browser simulator (MuJoCo WASM + three.js)
plugins/             LeRobot robot plugin (18 motors, read-only mode, torque guards)
tools/               servo bus utilities (scan, set ID, neck calibration)
docs_site/           documentation sources (Sphinx + MyST, en / zh)
```

## Status
This is v0.1, the first public release of a hobby project. What has been checked so far:
- The neck has been run with torque on, without the arms.
- The arms have only been read, never driven: their encoder wrap must be re-centred before they can be commanded.
- The browser simulator and the model are simulation only.

The docs list the known limitations: placeholder gripper geometry, estimated wrist-camera FOV, and arm mounts taken
from CAD rather than measured.

## License
[Apache-2.0](LICENSE). This project builds on XLeRobot, LeRobot and the SO-ARM100/101 design, all Apache-2.0; see
[NOTICE](NOTICE).
