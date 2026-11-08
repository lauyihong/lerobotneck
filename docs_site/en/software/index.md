# Software: LeRobot plugin

`plugins/lerobot_robot_xlerobot_static` registers the robot type **`xlerobot_static`** with
[LeRobot](https://github.com/huggingface/lerobot). The standard `lerobot-*` commands can then record, replay and run
policies on wallerobot.

## Install

```bash
# inside a LeRobot environment (pip install -e ".[feetech]" in a LeRobot checkout)
pip install -e plugins/lerobot_robot_xlerobot_static
```

## Motors and observations

18 motors, named like the URDF joints:
- `left_arm_*` (bus 1, IDs 1–6);
- `neck_j1` … `neck_j6` (bus 1, IDs 7–12);
- `right_arm_*` (bus 2, IDs 1–6).

The cameras are `head` (D435i colour, plus `head_depth` with `head_use_depth=true`), `left_wrist` and `right_wrist`.

## Options

| option | default | meaning |
|---|---|---|
| `port_left` / `port_right` | `COM3` / `COM4` | serial ports of bus 1 (left arm + neck) and bus 2 (right arm) |
| `read_only` | `false` | no write of any kind to the motors; `send_action` raises. Use it first. |
| `torque_on_connect` | `false` | enable torque at the end of connect (goal = present first). With `false` every motor stays limp. |
| `max_relative_target` | `10.0` | per-step clamp on the goal: a number, or `{motor: value}` |
| `head_use_depth` | `false` | also stream D435i depth |
| `use_degrees` | `true` | degrees, as in current LeRobot SO101 |
| `cameras` | wrist index 0/2, D435i serial of the reference robot | set your own camera indices and RealSense serial |

Safety built into the plugin:
- It never writes the neck EEPROM, which `tools/calib_neck.py` owns.
- It refuses position commands for the arm shoulder_lift and elbow_flex until their encoder wrap is re-centred.
- It releases torque on disconnect.

## A first read-only test

```bash
lerobot-teleoperate --help        # check that lerobot sees the plugin (robot.type choices)
python plugins/lerobot_robot_xlerobot_static/tests/test_readonly_hw.py
```

The read-only test connects with `read_only=True`, compares one observation with a raw serial read, and checks that
torque is off before and after.

## Datasets

Recording follows the normal LeRobot workflow (`lerobot-record` with `--robot.type=xlerobot_static`). Keep
`--dataset.push_to_hub=false` until you mean to publish.

:::{note}
A ROS 2 stack with a web control desk and a safety-enforcing hardware server exists for the reference robot. It
runs on Windows with ROS 2 Jazzy in WSL2. It is not part of this repository yet.
:::
