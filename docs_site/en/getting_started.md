# Getting started

Choose the path that fits what you want to do. Each one needs less setup than the next.

## 1. Look at the robot in the browser

Open the {site}`browser simulator <sim/>`. It needs no install and also works on a phone.
- Press `?` for the key map.
- Click a part of the robot to select its joint.
- Click an arm to get a drag handle on its gripper.
- Under *Fixed routes*, play the neck and arm demos.

## 2. Build the model on your computer

You only need Python 3.10 or newer. The robot-learning stack is not needed.

```bash
git clone https://github.com/lauyihong/lerobotneck.git
cd lerobotneck
pip install -r requirements-model.txt
python hardware/build_all.py --check
```

`--check` rebuilds every generated file (URDF, MJCF, joint table, web model) from
`hardware/wallerobot_v0_1/robot.yaml`. It reports `CHECK OK` when the result matches what is committed. Open the
model in MuJoCo's viewer:

```bash
python projects/sim_mirror/scripts/view.py --pose home
```

Next, see [](model/customize.md) to make your own variant.

## 3. Run the browser simulator locally

```bash
cd projects/web_sim/web
npm ci
npm run dev        # http://localhost:5180
```

## 4. Connect a real robot

This path needs the hardware and a full [LeRobot](https://github.com/huggingface/lerobot) install. Read
[](hardware/safety.md) first, then:
1. Set the motor IDs ([](hardware/wiring.md)).
2. Calibrate ([](hardware/calibration.md)).
3. Install the plugin ([](software/index.md)) and connect it in read-only mode.
