# Simulation

There are three ways to simulate wallerobot, all built from the same generated model:

| | where | needs |
|---|---|---|
| **browser simulator** | {site}`sim/` | a browser (also on a phone) |
| **MuJoCo viewer** | `projects/sim_mirror/scripts/view.py` | `requirements-model.txt` |
| **sim-real mirror** | `projects/sim_mirror/scripts/mirror.py` | the real robot (read-only) + LeRobot |

```{tip}
The neck alone has its own showcase website, **LeRobotNeck** ({site}`website <./>`): the {site}`neck simulator <play/>` is a neck-only simulator with demos you can
pause and adjust, and the {site}`exploded view <neck/>` shows how a standard SO101 arm plus one new printed link and XLeRobot's soft-gripper jaw becomes
the neck. Its model is the `hardware/variants/lerobotneck.yaml` variant (tower + neck, no body, no arms).
```

## Browser simulator

MuJoCo 3.16 runs in the page as WebAssembly, and three.js draws it. Nothing is sent anywhere: it is a pure
simulation that never talks to hardware.

**Controls**
- **Keyboard:** move each arm's gripper along x, y and z, change the wrist pitch and roll, and open or close the
  gripper. The neck has its own keys. Press `?` for the full map.

  | | left arm | right arm |
  |---|---|---|
  | forward / back | W / S | I / K |
  | left / right | A / D | J / L |
  | up / down | Q / E | U / O |
  | wrist pitch | R / F | Y / H |
  | wrist roll | Z / C | N / M |
  | gripper | V | B |

  Neck: ←/→ turn the head (J1), ↑/↓ tilt the camera (J6). `[` / `]` select a joint and `,` / `.` move it.
  `G` = ready pose, `X` = home, space = play/pause the route.
- **Mouse:**
  - Click a part to select its joint.
  - Click an arm to get a 3-axis drag handle on its gripper; inverse kinematics follows the handle.
  - Type xyz targets in millimetres.
- **Routes:** fixed demos (neck, arms, "look at the hands"), interpolated with minimum-jerk curves. You can record
  your own route pose by pose, then play it, save it, or load a JSON route.
- **Camera picture-in-picture:** what the neck D435i sees (vertical FOV 43.2°).

Gravity compensation imitates the stiff position hold of the real servos. The IK is damped least squares on the
point between the two jaw tips, and keeps the elbow bent so that it does not flip through the singularity.

## MuJoCo viewer (Python)

```bash
python projects/sim_mirror/scripts/view.py                 # rest pose, kinematic (sliders = joint angles)
python projects/sim_mirror/scripts/view.py --pose zero     # all joints 0
python projects/sim_mirror/scripts/view.py --physics       # run the physics with position actuators
python projects/sim_mirror/scripts/view.py --collision     # show the collision hulls
python projects/sim_mirror/scripts/view.py --snapshot x.png   # headless: 4 views + the neck camera
```

`SimWallerobot` in `projects/sim_mirror/scripts/sim_robot.py` gives a plugin-like Python interface
(`get_observation()`, `send_action()`, camera images) for scripts and policies.

## Sim-real mirror (read-only)

`mirror.py` reads the 18 real encoders at about 70 Hz and poses the model to match. The real camera images are shown
above the simulated ones from the same joint state. It only **reads** registers and checks that torque stays off,
so it is the safe first thing to run on a new robot. Use it to verify your calibration: move a joint by hand and
the model must follow in the same direction.

```bash
python projects/sim_mirror/scripts/mirror.py --left-wrist 0 --right-wrist 2
```
