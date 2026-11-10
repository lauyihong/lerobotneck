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

## Why active perception

The {site}`active perception page <why/>` compares six camera setups in the simulator's desk scene. All six use the
same D435i camera model, mounted at the same spot on the tower:

| setup | DoF | share of the scene seen | requested views taken (of 80) | ... with a level horizon | own body in the image |
|---|---|---|---|---|---|
| fixed camera (XLeRobot head, motors frozen) | 0 | 22 % | 0 | 0 | 0 % |
| XLeRobot 2-axis head (pan + tilt) | 2 | 73 % | 0 | 0 | 3 % |
| stock SO101 + camera at the XLeRobot wrist-camera mount | 5 | 98 % | 32 | 16 | 13 % |
| SO101-X: the stock SO101, D435i holder in place of the moving jaw | 6 | 99 % | 58 | 45 | 15 % |
| LeRobotNeck | 6 | 94 % | 56 | 40 | 1 % |
| LeRobotNeck-02 (concept): J3–J4 link 7.4 cm longer | 6 | 98 % | 68 | 39 | 0 % |

What the comparison shows:

- **Moving the camera.** Turning the camera in place covers the open desk. It misses what is behind or inside things
  (the sticker on the far side of the block, the inside of the box). Only setups that move the camera itself see
  those.
- **The sixth joint.** Seeing a spot once is not the same as getting a requested view (camera here, looking there).
  For that, the sixth joint (camera tilt) matters most: SO101-X uses the gripper motor for it and needs no new parts.
- **The stock fixed jaw.** SO101-X keeps the stock fixed jaw, which covers on average 15 % of the image. LeRobotNeck
  uses XLeRobot's soft-gripper jaw there.
- **Link length.** LeRobotNeck sees a little less than the stock arm because its J3–J4 link is shorter. The
  LeRobotNeck-02 concept, with the link as long as the stock lower arm, closes that gap.

The page shows each setup's coloured point cloud and replays the 10 views that cover the most.

**How the numbers are made.** They come from `projects/web_sim/scripts/make_coverage.py`. Run it after
`make_neck_routes.py`; it takes about 3 minutes and writes `web/public/why/`.

- **Seen point:** a point counts as seen when it is in the image, 0.10–1.5 m away, and faces the camera (less than 80°
  from its normal). The ray to it must also hit nothing else: the desk, the objects, the tower and the setup's own
  moving parts block it, the camera's own housing does not.
- **Poses:** the head on a 2° pan × tilt grid; the arms 30,000 random joint poses each that keep clear of the desk,
  the objects and the tower.
- **Requested views:** 80 camera positions 12–32 cm above the desk, each looking at a point on the desk. Inverse
  kinematics from 12 random starts must reach the position within 1.5 cm and the aim within 3°, with the line of sight
  clear. "Level" also keeps the horizon within 5°.

Limits: this is simulation only. LeRobotNeck-02 is a concept, with a box drawn as the extra link. The SO101-X tilt
range comes from the neck holder and was not checked on real parts. Motor torque is not modelled.

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
