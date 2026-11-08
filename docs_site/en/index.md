# wallerobot

```{image} /../_static/img/robot_ready.png
:alt: wallerobot model in the ready pose
:class: hero
```

**wallerobot** is an open, configurable robot description for a stationary
[XLeRobot](https://github.com/Vector-Wangel/XLeRobot) variant: two SO101 arms and a 6-DoF neck that carries an
Intel RealSense D435i. There is no mobile base. One YAML file is the single source of the URDF, the MuJoCo model and
the browser simulator, so you change the robot by editing numbers instead of patching a simulator.

::::{grid} 1 1 3 3
:gutter: 2

:::{grid-item-card} Try the simulator
:link: simulation/index
:link-type: doc
MuJoCo in your browser, with nothing to install. Use the keyboard, the mouse or fixed routes.
:::

:::{grid-item-card} Configure the model
:link: model/customize
:link-type: doc
Move an arm, drop an arm, or change the neck link, then rebuild the URDF and MJCF with one command.
:::

:::{grid-item-card} Build the hardware
:link: hardware/index
:link-type: doc
What changes compared with an XLeRobot, plus motor IDs, calibration and safety rules.
:::
::::

## At a glance

| | |
|---|---|
| Degrees of freedom | 18: two SO101 arms (6 each, including the gripper) and a 6-DoF neck |
| Actuators | 18 × Feetech STS3215 (12 V, 4096 ticks/rev) on two 1 Mbps serial buses |
| Cameras | RealSense D435i on the neck (RGB, depth, IMU) and one USB camera on each wrist |
| Model | URDF, MuJoCo MJCF (18 position actuators, 3 cameras), browser model |
| Software | LeRobot robot plugin `--robot.type=xlerobot_static` |

## How it differs from XLeRobot

- **No mobile base.** The XLeRobot body and truss tower stand on a desk.
- **6-DoF neck instead of the 2-DoF head.** The neck is a modified SO101. A new link between J3 and J4 turns the J4
  axis 90°, so J4 swings the head sideways. The D435i sits on the upstream head-tilt holder.
- **Both arms face forward**, in the same direction as the main camera.
- **Configuration-driven model.** Arm mounts, neck geometry, cameras and simulation parameters live in one YAML file.
  The build is checked in CI, and variants are override files.

## Status (v0.1)

:::{admonition} What has been verified
:class: note
- The model matches the real robot's encoder readings at rest. The neck camera orientation was fitted to the
  D435i IMU, with a residual of about 1°.
- The neck has run with torque on (J1, J2, J3 and J5) through a demo of poses.
- The arms have been read but never driven. Their shoulder_lift and elbow_flex encoders wrap inside their travel,
  and must be re-centred before position control.
:::

Known limitations of the model:
- The grippers are stock SO101 jaw meshes, while the real robot uses parallel grippers whose CAD is not public yet.
- The wrist camera field of view is estimated, not measured.
- The arm mounts come from CAD and have not been measured on the robot.
- The insertion depth of the J4 motor in the new neck link is an assumption.
- Neck inertias are placeholders.

```{toctree}
:maxdepth: 2
:hidden:

getting_started
hardware/index
model/index
simulation/index
software/index
community/index
```
