# Hardware

wallerobot is built from the parts of an XLeRobot. If you already have one, you keep the body, the tower and both
SO101 follower arms; you leave out the mobile base and replace the 2-DoF head with a 6-DoF neck.

```{toctree}
:maxdepth: 1

bom
wiring
calibration
safety
```

## Structure

| part | what it is |
|---|---|
| body + truss tower | XLeRobot body (lower part with the arm pads) and its truss tower, standing on a desk |
| arms | 2 × SO101 follower arm, mounted on the body pads, **both facing forward** (yaw −90° in the body frame) |
| neck | modified SO101 with 6 motors: J1 base pan, J2 lift, J3 elbow, **J4 side swing** (new link), J5 camera roll, J6 camera tilt |
| head camera | Intel RealSense D435i in the upstream XLeRobot head-tilt holder, on the neck's J6 |
| wrist cameras | one USB camera on each gripper (the XLeRobot wrist camera mount) |

### The neck

The neck is an SO101 arm whose gripper became a camera mount:

```text
body ─fixed─▶ neck_base (on top of the tower)
  ─J1 pan─▶ shoulder ─J2 lift─▶ upper arm ─J3 elbow─▶ NEW LINK
  ─J4 side swing─▶ wrist ─J5 camera roll─▶ J6 housing ─J6 camera tilt─▶ D435i holder
```

- J1–J3 and the J4→J6 wrist are stock SO101 parts.
- The **new J3→J4 link** (`hardware/neck_v0_1/wallerobot_new_wrist_dof_0_1.stl`) puts the J4 axis perpendicular to
  J3. In a stock SO101 these two axes are parallel. The link offsets J4 by 61.2 mm along the link and 16.85 mm to
  the side.
- In the folded **home** pose, J2 and J3 rest on their mechanical fold stops and the camera looks forward.

```{image} /../_static/img/robot_home.png
:alt: home pose, neck folded
:width: 600px
```

## Frames and naming

- **Root frame `wallerobot_body`** = XLeRobot's `top_base_link`. Robot **forward is −x**, the **left arm is at −y**,
  and z points up.
- **Joint names** are the motor names of the LeRobot plugin:
  - arms: `left_arm_{shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper}`, and the same for
    `right_arm_`;
  - neck: `neck_j1` … `neck_j6`.
- **Camera frames:** `neck_camera_color_optical_frame` (x right, y down, z forward), `left_wrist_camera`,
  `right_wrist_camera`.

The full joint table, with limits and calibration, is in [](../model/index.md).
