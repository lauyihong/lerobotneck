# Customize the robot

Every change follows the same three steps:

1. Write an **override file** with the keys you change. Edit `robot.yaml` itself only to change the reference robot.
2. Build it into its own folder: `python hardware/build_all.py --config my.yaml --out build/my`.
3. Look at it with `python projects/sim_mirror/scripts/view.py`, or load `build/my/mjcf/scene.xml` in MuJoCo.

```{image} /../_static/img/variant_reference_top.png
:alt: reference robot seen from the front
:width: 32%
```
```{image} /../_static/img/variant_arms_out_2cm.png
:alt: arms moved 2 cm outwards
:width: 32%
```
```{image} /../_static/img/variant_single_arm_left.png
:alt: single left arm
:width: 32%
```

*From left to right: the reference robot, `arms_out_2cm.yaml`, and `single_arm_left.yaml`.*

## Move the arms

The two arms are independent. `mount.xyz` is the centre of the SO101 base circle on the pad, and `yaw_deg` rotates
the arm about the vertical axis.

```yaml
# hardware/variants/arms_out_2cm.yaml: both arms 2 cm further out
arms:
  left:
    mount: {xyz: [-0.090, -0.1529, 0.048], yaw_deg: -90}
  right:
    mount: {xyz: [-0.090, 0.1529, 0.048], yaw_deg: -90}
```

```bash
python hardware/build_all.py --config hardware/variants/arms_out_2cm.yaml --out build/arms_out_2cm
```

## Single arm

```yaml
arms:
  right:
    enabled: false
```

The result has 12 joints (left arm + neck) and two cameras. The MJCF builder skips the missing arm's cameras and
tip sites, and drops the contact excludes that would refer to it.

## A different neck link

To try another J3→J4 link:
1. Export the link as an STL in millimetres.
2. Measure two points in the STL's own frame:
   - `j3_axis_mm`: the centre of the J3 horn (the axis the link turns about);
   - `j4_shaft_mm`: the non-shaft end of the J4 motor where it sits in the link.
3. Point `neck.link34.stl` at the file and set the two points:

```yaml
neck:
  link34:
    stl: my_parts/neck_link_long.stl
    j3_axis_mm: [41.0, 23.2, 12.0]
    j4_shaft_mm: [-40.2, 23.9, 28.85]
```

If the J4 motor comes out on the wrong side or faces the wrong way, flip `side` or `j4_shaft_sign` (±1). Check the
result at q = 0 with `view.py --pose zero`. The neck's J3 should look straight, with the J4 motor in line with the
upper arm.

## Camera mount and lens

```yaml
cameras:
  head:
    offset_in_holder_m: [0.025, 0.0, 0.03]
    vfov_deg: 43.2          # from your intrinsics: 2*atan(height / (2*fy))
  wrist:
    vfov_deg: 72.0
```

## Your own calibration

Point `calibration:` to your files. The URDF limits, the home pose of the web model and the joint table all follow
them.

```yaml
calibration:
  neck: my_robot/neck_calibration.json
  arms: my_robot/arm_calibration.json
```

:::{tip}
Share a variant that works: add the override file to `hardware/variants/` in a pull request. CI builds every file in
that folder.
:::
