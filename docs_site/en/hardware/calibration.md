# Calibration

Calibration maps raw servo ticks to the joint angles of the URDF:

$$q = \text{sign} \cdot (\text{Present\_Position} - \text{zero\_tick}) \cdot \frac{2\pi}{4096}$$

`Present_Position` already includes the servo's EEPROM `Homing_Offset`. Grippers are linear from `closed_tick` to
`open_tick`. Each robot has its own calibration files, and `robot.yaml` points to them:

| file | contents |
|---|---|
| `hardware/neck_v0_1/neck_calibration.json` | neck: per joint `id`, `sign`, `zero_tick`, range, URDF limits; the `home` pose; history |
| `hardware/wallerobot_v0_1/arm_calibration.json` | arms: `sign`, `zero_tick`, ranges, gripper closed/open, the standard rest pose |

The repository ships the reference robot's files. With your own robot, calibrate and replace them (or point
`calibration:` in an override file to yours), then run `python hardware/build_all.py`. The URDF limits follow the
calibration.

## Neck: `tools/calib_neck.py`

Hand-guided, with **torque off the whole time**: nothing moves by itself.

```bash
python tools/calib_neck.py guide        # draws calib_guide.png: the zero pose and '+' direction of every joint
python tools/calib_neck.py calibrate    # interactive; bus 1 must be free
python tools/calib_neck.py show         # read the encoders and draw the current pose
```

1. **Zero:** put each joint at its URDF zero (see the guide). The zero poses are:
   - J2: upper arm horizontal;
   - J3: J4 motor in line with the upper arm;
   - J4: wrist straight;
   - J5: camera upright, below the J6 housing;
   - J6: optical axis along the J5 axis.
2. **Direction:** turn each joint about 30° in its "+" direction.
3. **Range:** move every joint slowly through its full range.
4. **Write:** the tool writes `Homing_Offset` (range midpoint → 2047) and the position limits to the EEPROM, and
   writes the JSON file. The old EEPROM values are backed up first and restored if anything fails.

:::{note}
`calib_neck.py` currently reads the keyboard through Windows' `msvcrt` module. On Linux or macOS, adapting it is a
welcome contribution.
:::

**Zeros you cannot see well.** The reference robot fixed two zeros afterwards, in the JSON file only:
- **J3** from the fold contact: in the folded pose the wrist touches the upper arm, which the model places at
  +185.1°.
- **J2 / J6** from the D435i IMU: the measured gravity direction was compared with the model in 11 poses (residual
  about 1°). The script is `projects/sim_mirror/scripts/imu_fit.py`.

## Arms: from the standard rest pose

Put both arms in the standard rest pose:
- shoulder_pan 0;
- shoulder_lift on its fold stop (+187°);
- elbow +180°;
- wrist_flex +50°;
- wrist_roll +90° (wrist camera on top);
- gripper closed.

`hardware/wallerobot_v0_1/calib_arm_rest.py` computes the zeros from one encoder read in that pose. Check each
joint's direction in the read-only mirror: move the arm by hand and watch the model follow.

:::{warning}
On the SO101, **shoulder_lift and elbow_flex have no EEPROM limit, and their encoder wrap (4095 → 0) lies inside
the travel.** Readers unwrap them, but **position control would take the long way round into the stop**. Re-centre
their `Homing_Offset` before you command them. The plugin refuses to until then.
:::
