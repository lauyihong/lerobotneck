# Parts (changes from XLeRobot)

Start from the [XLeRobot bill of materials](https://xlerobot.readthedocs.io/en/latest/hardware/getting_started/material.html)
and apply these changes.

| item | XLeRobot | wallerobot | note |
|---|---|---|---|
| STS3215 servos (12 V) | 17 (12 arm + 3 base + 2 head) | **18** (12 arm + 6 neck) | no wheel motors; the neck needs 6 |
| servo driver boards | 2 | 2 | bus 1: left arm + neck (12 servos), bus 2: right arm |
| mobile base (wheels, cart, battery) | yes | **no** | the body stands on a desk; it runs on a 12 V bench supply |
| head camera | RealSense / stereo | **Intel RealSense D435i** | RGB, depth and IMU; the IMU is used to calibrate the camera orientation |
| wrist cameras | 2 USB | 2 USB | as in XLeRobot |
| grippers | SO101 jaw | parallel grippers | the model still uses the stock SO101 jaw meshes |
| neck structure | 2-DoF head parts | SO101 arm parts (base, shoulder, upper arm, wrist) + **the new J3→J4 link** + the XLeRobot head-tilt holder | printed parts below |

## Printed parts for the neck

| file | what it is |
|---|---|
| `hardware/neck_v0_1/wallerobot_new_wrist_dof_0_1.stl` | **new link between J3 and J4.** It turns J4 into a side swing. |
| `hardware/neck_v0_1/cam_tilt_housing_softfin.stl` | J6 motor housing: the XLeRobot soft-finger fixed jaw with the finger removed |
| SO101 parts | base, rotation pitch, upper arm and wrist pitch/roll: print them as for an SO101 follower arm |
| XLeRobot head-tilt holder | holds the D435i on J6; mounted mirrored compared with the old head |

The body meshes in `hardware/neck_v0_1/` (`base_*.stl`) come from the XLeRobot CAD. They are there for the model and
are not new prints.

:::{note}
Some neck dimensions in the model are assumptions until someone measures them, for example the J4 motor insertion
depth in the new link. Measurements are welcome, see [](../community/index.md).
:::

## Power

The 12 V version of the STS3215 is used, at about 12.2–12.5 V. Bus 1 drives 12 servos (left arm + neck), so check
that your supply can deliver the current when several joints hold torque at once.
