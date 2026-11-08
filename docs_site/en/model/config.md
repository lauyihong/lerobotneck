# Configuration reference: `robot.yaml`

Every geometric and simulation number of the generated model lives in
`hardware/wallerobot_v0_1/robot.yaml`. The build scripts contain no robot constants, only comments that say where
each default value came from.

**Conventions**
- **Units:** metres and radians, unless a key ends in `_mm` or `_deg`.
- **Paths** are relative to the repository root.
- **Frame:** `wallerobot_body`, with robot forward = −x, left arm at −y, z up.

## Sections

| key | what it controls |
|---|---|
| `upstream.urdf`, `upstream.meshes` | upstream XLeRobot URDF and meshes: SO101 joint frames, limits, inertias (vendored in `hardware/upstream/xlerobot/`) |
| `calibration.neck`, `calibration.arms` | the robot's calibration files; they give the URDF limits, the home/rest poses and the tick ↔ angle mapping |
| `joint_defaults` | URDF `<limit effort velocity>` |
| `arms.footprint_centre` | centre of the SO101 Base footprint circle in the Base frame |
| `arms.<side>.enabled` | build this arm or leave it out (a single-arm robot) |
| `arms.<side>.upstream_chain` | which upstream chain (`L` / `R`) provides the kinematics |
| `arms.<side>.mount.xyz`, `.yaw_deg` | where the Base footprint circle sits on the body pad, and the rotation about z (−90 = facing forward) |
| `arms.<side>.color` | RGBA of the arm in the model |
| `neck.mount.xyz`, `.rpy_deg` | pose of the neck base (J1) on top of the tower |
| `neck.link34.stl` | mesh of the J3→J4 link |
| `neck.link34.j3_axis_mm`, `.j4_shaft_mm` | the J3 axis point and the J4 motor end on that mesh, in the STL's own frame; together they define the link length and offset |
| `neck.link34.side`, `.j4_shaft_sign` | orientation flags of the link and of the J4 motor (±1) |
| `neck.j5_zero_shift_deg` | shifts the J5 zero (−90 = camera upright below the J6 housing) |
| `neck.camera_holder.flip`, `.camera_flip`, `.axial_shift_mm` | how the D435i holder sits on J6 |
| `neck.placeholder_limit_rad` | J4/J6 limits used only when the calibration file has no entry |
| `cameras.head` | D435i offset in its holder, image size, vertical FOV |
| `cameras.wrist` | wrist camera image size, FOV, pitch (estimated) |
| `sim.*` | MJCF timestep, actuator gain and force limit, damping, armature, desk height, rest pose used for contact excludes |

## Override files

An override file lists only the keys it changes. It is deep-merged over `robot.yaml`: mappings merge key by key,
while lists and scalars are replaced.

```yaml
# hardware/variants/single_arm_left.yaml
arms:
  right:
    enabled: false
```

Use it with `--config` (see [](build.md)), or set the environment variable `WALLEROBOT_CONFIG`. The second way also
reaches scripts that read the calibration through `projects/sim_mirror/scripts/calib.py`.

## The full file

```{literalinclude} ../../../hardware/wallerobot_v0_1/robot.yaml
:language: yaml
```
