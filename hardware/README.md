# hardware/ — robot description of the wallerobot

Human overview (Chinese): [docs/robot_design.md](../docs/robot_design.md). Machine-readable sources:

| path | what |
|---|---|
| `hardware.yaml` | ports, motor IDs, cameras, bench findings, todo — **source of truth** for the wiring |
| `wallerobot_v0_1/` | **current full robot**: `urdf/wallerobot_static.urdf` (+ meshes), `arm_calibration.json`, `JOINTS.md` (generated joint table), build / render / calibration scripts, `archive/` snapshots |
| `wallerobot_v0_1/robot.yaml` | **configuration** of the generated model (see below) |
| `upstream/xlerobot/` | vendored upstream URDF + meshes (Apache-2.0, `SOURCE.md`) |
| `variants/` | example override files |
| `neck_v0_1/` | the 6-DoF neck: CAD (new J3-J4 link STL, body meshes), `build_urdf.py`, `neck_calibration.json` (zeros, ranges, home pose, history), photos, `calib_backups/`, `archive/` |

## Configure and rebuild

All robot numbers live in **`wallerobot_v0_1/robot.yaml`**: arm mounts, which arms exist, the neck mount, the new-link
geometry and orientation flags, camera mounts and intrinsics, simulation drive numbers, and the calibration files.
The builders contain no robot constants; they only have comments on where the defaults came from.

```bash
python hardware/build_all.py            # robot.yaml -> every generated file (URDF, MJCF, JOINTS.md, web model)
python hardware/build_all.py --check    # same, then fail if a committed generated file changed (CI)
python hardware/build_all.py --config hardware/variants/arms_out_2cm.yaml --out build/arms_out_2cm   # a variant
```

- **Override files.** An override (`--config`, or env `WALLEROBOT_CONFIG`) only lists the keys it changes. It is
  deep-merged over `robot.yaml`. Examples are in `variants/`: `arms_out_2cm.yaml` and `single_arm_left.yaml`.
- **Dependencies.** Only `requirements-model.txt` is needed: numpy, scipy, trimesh, mujoco and pyyaml. The upstream
  XLeRobot URDF and meshes are vendored in `upstream/xlerobot/` (see `SOURCE.md`), so the submodules are not needed.
- **Collision hulls** are regenerated when their source mesh changes. The record is kept in
  `wallerobot_v0_1/urdf/meshes/_hull_sources.json`.

`build_all.py` runs these steps in order; each step can also be run on its own:

```bash
python hardware/neck_v0_1/build_urdf.py                # neck-only URDF
python hardware/wallerobot_v0_1/build_full_urdf.py     # full URDF + meshes + hulls
python projects/sim_mirror/scripts/build_mjcf.py       # MuJoCo model + scene, rest-pose contact excludes
python hardware/wallerobot_v0_1/joint_table.py --write
python projects/web_sim/scripts/export_assets.py       # browser model
```

Rules: the neck EEPROM is written only by `tools/calib_neck.py`; calibration fixes are file-only (`zero_tick`), with a backup
in `neck_v0_1/calib_backups/` first. Upstream CAD stays read-only in `third_party/XLeRobot`; the files the builders need are copied, unmodified, into `upstream/xlerobot/`.
