# Build pipeline

```bash
pip install -r requirements-model.txt     # numpy, scipy, trimesh, mujoco, pyyaml
python hardware/build_all.py              # rebuild every generated file
python hardware/build_all.py --check      # ... and fail if anything differs from the committed files
python hardware/build_all.py --config v.yaml --out build/v   # a variant (URDF + MJCF only)
```

`build_all.py` runs five steps. Each script can also be run on its own and accepts `--config`.

| # | script | output |
|---|---|---|
| 1 | `hardware/neck_v0_1/build_urdf.py` | `hardware/neck_v0_1/urdf/wallerobot_neck.urdf`: neck + body |
| 2 | `hardware/wallerobot_v0_1/build_full_urdf.py` | `hardware/wallerobot_v0_1/urdf/wallerobot_static.urdf`: adds both arms, wrist cameras, jaw tips, convex hulls |
| 3 | `projects/sim_mirror/scripts/build_mjcf.py` | `projects/sim_mirror/models/{wallerobot_static,scene}.xml`: MuJoCo import, then actuators, cameras, sites, and contact excludes at the rest pose |
| 4 | `hardware/wallerobot_v0_1/joint_table.py --write` | `hardware/wallerobot_v0_1/JOINTS.md` |
| 5 | `projects/web_sim/scripts/export_assets.py` | `projects/web_sim/web/public/model/`: only for the two-arm robot |

**Rules**
- **Generated files** start with a `GENERATED` comment. Commit them together with the config change that produced
  them. CI runs `build_all.py --check` and rejects stale files.
- **Collision hulls** (`*.hull.stl`) are rebuilt when their source mesh changes. The record is kept in
  `urdf/meshes/_hull_sources.json`.
- **Upstream files** (the XLeRobot URDF and meshes) are copied unmodified into `hardware/upstream/xlerobot/`, with
  their commit in `SOURCE.md`, so a clone builds without submodules.

## Checking the result

```bash
python projects/sim_mirror/scripts/check_mjcf.py   # MJCF vs URDF: frames, axes, limits, meshes, cameras, mass
python projects/sim_mirror/scripts/sim_check.py    # physics smoke test: holds a pose, tracks a sinusoid, no NaN
```

`check_mjcf.py` computes the URDF forward kinematics itself, without MuJoCo's importer. It compares frames and
mesh bounds at q = 0, at the rest pose and at 30 random poses.
