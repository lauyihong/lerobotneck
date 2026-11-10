# Vendored upstream files: XLeRobot

These are unmodified copies of the upstream files the model builders need. With them in the repository, a fresh
clone can build the URDF and MJCF without the 387 MB `third_party/XLeRobot` submodule.

| here | upstream path (XLeRobot `b017b5e6354bd9f61f4247a920c72622ca0aade0`, 2026-10-06) |
|---|---|
| `xlerobot.urdf` | `simulation/xlerobot/xlerobot.urdf` |
| `meshes/*.stl` (24 files) | `simulation/xlerobot/meshes/xlerobot/assets/` |
| `LICENSE` | `LICENSE` (Apache-2.0) |

- **Upstream project:** XLeRobot (https://github.com/Vector-Wangel/XLeRobot) and its contributors. It is licensed under Apache-2.0.
- **SO101 parts:** the arm meshes (`Base`, `Rotation_Pitch`, `Upper_Arm`, `Lower_Arm`, `Wrist_Pitch_Roll`, `*_Jaw*` and the `*_Motor` parts) come from the SO-ARM100/101 design by TheRobotStudio, also Apache-2.0.

Which files are used:
- The builders use only the joint frames, axes, limits and inertias from `xlerobot.urdf`, and copy the meshes listed here.
- The SO101 arm meshes are shared by the arms and by the neck. The neck is a modified SO101.
- `topbase2.stl` is the old head-pan motor, used here as the neck J1 motor.
- `tophead5.stl` / `tophead6.stl` are the camera holder and the D435i body.
- `tophead4.stl` is the 2-axis head's pan bracket. Only the active-perception comparison on the website uses it
  (`projects/web_sim/scripts/make_coverage.py` draws the original XLeRobot head).

To refresh from a newer upstream, copy the same paths from the submodule, update the commit above, and rebuild with
`python hardware/build_all.py`. Check the result with `git diff` on the generated files.
