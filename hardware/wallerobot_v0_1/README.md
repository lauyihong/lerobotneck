# wallerobot_v0_1 — full static robot description (body + dual SO101 arms + neck v0.1)

Overview: [docs/robot_design.md](../../docs/robot_design.md). Joint table (generated): [JOINTS.md](JOINTS.md). Design snapshots: `archive/`.

```
.venv/Scripts/python.exe hardware/wallerobot_v0_1/seed_arm_calibration.py      # once: arm_calibration.json (seed)
.venv/Scripts/python.exe hardware/wallerobot_v0_1/build_full_urdf.py           # urdf/wallerobot_static.urdf + meshes/
.venv/Scripts/python.exe hardware/wallerobot_v0_1/render_full.py --pose projects/sim_mirror/outputs/rest_pose_20261105.json --out x.png
.venv/Scripts/python.exe projects/sim_mirror/scripts/real_reader.py --json pose.json   # READ-ONLY encoder read (18 joints)
.venv/Scripts/python.exe hardware/wallerobot_v0_1/joint_table.py --write              # JOINTS.md + docs/robot_design.md section 4
```

| item | source / status |
|---|---|
| neck, body (truss tower + J1 housing), D435i | `hardware/neck_v0_1/build_urdf.py` (re-generated, unchanged; see its README) |
| arms | upstream `third_party/XLeRobot/simulation/xlerobot/xlerobot.urdf` SO101 chains; upstream `_L` (y = −0.11, robot forward = −X) = our **left** arm (COM3). Upstream names its camera "Right_Arm_Camera" → our `left_wrist_camera` |
| arm mounts | **both arms face FORWARD** (user 2026-11-05; upstream simulation/xlerobot.urdf mounts them outwards — wrong for this robot): Base yaw −90° in the body frame. SO101 Base footprint circle (r 56.2 mm) centred on the body pad at (−90, ∓132.9) mm, pad top z 48 mm (`base_body_lower.stl`); cross-checked with the XLeRobot 0.4.0 armbase CAD (`third_party/XLeRobot/hardware/step/XLeRobot_040/XLeRobot040_armbase.step`, circles r 56.5 at tower-relative (−2.2, ±136.5) mm, within 4 mm). Not measured on the real body |
| joint names | = plugin motor names: `left_arm_{shoulder_pan,shoulder_lift,elbow_flex,wrist_flex,wrist_roll,gripper}`, `right_arm_*`, `neck_j1..6` |
| arm tick ↔ rad | `arm_calibration.json` (same convention as `neck_calibration.json`). **Zeros from the standard rest pose** (user placed both arms, photo `projects/sim_mirror/outputs/photo_full_20261105.jpg`) via `calib_arm_rest.py`: rest = pan 0, lift +187 (fold stop, same as neck J2 rest), elbow +180 (model contact 180.75), wrist_flex +50, wrist_roll +90 (wrist camera on top), gripper closed. Signs: lift −1 / elbow +1 from `projects/handover_test/outputs/ik_calib` tick directions; pan / wrist_flex / wrist_roll +1 **verified by the user in the mirror** (2026-11-05). Old seed kept in `arm_calibration.seed.json` |
| limits | neck: neck_calibration.json; arms: upstream limits ∩ calibrated range through the seed mapping (lift/elbow have no motor limit → upstream) |
| grippers | real = **parallel** grippers (2026-10-17/18), no CAD in the workspace → stock SO101 jaws as placeholder; tick → Jaw 0..1.7 rad linear (closed..open) |
| collision | upstream convex jaw parts; otherwise `*.hull.stl` = convex hull of each visual mesh (coarse) |
| frames | `left/right_wrist_camera` (upstream camera link), `*_arm_{fixed,moving}_jaw_tip` (ManiSkill tip offsets), `neck_camera_color_optical_frame` |
| inertia | arms: upstream values; neck: placeholders |

Previews: `preview_q0.png` (all zeros: arms and neck stretched forward), `preview_rest.png` (real encoder read 2026-11-05, read-only), `projects/sim_mirror/outputs/compare_rest_photo.png` (photo vs model, same view).

| 2026-11-06 | |
|---|---|
| neck zeros | J2 2942→3022, J6 2219→2115 from the torque-demo IMU fit (see neck_v0_1/README.md) |
| arm readings | lift / elbow unwrapped (`wrap_lo` window, encoder wrap lies inside their travel); arm torque refused by the plugin until re-centred |
