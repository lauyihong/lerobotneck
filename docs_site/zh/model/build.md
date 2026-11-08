# 生成流程

```bash
pip install -r requirements-model.txt     # numpy、scipy、trimesh、mujoco、pyyaml
python hardware/build_all.py              # 重新生成所有文件
python hardware/build_all.py --check      # 重新生成，并在结果和提交的文件不一致时报错
python hardware/build_all.py --config v.yaml --out build/v   # 生成一个变体（只有 URDF 和 MJCF）
```

`build_all.py` 依次执行以下 5 步。每个脚本也可以单独运行，都支持 `--config` 参数。

| # | 脚本 | 输出 |
|---|---|---|
| 1 | `hardware/neck_v0_1/build_urdf.py` | `hardware/neck_v0_1/urdf/wallerobot_neck.urdf`：脖子和机身 |
| 2 | `hardware/wallerobot_v0_1/build_full_urdf.py` | `hardware/wallerobot_v0_1/urdf/wallerobot_static.urdf`：加上双臂、手腕相机、夹爪末端点、凸包 |
| 3 | `projects/sim_mirror/scripts/build_mjcf.py` | `projects/sim_mirror/models/{wallerobot_static,scene}.xml`：先用 MuJoCo 导入 URDF，再加执行器、相机、末端点，以及休息位的接触排除 |
| 4 | `hardware/wallerobot_v0_1/joint_table.py --write` | `hardware/wallerobot_v0_1/JOINTS.md` |
| 5 | `projects/web_sim/scripts/export_assets.py` | `projects/web_sim/web/public/model/`（只针对双臂机器人） |

规则：
- **生成的文件**开头有 `GENERATED` 注释。要和产生它们的配置改动放在同一个提交里。CI 会运行 `build_all.py --check`，过期的生成文件会被拒绝。
- **碰撞凸包**（`*.hull.stl`）在源网格变化时会重新生成，生成记录保存在 `urdf/meshes/_hull_sources.json`。
- **上游文件**（XLeRobot 的 URDF 和网格）原样复制在 `hardware/upstream/xlerobot/`，对应的上游 commit 写在 `SOURCE.md` 里。所以 clone 下来不需要子模块就能生成。

## 检查结果

```bash
python projects/sim_mirror/scripts/check_mjcf.py   # MJCF 对比 URDF：坐标系、轴、限位、网格、相机、质量
python projects/sim_mirror/scripts/sim_check.py    # 物理冒烟测试：保持姿态、跟踪正弦、检查 NaN
```

`check_mjcf.py` 自己计算 URDF 的正运动学，不依赖 MuJoCo 的导入器。它在三类姿态下比对坐标系和网格包围盒：q = 0、休息位、30 个随机姿态。
