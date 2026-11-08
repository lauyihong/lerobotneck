# 配置参考：`robot.yaml`

生成模型用到的所有几何和仿真数字都在 `hardware/wallerobot_v0_1/robot.yaml` 里。生成脚本本身不含机器人参数，只有注释说明每个默认值的来源。

约定：
- **单位**：米和弧度。键名以 `_mm` 或 `_deg` 结尾的除外。
- **路径**：相对仓库根目录。
- **坐标系**：`wallerobot_body`。机器人前方是 −x，左臂在 −y 一侧，z 朝上。

## 各部分

| 键 | 控制什么 |
|---|---|
| `upstream.urdf`, `upstream.meshes` | 上游 XLeRobot 的 URDF 和网格：SO101 的关节坐标系、限位、惯量（放在 `hardware/upstream/xlerobot/`） |
| `calibration.neck`, `calibration.arms` | 这台机器人的标定文件：决定 URDF 限位、home / 休息位，以及 tick 和角度之间的换算 |
| `joint_defaults` | URDF 里 `<limit>` 的 effort 和 velocity |
| `arms.footprint_centre` | SO101 底座圆在 Base 坐标系里的圆心 |
| `arms.<side>.enabled` | 生成还是去掉这条臂（单臂机器人） |
| `arms.<side>.upstream_chain` | 用上游哪条运动链（`L` / `R`）提供运动学 |
| `arms.<side>.mount.xyz`, `.yaw_deg` | 底座圆落在机身安装台上的位置，以及绕 z 的转角（−90 = 朝前） |
| `arms.<side>.color` | 手臂在模型里的 RGBA 颜色 |
| `neck.mount.xyz`, `.rpy_deg` | 脖子底座（J1）在立柱顶上的位姿 |
| `neck.link34.stl` | J3→J4 连杆的网格文件 |
| `neck.link34.j3_axis_mm`, `.j4_shaft_mm` | 网格上 J3 轴心的点和 J4 电机末端的点，用 STL 自己的坐标系表示；这两个点决定连杆的长度和偏移 |
| `neck.link34.side`, `.j4_shaft_sign` | 连杆和 J4 电机的方向开关（±1） |
| `neck.j5_zero_shift_deg` | 平移 J5 的零位（−90 = 相机正立，在 J6 外壳下方） |
| `neck.camera_holder.flip`, `.camera_flip`, `.axial_shift_mm` | D435i 支架装在 J6 上的方式 |
| `neck.placeholder_limit_rad` | 标定文件里没有 J4 / J6 时才用的占位限位 |
| `cameras.head` | D435i 在支架里的偏移、图像尺寸、垂直视场角 |
| `cameras.wrist` | 手腕相机的图像尺寸、视场角、下倾角（估计值） |
| `sim.*` | MJCF 的时间步长、执行器增益和力矩上限、阻尼、armature、桌面高度，以及用来排除接触的休息位 |

## 覆盖文件

覆盖文件只需要写要改的键。它会被深度合并到 `robot.yaml` 上：字典按键逐层合并，列表和单个值直接替换。

```yaml
# hardware/variants/single_arm_left.yaml
arms:
  right:
    enabled: false
```

使用方式有两种：
- 加 `--config` 参数（见[](build.md)）；
- 设置环境变量 `WALLEROBOT_CONFIG`。这种方式对通过 `projects/sim_mirror/scripts/calib.py` 读取标定的脚本也同样生效。

## 完整文件

```{literalinclude} ../../../hardware/wallerobot_v0_1/robot.yaml
:language: yaml
```
