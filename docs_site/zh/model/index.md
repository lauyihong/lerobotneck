# 机器人模型

URDF、MuJoCo 模型和网页模型都是**生成**出来的。来源有三个：
- 配置文件 {repo}`hardware/wallerobot_v0_1/robot.yaml`；
- 标定文件；
- 仓库里附带的上游 XLeRobot URDF 和网格。

```{toctree}
:maxdepth: 1

config
customize
build
```

## 文件

| 文件 | 说明 |
|---|---|
| `hardware/wallerobot_v0_1/urdf/wallerobot_static.urdf` + `meshes/` | 整机 URDF：18 个转动关节、可见网格、凸包碰撞网格 |
| `hardware/neck_v0_1/urdf/wallerobot_neck.urdf` | 只有脖子（连同机身）的 URDF |
| `projects/sim_mirror/models/wallerobot_static.xml` | MuJoCo 模型：18 个位置执行器，相机 `neck_color`、`left_wrist`、`right_wrist`，夹爪末端点 |
| `projects/sim_mirror/models/scene.xml` | 带桌面和灯光的场景 |
| `projects/web_sim/web/public/model/` | 网页用的副本：用到的网格、IK 目标点 `left_ee`/`right_ee`、`poses.json` |
| `hardware/wallerobot_v0_1/JOINTS.md` | 下面的关节表 |

## 姿态

```{image} /../_static/img/robot_zero.png
:alt: 全部关节为 0
:width: 600px
```

*所有关节为 0 时，手臂和脖子都向前伸直。* **home**（折叠，见[](../hardware/index.md)）和 **准备姿势**（脖子抬起、手臂举起）的图在其他页面。

## 关节表

由 `hardware/wallerobot_v0_1/joint_table.py` 根据标定文件和 URDF 生成。*bus / ID* 指舵机总线：总线 1 在参考机上是 `COM3`，总线 2 是 `COM4`。

```{include} ../../../hardware/wallerobot_v0_1/JOINTS.md
:start-line: 2
```

## 建模选择

| 项目 | 做法 |
|---|---|
| 手臂 | 上游 XLeRobot 的 SO101 运动链（关节坐标系、轴、限位、惯量）；限位再和标定的电机行程取交集 |
| 脖子 J1–J3 和 J4–J6 腕部 | 上游 SO101 的坐标系 |
| 脖子 J3→J4 新零件 | 由它的 STL 决定：`robot.yaml` 里的两个实测点和方向开关 |
| 碰撞 | 每个可见网格的凸包（比较粗）；夹爪用上游的凸包件；折叠休息位里本来就接触的部件对，排除在碰撞检测之外 |
| 驱动（MJCF） | 位置执行器 kp 20 N·m/rad，±3.35 N·m，关节阻尼 1，armature 0.01 |
| 惯量 | 手臂用上游的值；脖子是占位值（每段 0.05 kg） |
| 相机 | D435i 彩色 640×480，垂直视场角 43.2°（用实测内参算的）；手腕相机 60°（估计值） |
