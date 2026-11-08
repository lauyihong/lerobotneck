# 硬件

wallerobot 用 XLeRobot 的零件搭成。如果你已经有一台 XLeRobot：机身、立柱和两条 SO101 从动臂都保留，不装移动底盘，再把 2 自由度的头换成 6 自由度的脖子。

```{toctree}
:maxdepth: 1

bom
wiring
calibration
safety
```

## 结构

| 部件 | 说明 |
|---|---|
| 机身和桁架立柱 | XLeRobot 机身（下半部分带两个手臂安装台）和它的桁架立柱，直接放在桌上 |
| 手臂 | 2 条 SO101 从动臂，装在机身的安装台上，**都朝前**（在机身坐标系里绕 z 转 −90°） |
| 脖子 | 改造的 SO101，共 6 个电机：J1 底座水平转，J2 抬升，J3 肘，**J4 左右侧摆**（新零件），J5 相机横滚，J6 相机俯仰 |
| 头部相机 | Intel RealSense D435i，装在上游 XLeRobot 的头部俯仰支架里，挂在脖子的 J6 上 |
| 手腕相机 | 每个夹爪上一个 USB 相机，用 XLeRobot 的手腕相机支架 |

### 脖子

脖子就是一条 SO101 机械臂，原来装夹爪的地方改成了相机支架：

```text
机身 ─固定─▶ neck_base（立柱顶上）
  ─J1 水平转─▶ 肩 ─J2 抬升─▶ 上臂 ─J3 肘─▶ 新零件
  ─J4 侧摆─▶ 腕 ─J5 相机横滚─▶ J6 外壳 ─J6 相机俯仰─▶ D435i 支架
```

- J1–J3，以及从 J4 到 J6 的腕部，都是原装 SO101 零件。
- **J3→J4 的新零件**（`hardware/neck_v0_1/wallerobot_new_wrist_dof_0_1.stl`）让 J4 的轴垂直于 J3；原装 SO101 里这两个轴是平行的。它把 J4 沿连杆方向偏移 61.2 mm，向侧面偏移 16.85 mm。
- 折叠的 **home 姿态**下，J2 和 J3 靠在机械折叠限位上，相机朝前。

```{image} /../_static/img/robot_home.png
:alt: home 姿态，脖子折叠
:width: 600px
```

## 坐标系和命名

- **根坐标系 `wallerobot_body`** 就是 XLeRobot 的 `top_base_link`。机器人**前方是 −x**，**左臂在 −y 一侧**，z 朝上。
- **关节名**和 LeRobot 插件里的电机名一致：
  - 手臂：`left_arm_{shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper}`，右臂把前缀换成 `right_arm_`；
  - 脖子：`neck_j1` … `neck_j6`。
- **相机坐标系**：
  - `neck_camera_color_optical_frame`（x 向右，y 向下，z 向前）；
  - `left_wrist_camera`、`right_wrist_camera`。

带限位和标定数据的完整关节表见[](../model/index.md)。
