# wallerobot

```{image} /../_static/img/robot_ready.png
:alt: wallerobot 模型，准备姿势
:class: hero
```

**wallerobot** 是一套开源、可配置的机器人描述，对应一台固定式的 [XLeRobot](https://github.com/Vector-Wangel/XLeRobot) 变体。它有两条 SO101 机械臂，加一个 6 自由度的"脖子"，脖子末端装着 Intel RealSense D435i；没有移动底盘。

URDF、MuJoCo 模型和网页模拟器都从同一个 YAML 文件生成。想改机器人，改这个文件里的数字就行，不用去改仿真器的源码。

::::{grid} 1 1 3 3
:gutter: 2

:::{grid-item-card} 试试模拟器
:link: simulation/index
:link-type: doc
MuJoCo 直接在浏览器里运行，不用安装。可以用键盘、鼠标或固定路线来控制。
:::

:::{grid-item-card} 配置模型
:link: model/customize
:link-type: doc
挪手臂、去掉一条臂、换脖子连杆，然后一条命令重新生成 URDF 和 MJCF。
:::

:::{grid-item-card} 搭建硬件
:link: hardware/index
:link-type: doc
和 XLeRobot 相比改了什么，以及舵机 ID、标定和安全规则。
:::
::::

## 概览

| | |
|---|---|
| 自由度 | 18 个：两条 SO101 臂各 6 个（含夹爪），脖子 6 个 |
| 执行器 | 18 个飞特 STS3215（12 V，每圈 4096 tick），两条 1 Mbps 串行总线 |
| 相机 | 脖子上的 RealSense D435i（RGB、深度、IMU），两个手腕各一个 USB 相机 |
| 模型 | URDF、MuJoCo MJCF（18 个位置执行器、3 个相机）、网页模型 |
| 软件 | LeRobot 机器人插件 `--robot.type=xlerobot_static` |

## 和 XLeRobot 的区别

- **没有移动底盘。** XLeRobot 的机身和桁架立柱直接放在桌上。
- **用 6 自由度脖子代替 2 自由度的头。** 脖子是改造过的 SO101：J3 和 J4 之间换成一个新零件，J4 的轴因此转了 90°，变成左右侧摆。D435i 装在上游 XLeRobot 的头部俯仰支架上。
- **两条臂都朝前安装**，和主相机朝向一致。
- **模型由配置驱动。** 手臂安装位置、脖子几何、相机和仿真参数都写在一个 YAML 文件里。CI 会检查生成结果，变体就是一个覆盖文件。

## 状态（v0.1）

:::{admonition} 验证过的部分
:class: note
- 休息位下，模型和真机的编码器读数一致。脖子相机的朝向用 D435i 的 IMU 拟合过，残差约 1°。
- 脖子在真机上带扭矩跑过一套姿态示范（J1、J2、J3、J5）。
- 手臂只读过、还没驱动过：shoulder_lift 和 elbow_flex 的编码器跳变点落在行程里面，要先重新居中，才能做位置控制。
:::

模型目前的已知局限：
- 夹爪用的是原装 SO101 的夹爪网格，真机用的是平行夹爪，它的 CAD 还没公开；
- 手腕相机的视场角是估计值，没有实测；
- 手臂安装位置取自 CAD，没有在真机上量过；
- J4 电机插入新连杆的深度是假设值；
- 脖子的惯量是占位值。

```{toctree}
:maxdepth: 2
:hidden:

getting_started
hardware/index
model/index
simulation/index
software/index
community/index
```
