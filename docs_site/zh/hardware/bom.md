# 零件清单（相对 XLeRobot 的改动）

以 [XLeRobot 物料清单](https://xlerobot.readthedocs.io/en/latest/hardware/getting_started/material.html)为基础，做下面这些改动。

| 项目 | XLeRobot | wallerobot | 说明 |
|---|---|---|---|
| STS3215 舵机（12 V） | 17 个（手臂 12，底盘 3，头 2） | **18 个**（手臂 12，脖子 6） | 没有轮子电机，脖子要 6 个 |
| 舵机驱动板 | 2 块 | 2 块 | 总线 1：左臂和脖子（12 个舵机）；总线 2：右臂 |
| 移动底盘（轮子、小车、电池） | 有 | **没有** | 机身直接放在桌上，用 12 V 台式电源供电 |
| 头部相机 | RealSense 或双目 | **Intel RealSense D435i** | RGB、深度和 IMU；IMU 用来标定相机朝向 |
| 手腕相机 | 2 个 USB | 2 个 USB | 和 XLeRobot 相同 |
| 夹爪 | SO101 夹爪 | 平行夹爪 | 模型里暂时还是原装 SO101 夹爪的网格 |
| 脖子结构 | 2 自由度头部零件 | SO101 臂的零件（底座、肩、上臂、腕）+ **J3→J4 新零件** + XLeRobot 头部俯仰支架 | 打印件见下表 |

## 脖子的打印件

| 文件 | 说明 |
|---|---|
| `hardware/neck_v0_1/wallerobot_new_wrist_dof_0_1.stl` | **J3 和 J4 之间的新零件**，让 J4 变成侧摆 |
| `hardware/neck_v0_1/cam_tilt_housing_softfin.stl` | J6 电机外壳，就是 XLeRobot 软指夹爪的固定爪去掉手指 |
| SO101 零件 | 底座、肩（rotation pitch）、上臂、腕（wrist pitch/roll），按 SO101 从动臂的方式打印 |
| XLeRobot 头部俯仰支架 | 把 D435i 固定在 J6 上；和原来的头相比是镜像安装的 |

`hardware/neck_v0_1/` 里的机身网格（`base_*.stl`）来自 XLeRobot 的 CAD，只给模型用，不需要另外打印。

:::{note}
模型里有些脖子尺寸还是假设值，比如 J4 电机插进新零件的深度，等有人量过才能确定。欢迎提供实测数据，见[](../community/index.md)。
:::

## 供电

用的是 12 V 版的 STS3215，实际电压约 12.2–12.5 V。总线 1 带 12 个舵机（左臂和脖子），几个关节同时上扭矩保持时电流会比较大，请确认电源能提供足够的电流。
