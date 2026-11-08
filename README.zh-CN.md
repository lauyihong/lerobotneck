# wallerobot

[English](README.md) · **中文**

wallerobot 是一套开源、**可配置**的机器人描述，对应一台固定式的 XLeRobot 变体：
- 两条 SO101 机械臂；
- 一个 6 自由度的"脖子"，末端装 Intel RealSense D435i；
- **没有移动底盘**。

URDF、MuJoCo 模型和网页模拟器都从同一个 YAML 文件生成。想改机器人，就改这个文件，不用去改仿真器的源码。

- **在线试玩：** [网页模拟器](https://lauyihong.github.io/wallerobot/sim/)。MuJoCo 直接在浏览器里运行，不用安装任何东西。
- **文档：** [https://lauyihong.github.io/wallerobot/zh/](https://lauyihong.github.io/wallerobot/zh/)，有中文和英文两种。

| | |
|---|---|
| 自由度 | 18 个：两条 SO101 臂各 6 个（含夹爪），脖子 6 个 |
| 执行器 | 18 个飞特 STS3215（12 V），两条 1 Mbps 总线 |
| 相机 | 脖子上的 RealSense D435i（RGB-D 和 IMU），两个手腕各一个 USB 相机 |
| 模型输出 | URDF、MuJoCo MJCF（位置执行器、3 个相机）、网页模型 |
| 软件 | LeRobot 机器人插件 `--robot.type=xlerobot_static` |

## 和 XLeRobot 的区别
[XLeRobot](https://github.com/Vector-Wangel/XLeRobot) 是一台移动式双臂机器人。wallerobot 保留了它的机身、立柱和 SO101 机械臂，改了三处：
- **去掉移动底盘**，机身直接放在桌上。
- **用 6 自由度脖子代替原来 2 自由度的头**。脖子是改造过的 SO101：J3 和 J4 之间换成一个新零件，让 J4 变成左右侧摆。
- **两条臂朝前安装**，和主相机朝向一致。

模型由配置生成。手臂安装位置、脖子几何、相机和仿真参数都写在 [`hardware/wallerobot_v0_1/robot.yaml`](hardware/wallerobot_v0_1/robot.yaml) 里。

## 生成模型

只需要 Python，不需要子模块，也不需要整套机器人学习环境。

```bash
pip install -r requirements-model.txt
python hardware/build_all.py              # 从 robot.yaml 生成 URDF、MJCF、关节表和网页模型
python hardware/build_all.py --check      # 重新生成，并确认生成文件没有变化
```

生成一个变体，不会影响参考模型：

```bash
python hardware/build_all.py --config hardware/variants/single_arm_left.yaml --out build/single_arm_left
```

各个输出文件的位置，以及仓库结构，见[英文 README](README.md)。

## 状态
这是 v0.1，一个业余项目的第一个公开版本。目前验证过的情况：
- 脖子已经在真机上带扭矩运行过（手臂没有一起动）。
- 手臂只读过编码器，还没有驱动过。要先把编码器的跳变点重新居中，才能发位置指令。
- 网页模拟器和模型都只是仿真。

已知的局限都写在文档里：
- 夹爪几何是占位模型；
- 手腕相机的视场角是估计值；
- 手臂安装位置取自 CAD，没有实测。

## 许可证
[Apache-2.0](LICENSE)。本项目基于 XLeRobot、LeRobot 和 SO-ARM100/101 的设计，它们都使用 Apache-2.0，详见 [NOTICE](NOTICE)。
