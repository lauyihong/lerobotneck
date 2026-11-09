# 快速开始

按你想做的事选一条路。从上到下，需要准备的东西越来越多。

## 1. 在浏览器里看机器人

打开{site}`网页模拟器 <sim/>`，不用安装，手机上也能用。
- 按 `?` 查看按键说明。
- 点机器人的某个部件，就选中对应的关节。
- 点手臂，夹爪上会出现拖动手柄。
- 在「固定路线」里可以播放脖子和双臂的示范。

## 2. 在自己电脑上生成模型

只需要 Python 3.10 或更新版本，不需要整套机器人学习环境。

```bash
git clone https://github.com/lauyihong/lerobotneck.git
cd lerobotneck
pip install -r requirements-model.txt
python hardware/build_all.py --check
```

`--check` 会从 `hardware/wallerobot_v0_1/robot.yaml` 重新生成所有文件（URDF、MJCF、关节表、网页模型）。结果和仓库里提交的一致时，会输出 `CHECK OK`。

用 MuJoCo 查看器打开模型：

```bash
python projects/sim_mirror/scripts/view.py --pose home
```

接下来可以看[](model/customize.md)，做一个自己的变体。

## 3. 在本地运行网页模拟器

```bash
cd projects/web_sim/web
npm ci
npm run dev        # http://localhost:5180
```

## 4. 连接真机

这条路需要实物硬件，还要完整安装 [LeRobot](https://github.com/huggingface/lerobot)。请先读[](hardware/safety.md)，然后：
1. 设置舵机 ID（[](hardware/wiring.md)）；
2. 标定（[](hardware/calibration.md)）；
3. 安装插件（[](software/index.md)），先用只读模式连接。
