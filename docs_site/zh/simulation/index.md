# 仿真

有三种方式仿真 wallerobot，用的都是同一个生成出来的模型：

| | 在哪里 | 需要什么 |
|---|---|---|
| **网页模拟器** | {site}`sim/` | 浏览器（手机也可以） |
| **MuJoCo 查看器** | `projects/sim_mirror/scripts/view.py` | `requirements-model.txt` |
| **虚实镜像** | `projects/sim_mirror/scripts/mirror.py` | 真机（只读）+ LeRobot |

## 网页模拟器

MuJoCo 3.16 以 WebAssembly 的形式在网页里运行，画面由 three.js 绘制。它是纯仿真，不会向外发送任何数据，也从不连接硬件。

**操作方式**
- **键盘**：沿 x、y、z 移动每条臂的夹爪，调整手腕俯仰和横滚，开合夹爪。脖子有单独的按键。按 `?` 可以看到完整的按键表。

  | | 左臂 | 右臂 |
  |---|---|---|
  | 前 / 后 | W / S | I / K |
  | 左 / 右 | A / D | J / L |
  | 上 / 下 | Q / E | U / O |
  | 手腕俯仰 | R / F | Y / H |
  | 手腕旋转 | Z / C | N / M |
  | 夹爪 | V | B |

  其他按键：
  - 脖子：←/→ 转头（J1），↑/↓ 相机俯仰（J6）；
  - 单个关节：`[` / `]` 选关节，`,` / `.` 转动它；
  - `G` 准备姿势，`X` 回 home，空格播放 / 暂停路线。
- **鼠标**：
  - 点一个部件，就选中它的关节；
  - 点手臂，夹爪上会出现三轴拖动手柄，逆运动学会跟着手柄走；
  - 也可以直接输入以毫米为单位的 xyz 目标。
- **路线**：
  - 自带几条固定示范：脖子、双臂、"看着手"；路点之间用最小加加速度曲线插值。
  - 也可以一个姿态一个姿态地录制自己的路线，然后播放、保存，或者导入 JSON 路线。
- **相机小窗**：显示脖子上 D435i 看到的画面（垂直视场角 43.2°）。

关于物理和逆运动学：
- 重力补偿模拟真机舵机那种很硬的位置保持。
- 逆运动学用阻尼最小二乘法，目标是两个指尖之间的中点。
- 求解时让肘部保持弯曲，避免手臂穿过奇异点时翻转。

## MuJoCo 查看器（Python）

```bash
python projects/sim_mirror/scripts/view.py                 # 休息位，运动学模式（滑块就是关节角）
python projects/sim_mirror/scripts/view.py --pose zero     # 所有关节为 0
python projects/sim_mirror/scripts/view.py --physics       # 跑物理，使用位置执行器
python projects/sim_mirror/scripts/view.py --collision     # 显示碰撞凸包
python projects/sim_mirror/scripts/view.py --snapshot x.png   # 不开窗口：输出 4 个视角加脖子相机的图
```

`projects/sim_mirror/scripts/sim_robot.py` 里的 `SimWallerobot` 提供一个类似插件的 Python 接口，方便脚本和策略调用：`get_observation()`、`send_action()`、相机图像。

## 虚实镜像（只读）

`mirror.py` 以大约 70 Hz 读取真机的 18 个编码器，让模型摆成同样的姿态。窗口上排是真实相机画面，下排是同一关节状态下的仿真画面。

它只**读**寄存器，并检查扭矩始终是关着的，所以接上一台新机器人时，最适合先运行它。可以用它来验证标定：用手转动一个关节，模型应该朝同一个方向跟着动。

```bash
python projects/sim_mirror/scripts/mirror.py --left-wrist 0 --right-wrist 2
```
