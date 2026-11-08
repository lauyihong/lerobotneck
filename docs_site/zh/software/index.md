# 软件：LeRobot 插件

`plugins/lerobot_robot_xlerobot_static` 会在 [LeRobot](https://github.com/huggingface/lerobot) 里注册机器人类型 **`xlerobot_static`**。注册之后，就可以用标准的 `lerobot-*` 命令在 wallerobot 上录制数据、回放和运行策略。

## 安装

```bash
# 在 LeRobot 环境里执行（先在 LeRobot 源码目录里 pip install -e ".[feetech]"）
pip install -e plugins/lerobot_robot_xlerobot_static
```

## 电机和观测

18 个电机，名字和 URDF 的关节名一致：
- `left_arm_*`：总线 1，ID 1–6；
- `neck_j1` … `neck_j6`：总线 1，ID 7–12；
- `right_arm_*`：总线 2，ID 1–6。

相机有三个：`head`（D435i 彩色；设 `head_use_depth=true` 时还会多一路 `head_depth`）、`left_wrist`、`right_wrist`。

## 选项

| 选项 | 默认值 | 含义 |
|---|---|---|
| `port_left` / `port_right` | `COM3` / `COM4` | 总线 1（左臂 + 脖子）和总线 2（右臂）的串口 |
| `read_only` | `false` | 不向电机写任何东西，调用 `send_action` 会报错。第一次连接时用它。 |
| `torque_on_connect` | `false` | 连接完成时上扭矩（会先把目标设为当前位置）。为 `false` 时所有电机都保持放松。 |
| `max_relative_target` | `10.0` | 每一步目标位置的变化上限：一个数，或者 `{电机名: 值}` |
| `head_use_depth` | `false` | 同时读取 D435i 的深度 |
| `use_degrees` | `true` | 用角度为单位，和当前 LeRobot 的 SO101 一致 |
| `cameras` | 手腕相机编号 0 和 2，参考机的 D435i 序列号 | 改成你自己的相机编号和 RealSense 序列号 |

插件内置的安全措施：
- 从不写脖子的 EEPROM，它只由 `tools/calib_neck.py` 负责；
- 手臂的 shoulder_lift 和 elbow_flex 在编码器跳变点重新居中之前，拒绝接受位置指令；
- 断开连接时放松扭矩。

## 第一次只读测试

```bash
lerobot-teleoperate --help        # 确认 lerobot 能找到插件（看 robot.type 的可选值）
python plugins/lerobot_robot_xlerobot_static/tests/test_readonly_hw.py
```

这个只读测试会：
- 以 `read_only=True` 连接；
- 把一次观测和直接读串口的结果对比；
- 检查连接前后扭矩都是关着的。

## 数据集

录制数据按 LeRobot 的常规流程，用 `lerobot-record` 加 `--robot.type=xlerobot_static`。在真正想公开之前，保持 `--dataset.push_to_hub=false`。

:::{note}
参考机上还有一套 ROS 2 系统：网页操控台，加一个负责执行安全规则的硬件服务器。它运行在 Windows 上，ROS 2 Jazzy 跑在 WSL2 里。这套系统暂时不在本仓库中。
:::
