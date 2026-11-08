# 标定

标定把舵机的原始 tick 换算成 URDF 里的关节角：

$$q = \text{sign} \cdot (\text{Present\_Position} - \text{zero\_tick}) \cdot \frac{2\pi}{4096}$$

`Present_Position` 已经包含舵机 EEPROM 里的 `Homing_Offset`。夹爪按 `closed_tick` 到 `open_tick` 线性换算。

每台机器人有自己的标定文件，由 `robot.yaml` 指定：

| 文件 | 内容 |
|---|---|
| `hardware/neck_v0_1/neck_calibration.json` | 脖子：每个关节的 `id`、`sign`、`zero_tick`、行程范围、URDF 限位；`home` 姿态；修改历史 |
| `hardware/wallerobot_v0_1/arm_calibration.json` | 手臂：`sign`、`zero_tick`、行程范围、夹爪的闭合 / 张开值、标准休息位 |

仓库里附带的是参考机的标定文件。用自己的机器人时：
1. 先标定，再替换这两个文件，或者在覆盖文件的 `calibration:` 里指向你自己的文件；
2. 然后运行 `python hardware/build_all.py`。

URDF 的限位会跟着标定文件更新。

## 脖子：`tools/calib_neck.py`

手扶着标定，**全程不上扭矩**，舵机不会自己动。

```bash
python tools/calib_neck.py guide        # 生成 calib_guide.png：每个关节的零位和正方向
python tools/calib_neck.py calibrate    # 交互式；总线 1 不能被别的程序占用
python tools/calib_neck.py show         # 读编码器，画出当前姿态
```

1. **零位**：把每个关节摆到它的 URDF 零位（对照 guide 图）：
   - J2 = 0：上臂水平；
   - J3 = 0：J4 电机和上臂在一条线上；
   - J4 = 0：腕部伸直；
   - J5 = 0：相机正立，在 J6 外壳下方；
   - J6 = 0：光轴沿着 J5 的转轴。
2. **方向**：把每个关节往"正"方向转大约 30°。
3. **范围**：把每个关节慢慢转过它的全部行程。
4. **写入**：
   - 往 EEPROM 写 `Homing_Offset`（让行程中点读数为 2047）和位置限位；
   - 同时写好 JSON 文件；
   - 写之前会先备份旧的 EEPROM 值，中途出错就自动恢复。

:::{note}
`calib_neck.py` 目前用 Windows 的 `msvcrt` 模块读键盘。欢迎贡献 Linux / macOS 的适配。
:::

**有些零位不好目测。** 参考机事后修正了几个零位，都只改 JSON 文件：
- **J3** 用折叠接触来定：折叠时腕部贴住上臂，模型里这个位置是 +185.1°。
- **J2 / J6** 用 D435i 的 IMU 来定：在 11 个姿态下，把测到的重力方向和模型对比，残差约 1°。脚本是 `projects/sim_mirror/scripts/imu_fit.py`。

## 手臂：从标准休息位标定

把两条臂摆成标准休息位：
- shoulder_pan 0；
- shoulder_lift 靠在折叠限位上（+187°）；
- elbow +180°；
- wrist_flex +50°；
- wrist_roll +90°（手腕相机朝上）；
- 夹爪闭合。

`hardware/wallerobot_v0_1/calib_arm_rest.py` 根据这个姿态下的一次编码器读数算出零位。然后在只读镜像里逐个检查方向：用手转动关节，看模型是不是朝同一个方向跟着动。

:::{warning}
SO101 的 **shoulder_lift 和 elbow_flex 没有 EEPROM 限位，编码器的跳变点（4095 → 0）落在行程里面。** 读数时程序会把它展开，但**位置控制会绕远路，一直顶进限位**。在给它们发指令之前，要先把它们的 `Homing_Offset` 重新居中；在那之前，插件会拒绝控制这两个关节。
:::
