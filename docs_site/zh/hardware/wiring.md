# 接线和舵机 ID

两条飞特串行总线，1 Mbps，每条一个 USB 转接器。

| 总线 | 舵机 | ID |
|---|---|---|
| 总线 1（`port_left`） | 左臂 + 脖子 | 1–6 = 左臂（shoulder_pan … gripper），**7–12 = 脖子 J1–J6** |
| 总线 2（`port_right`） | 右臂 | 1–6 = 右臂 |

插件默认用 `COM3` / `COM4`，这是参考机（Windows）上的端口。Linux 上一般是 `/dev/ttyACM0` / `/dev/ttyACM1`，用 `--robot.port_left=… --robot.port_right=…` 指定。

## 设置 ID

新的 STS3215 出厂 ID 都是 1。**每次只接一个舵机**，给它设 ID：

```bash
python tools/scan_motors.py COM3            # 列出总线上的 ID（Linux：/dev/ttyACM0）
python tools/set_motor_id.py COM3 1 7       # 端口、旧 ID、新 ID
```

遇到以下情况，`set_motor_id.py` 会拒绝执行：旧 ID 不存在或有重复，或者新 ID 已经被占用。边设边给舵机贴标签，比如 `L1–L6`、`N7–N12`、`R1–R6`。

## 容易踩的坑

- **一个串口同一时间只能被一个程序打开。** 报 "Access is denied" 或 "port busy"，说明还有别的脚本或查看器占着它。
- **一条总线挂 12 个舵机时，广播 ping 会丢回复。** 所以这些工具都是逐个 ID 去 ping。
- **手腕相机的编号**插拔后可能会变。`projects/sim_mirror/scripts/probe_cams.py` 会从每个 OpenCV 编号各抓一帧，方便你认出哪个是哪个。
- **D435i 在 Windows 上也会显示成一个 OpenCV 相机。** RealSense SDK 正在取流时，不要再用 OpenCV 打开它。
- **不同批次的舵机，EEPROM 默认值可能不一样**（相位、最高温度、最高电压）。如果某个舵机每个回包都报电压错误，把它的 `Max_Voltage_Limit` 和其他舵机比一比。
