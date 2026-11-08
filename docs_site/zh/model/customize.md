# 定制机器人

每种改动都是同样三步：

1. 写一个**覆盖文件**，只放要改的键。只有要修改参考机本身时，才直接改 `robot.yaml`。
2. 生成到单独的文件夹：`python hardware/build_all.py --config my.yaml --out build/my`。
3. 用 `python projects/sim_mirror/scripts/view.py` 查看，或者在 MuJoCo 里打开 `build/my/mjcf/scene.xml`。

```{image} /../_static/img/variant_reference_top.png
:alt: 参考机正视
:width: 32%
```
```{image} /../_static/img/variant_arms_out_2cm.png
:alt: 手臂外移 2 cm
:width: 32%
```
```{image} /../_static/img/variant_single_arm_left.png
:alt: 只有左臂
:width: 32%
```

*从左到右：参考机、`arms_out_2cm.yaml`、`single_arm_left.yaml`。*

## 挪手臂

两条臂的设置互相独立。`mount.xyz` 是 SO101 底座圆在安装台上的圆心，`yaw_deg` 是手臂绕竖直轴的转角。

```yaml
# hardware/variants/arms_out_2cm.yaml：两条臂各向外挪 2 cm
arms:
  left:
    mount: {xyz: [-0.090, -0.1529, 0.048], yaw_deg: -90}
  right:
    mount: {xyz: [-0.090, 0.1529, 0.048], yaw_deg: -90}
```

```bash
python hardware/build_all.py --config hardware/variants/arms_out_2cm.yaml --out build/arms_out_2cm
```

## 单臂

```yaml
arms:
  right:
    enabled: false
```

生成结果有 12 个关节（左臂和脖子）、2 个相机。MJCF 生成脚本会自动跳过缺失手臂的相机和末端点，相关的接触排除也会去掉。

## 换一个脖子连杆

想试别的 J3→J4 连杆，按这几步做：
1. 把连杆导出成 STL，单位用毫米。
2. 在 STL 自己的坐标系里量两个点：
   - `j3_axis_mm`：J3 舵盘的中心，也就是连杆绕着转的那根轴；
   - `j4_shaft_mm`：J4 电机在连杆里那一端（非输出轴的一端）。
3. 把 `neck.link34.stl` 指向这个文件，再填上这两个点：

```yaml
neck:
  link34:
    stl: my_parts/neck_link_long.stl
    j3_axis_mm: [41.0, 23.2, 12.0]
    j4_shaft_mm: [-40.2, 23.9, 28.85]
```

如果 J4 电机出现在错的一侧，或者朝向反了，就翻转 `side` 或 `j4_shaft_sign`（±1）。最后用 `view.py --pose zero` 检查 q = 0 时的样子：脖子的 J3 应该是伸直的，J4 电机和上臂在一条线上。

## 相机安装和镜头

```yaml
cameras:
  head:
    offset_in_holder_m: [0.025, 0.0, 0.03]
    vfov_deg: 43.2          # 由内参计算：2*atan(height / (2*fy))
  wrist:
    vfov_deg: 72.0
```

## 用自己的标定

把 `calibration:` 指向你自己的文件。URDF 的限位、网页模型的 home 姿态和关节表都会跟着更新。

```yaml
calibration:
  neck: my_robot/neck_calibration.json
  arms: my_robot/arm_calibration.json
```

:::{tip}
做出了好用的变体，欢迎分享：在 pull request 里把覆盖文件加到 `hardware/variants/`。CI 会生成这个文件夹里的每一个变体。
:::
