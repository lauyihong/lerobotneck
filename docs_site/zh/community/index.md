# 社区

## 参与贡献

欢迎提 issue 和 pull request，中文或英文都可以。详细说明见 {repo}`CONTRIBUTING.md <CONTRIBUTING.md>`。最有帮助的是：

- **实测数据**，补上模型里还在用假设值的地方：
  - 手臂在机身上的安装位置；
  - J4 电机插进脖子连杆的深度；
  - 手腕相机的视场角；
  - 平行夹爪的几何。
- **变体**：你自己搭的版本的覆盖文件，放进 `hardware/variants/`。
- **文档**：修正错误、补充你的实物照片、翻译（`docs_site/en` ⇄ `docs_site/zh`）。
- **模拟器**：新的路线和界面改进。
- **跨平台**：让脖子标定工具支持 Linux 和 macOS。

修改模型时，把配置改动和重新生成的文件**放在同一个提交里**（先运行 `python hardware/build_all.py`）。CI 会检查两者是否一致。

## 许可证

{repo}`Apache-2.0 <LICENSE>`。

## 致谢

wallerobot 直接建立在以下项目之上，它们都使用 Apache-2.0 许可证（见 {repo}`NOTICE <NOTICE>`）：
- [XLeRobot](https://github.com/Vector-Wangel/XLeRobot)：机身、立柱、头部支架、机器人描述；
- TheRobotStudio 的 [SO-ARM100/101](https://github.com/TheRobotStudio/SO-ARM100)：手臂和脖子的零件；
- Hugging Face 的 [LeRobot](https://github.com/huggingface/lerobot)：机器人接口、数据集、策略；
- [MuJoCo](https://github.com/google-deepmind/mujoco) 和 [three.js](https://threejs.org/)：仿真和网页渲染。
