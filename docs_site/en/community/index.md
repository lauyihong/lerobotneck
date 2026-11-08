# Community

## Contributing

Issues and pull requests are welcome, in English or 中文. See
{repo}`CONTRIBUTING.md <CONTRIBUTING.md>` for the details. These help most:

- **Measurements** of what the model still assumes:
  - the arm mount positions on the body;
  - the J4 motor insertion depth in the neck link;
  - the wrist camera field of view;
  - the parallel gripper geometry.
- **Variants:** override files for your build, in `hardware/variants/`.
- **Docs:** fixes, photos of your build, translations (`docs_site/en` ⇄ `docs_site/zh`).
- **Simulator:** routes and UI improvements.
- **Portability:** the neck calibration tool on Linux and macOS.

When you change the model, commit the config change **together with** the regenerated files
(`python hardware/build_all.py`). CI checks that they match.

## License

{repo}`Apache-2.0 <LICENSE>`.

## Acknowledgements

wallerobot builds directly on these projects, all Apache-2.0 (see {repo}`NOTICE <NOTICE>`):
- [XLeRobot](https://github.com/Vector-Wangel/XLeRobot): body, tower, head holder, robot description;
- [SO-ARM100/101](https://github.com/TheRobotStudio/SO-ARM100) by TheRobotStudio: arms and neck parts;
- [LeRobot](https://github.com/huggingface/lerobot) by Hugging Face: robot API, datasets, policies;
- [MuJoCo](https://github.com/google-deepmind/mujoco) and [three.js](https://threejs.org/): simulation and the
  browser renderer.
