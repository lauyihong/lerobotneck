# Contributing

Thanks for your interest! Issues and pull requests are welcome in English or 中文.

## Good first contributions
- **Measurements** of the real hardware that the model still assumes: arm mount positions, the J4 motor insertion
  depth in the neck link, the wrist camera field of view. See "Known limitations" in the docs.
- **Variants:** an override file in `hardware/variants/` for your own build, for example other mounts or a single arm.
- **Docs:** corrections, photos of your build, translations (`docs_site/en` ⇄ `docs_site/zh`).
- **Simulator:** new routes for `projects/web_sim/web/public/routes/`, UI fixes.

## Changing the robot model
1. Edit `hardware/wallerobot_v0_1/robot.yaml`. For a variant, add an override file instead.
2. Run `python hardware/build_all.py`. This regenerates the URDF, MJCF, joint table and web model.
3. Commit the config change **and** the regenerated files together. CI runs `build_all.py --check` and fails when a
   generated file is out of date.

Generated files carry a `GENERATED` header. Edit the config or the builder, not the generated file.

## Pull requests
- Keep them focused: one change per PR.
- CI must pass: the model rebuild check, the web build and the docs build.
- For hardware changes, say what you measured and how.
- By contributing, you agree that your contribution is licensed under Apache-2.0.

## Safety
The plugin and tools can move real servos. Code that writes motor registers must keep the existing guards:
- read-only mode;
- no commands further into the mechanical stops;
- write the speed cap before the goal position;
- release torque on exit.

Describe how you tested it.
