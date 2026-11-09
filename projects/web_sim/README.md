# web_sim — wallerobot in the browser

The project's website, built with vite: MuJoCo (WASM) with three.js rendering. It is **simulation only** and never
talks to hardware. One build, two apps:

| URL | what | source |
|---|---|---|
| https://lauyihong.github.io/lerobotneck/ | **LeRobotNeck** showcase: the neck demo (home), `play/` neck simulator, `neck/` exploded view | `web/index.html`, `play/`, `neck/`, `src/neck/` |
| https://lauyihong.github.io/lerobotneck/sim/ | full wallerobot simulator (two arms + neck) | `web/sim/index.html`, `src/main.js` |

The docs are served next to them at `/docs/` (see `.github/workflows/pages.yml`).

## LeRobotNeck showcase
- One page app on one shared canvas: home (looping demo, click the neck to explode it), `play/` (demo cards; pause,
  move the joints, resume: the route is re-planned from your pose), `neck/` (7 steps: stock SO101 -> apart -> the two
  parts that change -> the new J3-J4 link + XLeRobot's soft-gripper jaw -> J4 turned 90° -> camera -> assembled; `neck/#step-N` opens a step).
- Model: `web/public/model_neck/`, exported by `scripts/export_assets.py --neck` from the
  `hardware/variants/lerobotneck.yaml` variant (`build_all.py` does it), plus a hidden copy of a stock SO101 arm
  (`stock_*` bodies) for the exploded view. Kinematic only (no contacts).
- Demos and desk scene: `web/public/routes_neck/*.json` (`scene.json` = the objects on the desk), written by
  `scripts/make_neck_routes.py` (labels in English and Chinese). The two active-perception demos (circle the green
  block, look inside the box) are solved with inverse kinematics on `model_neck`, and the script checks with ray
  tests that the things in the box are hidden from the ready pose and seen from the solved viewpoints.
- UI text: `src/neck/i18n.js` (English first, Chinese; follows the browser language, switch in the top bar).

## Full simulator
- **Controls:** keyboard end-effector moves for both arms, the neck, a mouse drag gizmo, typed xyz targets, fixed
  routes, and a route recorder. Press `?` in the app for the key map.

## Run locally
```bash
cd projects/web_sim/web
npm ci
npm run dev        # http://localhost:5180
npm run build      # -> dist/, static files; deployable under any sub-path (base: './')
```

## Model
`web/public/model/` is **generated** from the MuJoCo model by `scripts/export_assets.py`, which
`python hardware/build_all.py` runs. The export does three things:
- copies the referenced meshes;
- adds `left_ee` / `right_ee` IK target sites;
- writes `poses.json` with the home pose and the joint ranges.

Don't edit those files by hand. Change `hardware/wallerobot_v0_1/robot.yaml` and rebuild instead.

## Routes
`web/public/routes/*.json` hold hand-written fixed routes. Angles are in degrees, URDF convention; joints that a
waypoint omits keep their previous value. List a new route in `routes/index.json`. The in-app recorder saves the
same format.

```json
{ "name": "…", "speed_dps": 30, "from": "home",
  "waypoints": [ { "label": "stand up", "q": { "neck_j2": 90, "neck_j3": 150 }, "duration_s": 2.0, "dwell_s": 1.0 } ] }
```

## Stack
| part | version |
|---|---|
| `@mujoco/mujoco` | 3.16.0, the same as the Python MuJoCo used to generate the model |
| `three` | 0.186.1 |
| `vite` | 7.3.7 |

`scripts/package_artifact.py` packs the build into a single-page bundle for hosts that cannot serve `.stl` files.
