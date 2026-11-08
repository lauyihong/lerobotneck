# web_sim — wallerobot in the browser

A browser simulator of wallerobot: MuJoCo (WASM) physics with three.js rendering, built with vite. It is
**simulation only** and never talks to hardware.

- **Online:** https://lauyihong.github.io/wallerobot/sim/
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
