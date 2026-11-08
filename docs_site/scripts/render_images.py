"""Render the model images used by the docs (docs_site/_static/img/) from the generated MuJoCo model.

    python docs_site/scripts/render_images.py              # reference robot + the example variants

Needs requirements-model.txt only (offscreen MuJoCo). Re-run after the model changes (hardware/build_all.py).
Variants are built into build/docs_variants/ first. Poses are URDF degrees; READY is the browser simulator's start pose.
"""
import json
import math
import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "docs_site/_static/img"
SCENE = ROOT / "projects/sim_mirror/models/scene.xml"
HOME = {j["name"]: math.degrees(j["home"]) for j in
        json.loads((ROOT / "projects/web_sim/web/public/model/poses.json").read_text())["joints"]}
READY = {"neck_j2": 90, "neck_j3": 150, "neck_j4": 0, "neck_j5": 0, "neck_j6": -25}
for a in ("left", "right"):
    READY.update({f"{a}_arm_shoulder_pan": 0, f"{a}_arm_shoulder_lift": 100, f"{a}_arm_elbow_flex": 90,
                  f"{a}_arm_wrist_flex": 0, f"{a}_arm_wrist_roll": 90})
ZERO = {}


def render(scene, q_deg, out, az=30, el=-14, dist=0.95, look=(-0.12, 0.0, 0.22), size=(1200, 800)):
    """az 0 = looking at the robot's front (robot forward = body -x)."""
    m = mujoco.MjModel.from_xml_path(str(scene)); d = mujoco.MjData(m)
    m.vis.global_.offwidth, m.vis.global_.offheight = size
    for n, v in {**HOME, **q_deg}.items():
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
        if jid >= 0:
            lo, hi = m.jnt_range[jid]
            d.qpos[m.jnt_qposadr[jid]] = np.clip(math.radians(v), lo, hi)
    mujoco.mj_forward(m, d)
    cam = mujoco.MjvCamera(); cam.azimuth, cam.elevation, cam.distance = az, el, dist; cam.lookat[:] = look
    opt = mujoco.MjvOption(); opt.geomgroup[:] = [1, 1, 1, 0, 0, 0]          # visuals only, no collision hulls
    with mujoco.Renderer(m, size[1], size[0]) as r:
        r.update_scene(d, cam, opt)
        img = r.render()
    from PIL import Image
    out.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(img).save(out, optimize=True)
    print("wrote", out.relative_to(ROOT))


def main():
    render(SCENE, READY, OUT / "robot_ready.png")
    render(SCENE, {}, OUT / "robot_home.png")
    render(SCENE, ZERO | {n: 0 for n in HOME}, OUT / "robot_zero.png", az=60, el=-14, dist=1.25, look=(-0.30, 0, 0.22))
    for v in ("single_arm_left", "arms_out_2cm"):
        dest = ROOT / "build/docs_variants" / v
        subprocess.run([sys.executable, str(ROOT / "hardware/build_all.py"), "--config",
                        str(ROOT / f"hardware/variants/{v}.yaml"), "--out", str(dest)], check=True,
                       stdout=subprocess.DEVNULL)
        render(dest / "mjcf/scene.xml", READY, OUT / f"variant_{v}.png", az=0, el=-40, dist=0.85, look=(-0.15, 0, 0.15))
    render(SCENE, READY, OUT / "variant_reference_top.png", az=0, el=-40, dist=0.85, look=(-0.15, 0, 0.15))


if __name__ == "__main__":
    main()
