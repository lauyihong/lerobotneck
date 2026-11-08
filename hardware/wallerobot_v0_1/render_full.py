"""MuJoCo offscreen render of the full wallerobot static URDF (coloured per part, white background).

    .venv/Scripts/python.exe hardware/wallerobot_v0_1/render_full.py --out x.png                 # q = 0
    .venv/Scripts/python.exe hardware/wallerobot_v0_1/render_full.py --pose pose.json --out x.png  # {"q_deg": {...}}

Robot forward = body -X (azimuth 0 = looking at the robot's front), left arm = -Y.
"""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
URDF = HERE / "urdf/wallerobot_static.urdf"
VIEWS = [("front", 0, -15), ("left side", 90, -10), ("back-right 3/4", 225, -25), ("top", 0, -89)]


def color(body):
    if body.startswith("left_arm") or body == "left_wrist_camera":
        return (0.95, 0.6, 0.15, 1)
    if body.startswith("right_arm") or body == "right_wrist_camera":
        return (0.2, 0.55, 0.85, 1)
    if body.startswith("neck_cam"):
        return (0.85, 0.3, 0.6, 1)
    if body.startswith("neck"):
        return (0.3, 0.65, 0.3, 1)
    return (0.55, 0.55, 0.55, 1)


def model(urdf=URDF):
    m = mujoco.MjModel.from_xml_path(str(urdf))
    for g in range(m.ngeom):
        m.geom_rgba[g] = color(m.body(m.geom_bodyid[g]).name)
        if m.geom_group[g] != 1 and m.geom_contype[g]:   # hide collision hulls (URDF collision -> group 0 geoms)
            pass
    return m


def render(q_deg: dict, out, size=(640, 640), urdf=URDF, views=VIEWS, title=None, hide_collision=True):
    m = model(urdf)
    m.vis.headlight.ambient[:] = 0.45; m.vis.headlight.diffuse[:] = 0.6
    m.vis.global_.offheight, m.vis.global_.offwidth = size
    d = mujoco.MjData(m)
    for n, v in q_deg.items():
        try:
            d.qpos[m.jnt_qposadr[m.joint(n).id]] = np.radians(v)
        except KeyError:
            pass
    mujoco.mj_forward(m, d)
    opt = mujoco.MjvOption()
    if hide_collision:   # URDF import: visuals and collisions are both geoms; collision geoms are the *.hull/convex meshes
        for g in range(m.ngeom):
            if m.geom_dataid[g] >= 0 and any(k in m.mesh(m.geom_dataid[g]).name for k in ("hull", "convex")):
                m.geom_rgba[g, 3] = 0
    r = mujoco.Renderer(m, *size)
    fig, axs = plt.subplots(1, len(views), figsize=(4.2 * len(views), 4.6))
    cam = mujoco.MjvCamera()
    cam.lookat[:] = d.xpos[1:].mean(0); cam.distance = 1.0
    for ax, (name, az, el) in zip(np.atleast_1d(axs), views):
        cam.azimuth, cam.elevation = az, el   # MuJoCo azimuth 0: camera on -X looking along +X = in front of the robot
        r.update_scene(d, cam, opt)
        img = r.render().copy(); img[(img == 0).all(-1)] = 255   # empty background -> white
        ax.imshow(img); ax.set_title(name); ax.axis("off")
    if title:
        fig.suptitle(title, fontsize=13)
    plt.tight_layout(); plt.savefig(out, dpi=80); plt.close(fig)
    print("saved", out)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", type=Path); ap.add_argument("--out", type=Path, required=True); ap.add_argument("--title")
    a = ap.parse_args()
    q = json.loads(a.pose.read_text())["q_deg"] if a.pose else {}
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei"]
    render(q, a.out, title=a.title)
