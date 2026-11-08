"""MuJoCo offscreen render of the neck URDF (coloured links, camera optical axis as a red arrow, white background).

    .venv/Scripts/python.exe hardware/neck_v0_1/render_mj.py --q 0,0,0,0,0,0 --views side:180:0,top:180:-89 --out x.png
    views: name:azimuth:elevation[:distance[:lookat_link]]  (root = wallerobot_body, robot forward = -X:
    azimuth 90 = side view from the robot's left, 0 = facing the robot / the camera)"""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
import xml.etree.ElementTree as ET

from view_urdf import URDF, fk

COLORS = {  # body -> rgba
    "world": (0.25, 0.45, 0.75, 1), "neck_shoulder": (0.95, 0.55, 0.15, 1), "neck_upper_arm": (0.2, 0.6, 0.25, 1),
    "neck_link34": (0.85, 0.15, 0.15, 1), "neck_wrist": (0.55, 0.35, 0.75, 1),
    "neck_cam_tilt_housing": (0.55, 0.4, 0.3, 1), "neck_cam_holder": (0.9, 0.5, 0.75, 1),
}


def render(urdf, q_deg, views, size=(600, 600), lookat=None):
    """lookat: fixed 3-vector for all views (e.g. to overlay two poses); default = mean body position of this pose."""
    m = mujoco.MjModel.from_xml_path(str(urdf))
    for g in range(m.ngeom):
        m.geom_rgba[g] = COLORS.get(m.body(m.geom_bodyid[g]).name, (0.6, 0.6, 0.6, 1))
        if m.geom_dataid[g] >= 0 and m.mesh(m.geom_dataid[g]).name == "tophead6":  # the D435i body itself
            m.geom_rgba[g] = (0.88, 0.88, 0.86, 1)
    m.vis.headlight.ambient[:] = 0.45; m.vis.headlight.diffuse[:] = 0.6
    m.vis.global_.offheight, m.vis.global_.offwidth = size
    d = mujoco.MjData(m)
    q = np.radians(q_deg)
    for i in range(6):
        d.qpos[m.jnt_qposadr[m.joint(f"neck_j{i + 1}").id]] = q[i]
    mujoco.mj_forward(m, d)
    cam_T = fk(ET.parse(urdf).getroot(), {f"neck_j{i + 1}": q[i] for i in range(6)})["neck_camera_color_optical_frame"]

    r = mujoco.Renderer(m, *size)
    out = []
    for v in views:
        name, az, el, *rest = v.split(":")
        cam = mujoco.MjvCamera()
        cam.azimuth, cam.elevation = float(az), float(el)
        cam.distance = float(rest[0]) if rest else 0.75
        cam.lookat[:] = d.xpos[m.body(rest[1]).id] if len(rest) > 1 else d.xpos[1:].mean(0) if lookat is None else lookat
        r.update_scene(d, cam)
        s = r.scene
        mujoco.mjv_initGeom(s.geoms[s.ngeom], mujoco.mjtGeom.mjGEOM_ARROW, np.array([0.004, 0.004, 0.09]),
                            cam_T[:3, 3], cam_T[:3, :3].flatten(), np.array([1, 0, 0, 1], np.float32))
        s.ngeom += 1
        rgb = r.render().copy()
        r.enable_depth_rendering(); depth = r.render(); r.disable_depth_rendering()
        rgb[depth >= depth.max() - 1e-6] = 255
        out.append((name, rgb))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", type=Path, default=URDF)
    ap.add_argument("--q", default="0,0,0,0,0,0", help="neck_j1..j6 in degrees")
    ap.add_argument("--views", default="side:90:0:0.9,front:0:0:0.9,top:90:-89:0.9,iso:135:-25:1.0")
    ap.add_argument("--title", default="")
    ap.add_argument("--out", default="neck_render.png")
    a = ap.parse_args()
    imgs = render(a.urdf, [float(v) for v in a.q.split(",")], a.views.split(","))
    n = len(imgs); cols = min(n, 2); rows = (n + 1) // 2
    fig, axs = plt.subplots(rows, cols, figsize=(5.5 * cols, 5.5 * rows), squeeze=False)
    for ax in axs.flat:
        ax.axis("off")
    for ax, (name, img) in zip(axs.flat, imgs):
        ax.imshow(img); ax.set_title(name, fontsize=12)
    fig.suptitle(a.title or f"q = {a.q} deg", fontsize=13)
    plt.tight_layout(); plt.savefig(a.out, dpi=90); print("saved", a.out)


if __name__ == "__main__":
    main()
