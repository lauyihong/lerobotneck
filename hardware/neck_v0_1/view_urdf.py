"""Render the neck URDF at given joint angles (deg) and print the camera pose (no ROS / GL needed).

    .venv/Scripts/python.exe hardware/neck_v0_1/view_urdf.py --q 0,0,0,0,0,0 --out neck_q0.png [--views side:0:0] [--links a,b]"""
import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import trimesh
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from trimesh.transformations import euler_matrix, rotation_matrix, translation_matrix

URDF = Path(__file__).resolve().parent / "urdf/wallerobot_neck.urdf"


def T(o):
    xyz = [float(v) for v in o.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in o.get("rpy", "0 0 0").split()]
    return translation_matrix(xyz) @ euler_matrix(*rpy, axes="sxyz")


def fk(root, q):
    """World pose of every link, root link at identity."""
    joints = {j.find("child").get("link"): j for j in root.findall("joint")}
    poses = {}

    def pose(link):
        if link not in poses:
            j = joints.get(link)
            if j is None:
                poses[link] = np.eye(4)
            else:
                M = pose(j.find("parent").get("link")) @ T(j.find("origin"))
                if j.get("type") == "revolute":
                    M = M @ rotation_matrix(q.get(j.get("name"), 0.0), [float(v) for v in j.find("axis").get("xyz").split()])
                poses[link] = M
        return poses[link]

    for l in root.findall("link"):
        pose(l.get("name"))
    return poses


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--q", default="0,0,0,0,0,0", help="neck_j1..j6 in degrees")
    ap.add_argument("--out", default="neck_view.png")
    ap.add_argument("--views", default="iso:25:-60,side:0:0,front:0:-90,top:90:-90", help="name:elev:azim,...")
    ap.add_argument("--links", default="", help="comma list of links to draw (default all)")
    ap.add_argument("--urdf", type=Path, default=URDF)
    a = ap.parse_args()
    urdf = a.urdf
    only = set(a.links.split(",")) if a.links else None
    q = {f"neck_j{i + 1}": np.radians(float(v)) for i, v in enumerate(a.q.split(","))}
    root = ET.parse(urdf).getroot()
    P = fk(root, q)
    tris, cols = [], []
    palette = plt.cm.tab10.colors
    for k, link in enumerate(root.findall("link")):
        if only and link.get("name") not in only:
            continue
        for vis in link.findall("visual"):
            g = vis.find("geometry/mesh")
            m = trimesh.load(urdf.parent / g.get("filename"), force="mesh")
            if g.get("scale"):
                m.apply_scale(float(g.get("scale").split()[0]))
            m.apply_transform(P[link.get("name")] @ T(vis.find("origin")))
            if len(m.faces) > 6000:
                m = m.submesh([np.random.default_rng(0).choice(len(m.faces), 6000, replace=False)], append=True)
            shade = 0.45 + 0.55 * np.abs(m.face_normals @ np.array([0.3, -0.5, 0.8]) / np.linalg.norm([0.3, -0.5, 0.8]))
            tris.append(m.triangles)
            cols.append(np.c_[np.outer(shade, palette[k % 10]), np.ones(len(shade))])
    tris, cols = np.concatenate(tris), np.concatenate(cols)

    cam = P["neck_camera_color_optical_frame"]
    np.set_printoptions(precision=3, suppress=True)
    print("q (deg):", a.q)
    print("camera optical centre in neck_base [mm]:", cam[:3, 3] * 1000)
    print("camera optical axis (z) in neck_base:", cam[:3, 2])
    for n in ["neck_link34", "neck_wrist", "neck_cam_tilt_housing", "neck_cam_holder"]:
        print(f"  {n:24s} origin [mm] {P[n][:3, 3] * 1000}")

    views = [v.split(":") for v in a.views.split(",")]
    fig = plt.figure(figsize=(6 * min(len(views), 2), 5.5 * ((len(views) + 1) // 2)))
    pts = tris.reshape(-1, 3)
    c, r = (pts.max(0) + pts.min(0)) / 2, (pts.max(0) - pts.min(0)).max() / 2
    for i, (name, el, az) in enumerate(views):
        ax = fig.add_subplot((len(views) + 1) // 2, min(len(views), 2), i + 1, projection="3d")
        ax.add_collection3d(Poly3DCollection(tris, facecolors=cols, linewidths=0))
        o, zax = cam[:3, 3], cam[:3, 2] * 0.08
        ax.quiver(*o, *zax, color="red", linewidth=2)  # camera optical axis
        ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
        ax.set_box_aspect((1, 1, 1)); ax.view_init(float(el), float(az)); ax.set_title(f"{name}  q={a.q}")
        ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
    plt.tight_layout(); plt.savefig(a.out, dpi=80)
    print("saved", a.out)


if __name__ == "__main__":
    main()
