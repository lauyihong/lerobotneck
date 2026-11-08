"""Usage (repo root): .venv/Scripts/python.exe hardware/neck_v0_1/check_fold_contact.py
J3 angle where the wrist (+ far part of link34) first touches the upper arm. Signed distance via nearest sampled
surface point + its face normal (negative = inside the upper arm)."""
import sys, numpy as np, trimesh, xml.etree.ElementTree as ET
from pathlib import Path
from scipy.spatial import cKDTree
HERE = Path(__file__).resolve().parent; sys.path.insert(0, str(HERE))
from view_urdf import fk, T as Torig
urdf = HERE / "urdf/wallerobot_neck.urdf"; root = ET.parse(urdf).getroot()
def meshes(name):
    out = []
    for v in root.find(f"link[@name='{name}']").findall("visual"):
        g = v.find("geometry/mesh"); m = trimesh.load(urdf.parent / g.get("filename"), force="mesh")
        if g.get("scale"): m.apply_scale(float(g.get("scale").split()[0]))
        m.apply_transform(Torig(v.find("origin"))); out.append(m)
    return out
B = trimesh.util.concatenate(meshes("neck_upper_arm"))
pb, fi = trimesh.sample.sample_surface(B, 200000, seed=2); nb = B.face_normals[fi]; tree = cKDTree(pb)
W = trimesh.util.concatenate(meshes("neck_wrist")); pw = trimesh.sample.sample_surface(W, 60000, seed=3)[0]
L = trimesh.util.concatenate(meshes("neck_link34")); pl = trimesh.sample.sample_surface(L, 60000, seed=4)[0]
pl = pl[np.linalg.norm(pl[:, 1:], axis=1) > 0.030]   # link34 frame: x = J3 axis -> keep > 30 mm from it
home = [-5.4, 187.6, 0, 4.9, -6.2, 7.6]  # home pose (J3 swept); used 2026-11-05 to set the J3 zero (contact +185.1 deg)
def sd(j3):
    q = list(home); q[2] = j3
    P = fk(root, {f"neck_j{i+1}": np.radians(q[i]) for i in range(6)}); iu = np.linalg.inv(P["neck_upper_arm"])
    out = {}
    for name, pts, link in (("wrist", pw, "neck_wrist"), ("link34>30mm", pl, "neck_link34")):
        p = trimesh.transform_points(pts, iu @ P[link]); d, k = tree.query(p)
        s = np.einsum("ij,ij->i", p - pb[k], nb[k]); s = np.where(d < 0.004, s, d)
        out[name] = s.min() * 1000
    return out
for j3 in np.arange(176.0, 192.01, 0.5):
    print(f"J3 {j3:6.1f}: " + "  ".join(f"{k} {v:+6.2f} mm" for k, v in sd(j3).items()))
