"""Check that the MuJoCo twin (models/wallerobot_static.xml) is the same robot as the URDF it was built from.

    .venv/Scripts/python.exe projects/sim_mirror/scripts/check_mjcf.py [--poses 30] [--tol-mm 0.05]

Independent of MuJoCo's URDF importer: the URDF side is a small forward-kinematics implementation written here
(origin xyz/rpy, revolute axis), the meshes are read with trimesh. For q = 0, the real rest pose and N random poses inside
the joint limits it compares:
  1. joints     names, type, limits (URDF limit == MJCF range == actuator ctrlrange), world axis direction
  2. frames     every URDF link (also links fused away by MuJoCo through fixed joints) vs its MJCF body (+ fixed offset)
  3. geometry   every URDF visual/collision mesh: world-space AABB of its vertices vs the matching MJCF geom
                (catches wrong origin / rpy / scale / mesh file; MuJoCo re-centres meshes, so poses are compared via vertices)
  4. extras     neck_color camera = URDF optical frame (z forward, y down), jaw tip sites = URDF *_tip links,
                visual geoms do not collide, collision geoms are hidden in group 3, total mass
Exit code 0 = all checks pass. Prints a summary line per check.
"""
import argparse
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import trimesh
from trimesh.transformations import euler_matrix, rotation_matrix, translation_matrix

HERE = Path(__file__).resolve().parent
PROJ = HERE.parent
ROOT = PROJ.parents[1]
URDF = ROOT / "hardware/wallerobot_v0_1/urdf/wallerobot_static.urdf"
MODEL = PROJ / "models/wallerobot_static.xml"
REST = PROJ / "outputs/rest_pose_20261105.json"
sys.path.insert(0, str(HERE))


def T_of(o):
    if o is None:
        return np.eye(4)
    return translation_matrix([float(v) for v in o.get("xyz", "0 0 0").split()]) @ \
        euler_matrix(*[float(v) for v in o.get("rpy", "0 0 0").split()], axes="sxyz")


class UrdfFK:
    def __init__(self, path):
        self.root = ET.parse(path).getroot()
        self.dir = Path(path).parent
        self.joints = {j.get("name"): j for j in self.root.findall("joint")}
        self.parent = {j.find("child").get("link"): j for j in self.joints.values()}
        links = [l.get("name") for l in self.root.findall("link")]
        self.base = next(l for l in links if l not in self.parent)
        self.links = {l.get("name"): l for l in self.root.findall("link")}
        self.movable = {n: j for n, j in self.joints.items() if j.get("type") != "fixed"}

    def link_poses(self, q):
        out = {self.base: np.eye(4)}

        def pose(link):
            if link not in out:
                j = self.parent[link]
                M = pose(j.find("parent").get("link")) @ T_of(j.find("origin"))
                if j.get("type") in ("revolute", "continuous"):
                    ax = np.array([float(v) for v in (j.find("axis").get("xyz") if j.find("axis") is not None
                                                      else "1 0 0").split()])
                    M = M @ rotation_matrix(q.get(j.get("name"), 0.0), ax / np.linalg.norm(ax))
                elif j.get("type") != "fixed":
                    raise ValueError(f"unsupported joint type {j.get('type')}")
                out[link] = M
            return out[link]
        for l in self.links:
            pose(l)
        return out

    def geometries(self):
        """[(link, kind, mesh_path, T_link_geom, scale)]"""
        g = []
        for name, l in self.links.items():
            for kind in ("visual", "collision"):
                for e in l.findall(kind):
                    m = e.find("geometry/mesh")
                    if m is None:
                        raise ValueError(f"{name}: non-mesh {kind} geometry not handled")
                    sc = np.array([float(v) for v in m.get("scale", "1 1 1").split()])
                    g.append((name, kind, self.dir / m.get("filename"), T_of(e.find("origin")), sc))
        return g


def aabb(P):
    return np.concatenate([P.min(0), P.max(0)])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--poses", type=int, default=30); ap.add_argument("--tol-mm", type=float, default=0.05)
    ap.add_argument("--model", type=Path, default=MODEL); ap.add_argument("--urdf", type=Path, default=URDF)
    a = ap.parse_args()
    tol = a.tol_mm / 1000
    U = UrdfFK(a.urdf)
    m = mujoco.MjModel.from_xml_path(str(a.model)); d = mujoco.MjData(m)
    fails = []

    def report(name, ok, msg):
        print(f"[{'PASS' if ok else 'FAIL'}] {name}: {msg}")
        if not ok:
            fails.append(name)

    # ---- 1. joints ----------------------------------------------------------------------------------------------
    mj_joints = [m.joint(i).name for i in range(m.njnt)]
    names_ok = sorted(mj_joints) == sorted(U.movable)
    bad = []
    for n, j in U.movable.items():
        if n not in mj_joints:
            continue
        lim = j.find("limit"); lo, hi = float(lim.get("lower")), float(lim.get("upper"))
        jid = m.joint(n).id
        if m.jnt_type[jid] != mujoco.mjtJoint.mjJNT_HINGE or not m.jnt_limited[jid] \
                or not np.allclose(m.jnt_range[jid], (lo, hi), atol=1e-6):
            bad.append(f"{n} range {m.jnt_range[jid]} vs urdf ({lo}, {hi})")
        aid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
        if aid < 0 or m.actuator_trnid[aid, 0] != jid or not np.allclose(m.actuator_ctrlrange[aid], (lo, hi), atol=1e-4):
            bad.append(f"{n} actuator missing / wrong joint / ctrlrange")
    report("joints", names_ok and not bad,
           f"{len(mj_joints)} MJCF vs {len(U.movable)} URDF movable joints"
           + ("" if names_ok else f", name mismatch {set(mj_joints) ^ set(U.movable)}") + (f"; {bad}" if bad else ""))

    # poses: zero, rest, random
    lims = {n: (float(j.find("limit").get("lower")), float(j.find("limit").get("upper"))) for n, j in U.movable.items()}
    poses = [("zero", {n: 0.0 for n in U.movable})]
    if REST.exists():
        import calib
        poses.append(("rest", calib.ticks_to_q(json.loads(REST.read_text())["ticks"])))
    rng = np.random.default_rng(0)
    poses += [(f"rand{i}", {n: rng.uniform(*lims[n]) for n in U.movable}) for i in range(a.poses)]

    # URDF link -> (MJCF body, fixed offset) : walk up fixed joints until a link that exists as an MJCF body
    body_names = {m.body(i).name for i in range(m.nbody)}
    fused = {}
    for l in U.links:
        M, cur = np.eye(4), l
        while cur not in body_names and cur != U.base:
            j = U.parent[cur]
            assert j.get("type") == "fixed", f"{cur} missing in MJCF but its joint is {j.get('type')}"
            M = T_of(j.find("origin")) @ M; cur = j.find("parent").get("link")
        fused[l] = (cur if cur in body_names else "world", M)

    # geometry: URDF meshes -> local vertices; MJCF geoms -> world vertices per pose
    geoms = U.geometries()
    vcache = {}
    for _, _, p, _, sc in geoms:
        if (p, tuple(sc)) not in vcache:
            vcache[(p, tuple(sc))] = np.asarray(trimesh.load(p, force="mesh").vertices) * sc
    mj_geoms = [g for g in range(m.ngeom) if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH]
    if len(mj_geoms) != len(geoms):
        report("geometry count", False, f"{len(mj_geoms)} MJCF mesh geoms vs {len(geoms)} URDF meshes")

    err_axis = err_frame = err_geom = 0.0; worst_geom = worst_frame = ""; unmatched = set()
    for pname, q in poses:
        for n in U.movable:
            d.qpos[m.jnt_qposadr[m.joint(n).id]] = q.get(n, 0.0)
        mujoco.mj_forward(m, d)
        LP = U.link_poses(q)
        # base offset: MJCF world == URDF base frame (body fixed to world)
        for n, j in U.movable.items():
            ax = np.array([float(v) for v in j.find("axis").get("xyz").split()]); ax /= np.linalg.norm(ax)
            ua = LP[j.find("child").get("link")][:3, :3] @ ax
            err_axis = max(err_axis, np.linalg.norm(ua - d.xaxis[m.joint(n).id]))
        for l, (b, M) in fused.items():
            if b == "world":
                Bw = np.eye(4)
            else:
                Bw = np.eye(4); Bw[:3, :3] = d.xmat[m.body(b).id].reshape(3, 3); Bw[:3, 3] = d.xpos[m.body(b).id]
            W = Bw @ M
            e = np.linalg.norm(W[:3, 3] - LP[l][:3, 3]) + 0.1 * np.linalg.norm(W[:3, :3] - LP[l][:3, :3])
            if e > err_frame:
                err_frame, worst_frame = e, f"{l} @ {pname}"
        # geometry match (greedy by mesh-name stem + AABB distance)
        mj_box = {}
        for g in mj_geoms:
            mid = m.geom_dataid[g]
            V = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
            mj_box[g] = aabb(V @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g])
        free = set(mj_geoms)
        for link, kind, p, T, sc in geoms:
            W = LP[link] @ T
            box = aabb(vcache[(p, tuple(sc))] @ W[:3, :3].T + W[:3, 3])
            stem = p.name.removesuffix(".stl")
            cands = [g for g in free if m.mesh(m.geom_dataid[g]).name == stem]   # MuJoCo mesh name = file stem
            if not cands:
                unmatched.add(f"{link}:{p.name}"); continue
            g = min(cands, key=lambda g: np.abs(mj_box[g] - box).max())
            e = np.abs(mj_box[g] - box).max(); free.discard(g)
            want_visual = kind == "visual"
            is_visual = m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0
            if is_visual != want_visual or m.geom_group[g] != (2 if want_visual else 3):
                unmatched.add(f"{link}:{p.name} class/group wrong (group {m.geom_group[g]}, contype {m.geom_contype[g]})")
            if e > err_geom:
                err_geom, worst_geom = e, f"{link}:{p.name} @ {pname}"
        unmatched |= {f"extra MJCF geom {m.geom(g).name or g}" for g in free}

    report("joint axes", err_axis < 1e-5, f"max world-axis difference {err_axis:.1e} over {len(poses)} poses")
    report("link frames", err_frame < tol, f"max error {err_frame * 1000:.4f} mm (+0.1 m x rot) at {worst_frame}; "
                                           f"{len(U.links)} links, {len(poses)} poses")
    report("mesh geometry", err_geom < tol and not unmatched,
           f"{len(geoms)} meshes, max world AABB error {err_geom * 1000:.4f} mm at {worst_geom}"
           + (f"; problems: {sorted(unmatched)[:6]}" if unmatched else ""))

    # ---- 4. extras ----------------------------------------------------------------------------------------------
    mujoco.mj_resetData(m, d); mujoco.mj_forward(m, d); LP = U.link_poses({})
    extra = []
    cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "neck_color")
    if cid < 0:
        extra.append("camera neck_color missing")
    else:
        O = LP["neck_camera_color_optical_frame"]; Rc = d.cam_xmat[cid].reshape(3, 3)
        # MuJoCo camera looks along -z with image up +y; optical frame looks along +z with image down +y
        if np.linalg.norm(d.cam_xpos[cid] - O[:3, 3]) > tol or np.linalg.norm(Rc @ np.diag([1, -1, -1]) - O[:3, :3]) > 1e-5:
            extra.append("neck_color camera != URDF optical frame")
    for cam in ("left_wrist", "right_wrist"):
        if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, cam) < 0:
            extra.append(f"camera {cam} missing")
    for side in ("left", "right"):
        for jaw in ("fixed_jaw", "moving_jaw"):
            sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f"{side}_{jaw}_tip")
            if sid < 0 or np.linalg.norm(d.site_xpos[sid] - LP[f"{side}_arm_{jaw}_tip"][:3, 3]) > tol:
                extra.append(f"site {side}_{jaw}_tip")
    # links fused into the world body (fixed to the static base) carry no simulated mass in MuJoCo -> leave them out
    um = sum(float(l.find("inertial/mass").get("value")) for n, l in U.links.items()
             if l.find("inertial/mass") is not None and fused[n][0] != "world")
    mm = float(m.body_mass.sum())
    if abs(um - mm) > 1e-6:
        extra.append(f"mass URDF {um:.4f} vs MJCF {mm:.4f} kg")
    report("cameras / sites / mass", not extra, f"total mass {mm:.3f} kg, {m.ncam} cameras, {m.nsite} sites"
           + (f"; {extra}" if extra else ""))

    print("ALL PASS" if not fails else f"FAILED: {fails}")
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
