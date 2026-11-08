"""Build the full wallerobot static URDF: XLeRobot body (no mobile base) + dual SO101 arms + neck v0.1 (+ D435i).

    .venv/Scripts/python.exe hardware/wallerobot_v0_1/build_full_urdf.py     # -> urdf/wallerobot_static.urdf + urdf/meshes/

- Neck, body/tower and camera come from hardware/neck_v0_1/build_urdf.py (re-generated here, unchanged).
- Arms = upstream SO101 chains from third_party/XLeRobot/simulation/xlerobot/xlerobot.urdf (same file the neck frames
  come from). Upstream `_L` chain (base at y = -0.11, robot forward = -X) = our LEFT arm (COM3); upstream calls its
  wrist camera "Right_Arm_Camera", we call it left_wrist_camera. Arm base poses in wallerobot_body (= upstream
  top_base_link, chassis + 0.35 m): (-0.09, -/+0.11, 0.045), yaw 0 / pi.
- Joint names = lerobot plugin motor names (left_arm_shoulder_pan .. right_arm_gripper, neck_j1..6).
- Joint limits: neck from neck_calibration.json; arms = upstream URDF limits intersected with the calibrated motor
  range mapped through arm_calibration.json (seeded by seed_arm_calibration.py, unverified until the sim-real mirror).
- Collision: upstream convex parts for the jaws, otherwise the convex hull of every visual mesh (`*.hull.stl`).
- Gripper: real robot has PARALLEL grippers (2026-10-17/18) but no CAD is in the workspace -> stock SO101 jaw meshes
  as a placeholder (ASSUMPTION, see README.md).
- All robot numbers (arm mounts, colours, which arms exist, upstream files, calibration) come from robot.yaml next to this
  script; `--config my.yaml --out dir` builds a variant without touching the committed model.
"""
import copy
import hashlib
import json
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh
from trimesh.transformations import euler_from_matrix, euler_matrix, rotation_matrix, translation_matrix

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
NECK = ROOT / "hardware/neck_v0_1"
sys.path.insert(0, str(NECK))
sys.path.insert(0, str(ROOT / "hardware"))
import build_urdf as neck  # noqa: E402
import robot_config  # noqa: E402

OUT = HERE / "urdf"
TICKS = 4096
HULLS_JSON = "_hull_sources.json"   # in the mesh dir: hull file -> sha1 of the mesh it was made from

# upstream link -> our link suffix, upstream joint -> our motor suffix
LINKS = {"Base": "base", "Rotation_Pitch": "shoulder", "Upper_Arm": "upper_arm", "Lower_Arm": "lower_arm",
         "Wrist_Pitch_Roll": "wrist", "Fixed_Jaw": "fixed_jaw", "Moving_Jaw": "moving_jaw"}
JOINTS = {"Rotation": "shoulder_pan", "Pitch": "shoulder_lift", "Elbow": "elbow_flex", "Wrist_Pitch": "wrist_flex",
          "Wrist_Roll": "wrist_roll", "Jaw": "gripper"}
# upstream xlerobot.urdf chains: link/joint name suffixes and the wrist camera link of each (upstream `_L` carries the
# "Right_Arm_Camera"). robot.yaml arms.<side>.upstream_chain picks the chain for each of our arms.
CHAINS = {"L": {"suffix": "", "jsuffix": "_L", "cam": "Right_Arm_Camera", "base_joint": "fixed_Base"},
          "R": {"suffix": "_2", "jsuffix": "_R", "cam": "Left_Arm_Camera", "base_joint": "fixed_Base_2"}}
TIP = {"fixed_jaw": (0.01, -0.097, 0.0), "moving_jaw": (-0.01, -0.073, 0.0)}  # ManiSkill xlerobot.urdf *_Jaw_tip
# Where the robot.yaml `arms` defaults come from. User 2026-11-05: both SO101 face FORWARD (main camera direction) -
# upstream simulation/xlerobot.urdf mounts them facing outwards (yaw 0 / pi), which is wrong for this robot (the
# ManiSkill upstream URDF faces forward). SO101 arm forward at pan = 0 is Base -Y -> yaw -90 deg puts it on body -X
# (= robot forward) for BOTH arms. Mount circle: SO101 Base.stl footprint circle (r 56.2 mm, centre (0, -22.9) mm in
# Base frame) sits on the body pad. Pad centre (x -90, y -/+132.9 mm) = upstream Base origin (-0.09, -/+0.11) +
# footprint offset (same xy as the XLeRobot 0.4.0 armbase CAD circles r 56.5 at tower-relative (-2.2, +/-136.5),
# within 4 mm); pad top z 48 mm measured on base_body_lower.stl (upstream used 45).


def T_of(o):
    if o is None:
        return np.eye(4)
    xyz = [float(v) for v in o.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in o.get("rpy", "0 0 0").split()]
    return translation_matrix(xyz) @ euler_matrix(*rpy, axes="sxyz")


def origin_el(parent, M):
    return ET.SubElement(parent, "origin", xyz=" ".join(f"{v:.6f}" for v in M[:3, 3]),
                         rpy=" ".join(f"{v:.6f}" for v in euler_from_matrix(M, axes="sxyz")))


def hull_file(mesh_dir: Path, name: str) -> str:
    """Convex hull of meshes/<name> written next to it as <stem>.hull.stl (same units/scale as the visual).
    Regenerated when the source mesh changed (sha1 kept in meshes/_hull_sources.json); a hull that exists without a
    record is adopted as is, so committed hulls stay byte-identical."""
    out = name.rsplit(".", 1)[0] + ".hull.stl"
    rec_path = mesh_dir / HULLS_JSON
    rec = json.loads(rec_path.read_text()) if rec_path.exists() else {}
    sha = hashlib.sha1((mesh_dir / name).read_bytes()).hexdigest()
    if not (mesh_dir / out).exists() or rec.get(out, sha) != sha:
        trimesh.load(mesh_dir / name, force="mesh").convex_hull.export(mesh_dir / out)
    if rec.get(out) != sha:
        rec[out] = sha
        rec_path.write_text(json.dumps(dict(sorted(rec.items())), indent=1) + "\n")
    return out


def arm_limits(cal, motor, up_lim):
    """Upstream limits intersected with the motor range mapped through the arm calibration (rad)."""
    c = cal[motor]
    if c.get("type") == "gripper":   # widen to the calibrated closed/open angles (closed = finger contact -22 deg)
        return (min(up_lim[0], c["closed_rad"]), max(up_lim[1], c["open_rad"]))
    if c["range_min"] <= 0 and c["range_max"] >= TICKS - 1:  # no software limit (lift/elbow on a mechanical stop)
        return up_lim
    a, b = (c["sign"] * (r - c["zero_tick"]) * 2 * np.pi / TICKS for r in (c["range_min"], c["range_max"]))
    lo, hi = min(a, b), max(a, b)
    return (max(lo, up_lim[0]), min(hi, up_lim[1])) if up_lim else (lo, hi)


def build(out=OUT, cfg=None):
    cfg = cfg or robot_config.load()
    out.mkdir(parents=True, exist_ok=True)
    mesh_dir = out / "meshes"
    UP_URDF = robot_config.path(cfg["upstream"]["urdf"]); UP_MESH = robot_config.path(cfg["upstream"]["meshes"])
    jd = cfg["joint_defaults"]
    footprint = np.array(cfg["arms"]["footprint_centre"], float)
    neck_path = neck.build(out, cfg)                  # neck + body + camera, meshes copied into out/meshes
    robot = ET.parse(neck_path).getroot()
    neck_path.unlink()
    robot.set("name", "wallerobot_static")
    robot.remove(robot.find("mujoco")); mj = ET.Element("mujoco"); robot.insert(0, mj)
    ET.SubElement(mj, "compiler", meshdir="meshes/", strippath="true", discardvisual="false", balanceinertia="true")

    up = ET.parse(UP_URDF).getroot()
    up_links = {l.get("name"): l for l in up.findall("link")}
    up_joints = {j.get("name"): j for j in up.findall("joint")}
    child_of = {j.find("parent").get("link"): [] for j in up.findall("joint")}
    for j in up.findall("joint"):
        child_of[j.find("parent").get("link")].append(j)
    cal = json.loads(robot_config.path(cfg["calibration"]["arms"]).read_text(encoding="utf-8"))["joints"]
    T_top = T_of(up_joints["fixed_top_base_link"].find("origin"))

    def add_visuals(link_el, up_link, M=np.eye(4), color=None):
        for v in up_link.findall("visual"):
            g = v.find("geometry/mesh"); name = Path(g.get("filename")).name
            shutil.copy(UP_MESH / name, mesh_dir / name)
            vis = ET.SubElement(link_el, "visual"); origin_el(vis, M @ T_of(v.find("origin")))
            m = ET.SubElement(ET.SubElement(vis, "geometry"), "mesh", filename=f"meshes/{name}")
            if g.get("scale"):
                m.set("scale", g.get("scale"))
            if color:
                ET.SubElement(ET.SubElement(vis, "material", name=f"{link_el.get('name')}_{name}"), "color", rgba=color)

    def add_collisions(link_el, up_link, M=np.eye(4)):
        cols = [c for c in up_link.findall("collision") if "convex" in c.find("geometry/mesh").get("filename")]
        src = cols if cols else up_link.findall("visual")
        for c in src:
            g = c.find("geometry/mesh"); name = Path(g.get("filename")).name
            shutil.copy(UP_MESH / name, mesh_dir / name)
            if not cols:
                name = hull_file(mesh_dir, name)
            col = ET.SubElement(link_el, "collision"); origin_el(col, M @ T_of(c.find("origin")))
            m = ET.SubElement(ET.SubElement(col, "geometry"), "mesh", filename=f"meshes/{name}")
            if g.get("scale"):
                m.set("scale", g.get("scale"))

    for side in robot_config.enabled_arms(cfg):
        acfg = cfg["arms"][side]; s = CHAINS[acfg["upstream_chain"]]
        color = " ".join(str(v) for v in acfg["color"])

        def lname(up_name, s=s, side=side):
            return f"{side}_arm_{LINKS[up_name.removesuffix(s['suffix']) if s['suffix'] else up_name]}"
        for up_name in LINKS:
            un = up_name + s["suffix"]
            el = ET.SubElement(robot, "link", name=lname(un))
            add_visuals(el, up_links[un], color=color)
            for j in child_of.get(un, []):        # merge the *_geom_1 motor links (fixed, identity)
                ch = j.find("child").get("link")
                if ch.endswith("_geom_1"):
                    add_visuals(el, up_links[ch], T_of(j.find("origin")), color="0.1 0.1 0.1 1")
                    add_collisions(el, up_links[ch], T_of(j.find("origin")))
            add_collisions(el, up_links[un])
            inert = up_links[un].find("inertial")
            if inert is not None:
                el.append(inert)
        # mount on the body: SO101 Base footprint circle on the body's mount pad, arm facing FORWARD (-X)
        j = ET.SubElement(robot, "joint", name=f"{side}_arm_mount", type="fixed")
        ET.SubElement(j, "parent", link="wallerobot_body"); ET.SubElement(j, "child", link=f"{side}_arm_base")
        c = np.array(acfg["mount"]["xyz"], float)
        R = rotation_matrix(np.radians(acfg["mount"]["yaw_deg"]), [0, 0, 1])
        M = translation_matrix(c - R[:3, :3] @ footprint) @ R
        origin_el(j, M)
        # revolute chain
        for uj, motor_s in JOINTS.items():
            u = up_joints[uj + s["jsuffix"]]
            motor = f"{side}_arm_{motor_s}"
            j = ET.SubElement(robot, "joint", name=motor, type="revolute")
            ET.SubElement(j, "parent", link=lname(u.find("parent").get("link")))
            ET.SubElement(j, "child", link=lname(u.find("child").get("link")))
            origin_el(j, T_of(u.find("origin")))
            ET.SubElement(j, "axis", xyz=u.find("axis").get("xyz"))
            ul = u.find("limit"); lo, hi = arm_limits(cal, motor, (float(ul.get("lower")), float(ul.get("upper"))))
            ET.SubElement(j, "limit", lower=f"{lo:.4f}", upper=f"{hi:.4f}", effort=str(jd["effort"]),
                          velocity=str(jd["velocity"]))
        # wrist camera (upstream frame + mesh) and jaw tips
        cj = up_joints[f"fixed_{s['cam']}"]
        cam = ET.SubElement(robot, "link", name=f"{side}_wrist_camera")
        add_visuals(cam, up_links[s["cam"]], color="0.1 0.1 0.1 1")
        add_visuals(cam, up_links[s["cam"] + "_geom_1"], color="0.1 0.1 0.1 1")
        j = ET.SubElement(robot, "joint", name=f"{side}_wrist_camera_joint", type="fixed")
        ET.SubElement(j, "parent", link=f"{side}_arm_fixed_jaw"); ET.SubElement(j, "child", link=f"{side}_wrist_camera")
        origin_el(j, T_of(cj.find("origin")))
        for jaw, xyz in TIP.items():
            ET.SubElement(robot, "link", name=f"{side}_arm_{jaw}_tip")
            j = ET.SubElement(robot, "joint", name=f"{side}_arm_{jaw}_tip_joint", type="fixed")
            ET.SubElement(j, "parent", link=f"{side}_arm_{jaw}"); ET.SubElement(j, "child", link=f"{side}_arm_{jaw}_tip")
            origin_el(j, translation_matrix(xyz))

    # collision hulls for the neck / body links (they only have visuals)
    for link in robot.findall("link"):
        if link.get("name").startswith(("left_", "right_")) or link.find("collision") is not None:
            continue
        for v in link.findall("visual"):
            g = v.find("geometry/mesh"); name = hull_file(mesh_dir, Path(g.get("filename")).name)
            col = ET.SubElement(link, "collision"); col.append(copy.deepcopy(v.find("origin")))
            m = ET.SubElement(ET.SubElement(col, "geometry"), "mesh", filename=f"meshes/{name}")
            if g.get("scale"):
                m.set("scale", g.get("scale"))

    ET.indent(robot)
    path = out / "wallerobot_static.urdf"
    path.write_text("<?xml version=\"1.0\"?>\n<!-- GENERATED by hardware/wallerobot_v0_1/build_full_urdf.py - edit the "
                    "script, not this file. See hardware/wallerobot_v0_1/README.md for assumptions. -->\n"
                    + ET.tostring(robot, encoding="unicode") + "\n")
    print("wrote", path)
    return path


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=OUT, help="output dir (default urdf/)")
    ap.add_argument("--config", type=Path, help="override file merged over robot.yaml")
    a = ap.parse_args()
    cfg = robot_config.load(a.config)
    p = build(a.out, cfg)
    import mujoco
    m = mujoco.MjModel.from_xml_path(str(p))
    names = [m.joint(i).name for i in range(m.njnt)]
    print(f"MuJoCo load OK: {m.njnt} joints, {m.nbody} bodies, {m.ngeom} geoms:", names)
    expect = 6 + 6 * len(robot_config.enabled_arms(cfg))
    assert m.njnt == expect, f"expected {expect} actuated joints"
