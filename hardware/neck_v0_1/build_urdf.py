"""Build the neck_v0_1 URDF draft from upstream SO101 / XLeRobot frames + our STL measurements.

    .venv/Scripts/python.exe hardware/neck_v0_1/build_urdf.py   # writes urdf/wallerobot_neck.urdf + urdf/meshes/

Chain: J1-J3 = stock SO101 (upstream xlerobot.urdf, left arm), J3->J4 = our new link (wallerobot_new_wrist_dof_0_1.stl),
J4-J6 = stock SO101 wrist mounted on the J4 motor exactly as in SO101, J6 -> D435i = upstream XLeRobot head-tilt holder.
Root frame = wallerobot_body (= upstream XLeRobot top_base_link, same frame as the upstream arm bases); neck_base
(SO101 Base frame, J1) is fixed on the XLeRobot tower top where the head-pan motor was. Numbers marked ASSUMPTION are unverified on the real robot (see README.md).
Joint zeros: J1, J2 = SO101 URDF zeros; J3 = 0 when the J4 motor's long axis is in line with the upper arm;
J4 = 0 when the wrist continues straight along the J4 motor (as SO101); J5 = 0 in the user's normal state (camera landscape, upright, below the J6 housing);
J6 = 0 when the camera looks along the J5 roll axis, away from the wrist.

All robot numbers (mount, new-link points, orientation flags, camera offset, calibration file) come from
hardware/wallerobot_v0_1/robot.yaml (`neck`, `cameras.head`, `calibration.neck`); the comments below record where the
default values came from. `--config my.yaml` merges an override file."""
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import sys

import numpy as np
from trimesh.transformations import euler_from_matrix, euler_matrix, rotation_matrix, translation_matrix

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "hardware"))
import robot_config  # noqa: E402

OUT = HERE / "urdf"
# Upstream XLeRobot URDF + meshes, vendored in hardware/upstream/xlerobot (the SO101 meshes there are byte-identical to
# the ones the neck used before from projects/sim_isaac/assets, so one directory serves arms, neck and head parts).
HEAD_MESH = robot_config.path(robot_config.load()["upstream"]["meshes"])

# Where the robot.yaml `neck` defaults come from (README.md has the measurements):
# - link34.j3_axis_mm: fork horn centre; Y = middle of the fork (J3 motor centred, as SO101).
#   link34.j4_shaft_mm: ASSUMPTION, J4 motor's non-shaft end on the pocket floor (X 15).
# - link34.side -1: inferred from the user photo (J4 motor on the far side from the upper arm when J3 folds;
#   +1 would make the J4 motor intersect the upper arm at J3 = 160 deg). It flips the 16.85 mm J3-J4 offset.
# - link34.j4_shaft_sign -1: J4 output side faces part -Z; user 2026-11-04 compared guess_j4_AB.png -> "B".
# - camera_holder.flip -1: holder mirrored vs the old head tilt motor (user: "pink is reversed"; the camera looks forward
#   with its body below-forward of the J6 horn, only possible with -1).
# - j5_zero_shift_deg -90: J5 = 0 = user's 'normal state' (variant C in guess_j5_ABCD.png, 2026-11-04): J6 horizontal,
#   holder + camera BELOW the J6 housing (matches the folded photo), camera upright.
# - camera_holder.camera_flip: camera upright in state C -> D435i sits rotated 180 deg about its optical axis vs upstream.
# - camera_holder.axial_shift_mm -2.5: STS3215 centre along its shaft: tilt motor (tophead4) y = +2.0 mm in
#   head_tilt_link, J6 motor z = -0.5 mm in Moving_Jaw.
# - mount: SO101 Base_Motor.stl ICP-matched onto upstream topbase2 (0.44 mm median) -> neck_base pose in upstream
#   top_base_link (= wallerobot_body). J1 axis lands at x -104.1 mm (upstream head_pan_joint -103 mm). User: the blue
#   SO101 Base is replaced by the XLeRobot neck part, the J1 motor sits where the old head-pan motor was.

# J6 housing = XLeRobot TPU-finger fixed jaw (third_party/XLeRobot/hardware/SO101_soft_fin.stl, largest body, finger
# removed) exported as HOUSING_STL (mm). Pose in the SO101 Fixed_Jaw frame found by ICP (0.39 mm median), rotation snapped.
HOUSING_STL = "cam_tilt_housing_softfin.stl"
T_FJ_HOUSING = np.array([[0, 0, -1, 0.03553], [0, 1, 0, -0.20075], [1, 0, 0, -0.07985], [0, 0, 0, 1]], float)
TOPBASE_VIS = {"topbase1.stl": translation_matrix((-0.44, -0.005, -0.6)),  # upstream visual origins
               "topbase2.stl": translation_matrix((-0.298, 0.006, -0.715)) @ euler_matrix(0, 0, 3.14)}
# Real base (user 2026-11-05, candidate 4 in guess_base_candidates.png): truss tower + the J1 housing of upstream
# Archiv/Neck.step, made by make_base_meshes.py in the body frame (mm). topbase1 is replaced by its part below the tower;
# topbase2 (= old head-pan motor, where the J1 motor sits) is kept as the J1 motor visual.
BASE_STLS = ["base_body_lower.stl", "base_tower.stl", "base_j1_housing.stl"]


def T(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    return translation_matrix(xyz) @ euler_matrix(*rpy, axes="sxyz")


def origin(M):
    xyz = " ".join(f"{v:.6f}" for v in M[:3, 3])
    rpy = " ".join(f"{v:.6f}" for v in euler_from_matrix(M, axes="sxyz"))
    return {"xyz": xyz, "rpy": rpy}


def upstream_joints(urdf):
    out = {}
    for j in ET.parse(urdf).getroot().findall("joint"):
        o, a, lim = j.find("origin"), j.find("axis"), j.find("limit")
        out[j.get("name")] = {
            "T": T([float(v) for v in o.get("xyz").split()], [float(v) for v in o.get("rpy").split()]),
            "axis": a.get("xyz") if a is not None else None,
            "limit": (float(lim.get("lower")), float(lim.get("upper"))) if lim is not None and lim.get("lower") else None,
        }
    return out


def link34_frames(UP, nk):
    """Return (T_L_stl, T_L_motor4, T_L_j4) in the new link frame L (= SO101 Lower_Arm convention: origin on the J3 axis,
    X = J3 axis, J4 motor towards +Z). Mesh poses are for meshes in metres (STL is scaled by 0.001 in the URDF)."""
    l34 = nk["link34"]
    J3_AXIS_PT, J4_SHAFT_PT = np.array(l34["j3_axis_mm"], float), np.array(l34["j4_shaft_mm"], float)
    LINK34_SIDE, J4_SHAFT_SIGN = l34["side"], l34["j4_shaft_sign"]
    X, Y, Z = np.eye(3)
    # L: X = J3 axis (part Y), Z = J4 motor long axis (part -X) -> J3 = 0 puts the J4 motor in line with the upper arm,
    # exactly like the SO101 Lower_Arm_Motor (identity rotation in Lower_Arm)
    xl, zl = LINK34_SIDE * Y, -X
    yl = np.cross(zl, xl)
    so_j4 = UP["Wrist_Pitch_L"]["T"]                    # SO101: J4 at (0, 5.2, 134.9) mm in Lower_Arm
    R = np.vstack([xl, yl, zl])                         # part -> L rotation
    assert np.isclose(np.linalg.det(R), 1)
    T_L_stl = np.eye(4); T_L_stl[:3, :3] = R; T_L_stl[:3, 3] = -R @ J3_AXIS_PT / 1000
    # J4 motor frame (as SO101 Lower_Arm_Motor around its J4 joint): x = shaft, y = width, z = long axis towards the shaft end
    A = np.column_stack([J4_SHAFT_SIGN * Z, J4_SHAFT_SIGN * Y, -X])
    T_L_M = np.eye(4); T_L_M[:3, :3] = R @ A; T_L_M[:3, 3] = R @ (J4_SHAFT_PT - J3_AXIS_PT) / 1000
    assert np.isclose(np.linalg.det(T_L_M[:3, :3]), 1)
    so_M = translation_matrix(so_j4[:3, 3])             # SO101 motor frame in Lower_Arm (identity rotation)
    motor_visual = T_L_M @ np.linalg.inv(so_M)          # pose of Lower_Arm_Motor.stl (authored in Lower_Arm frame)
    return T_L_stl, motor_visual, motor_visual @ so_j4


def holder_frame(UP, nk):
    """Pose of the old head_tilt_link frame inside the J6 child frame (SO101 Moving_Jaw convention, J6 axis = z)."""
    HOLDER_AXIAL_SHIFT = nk["camera_holder"]["axial_shift_mm"] / 1000
    HOLDER_FLIP = nk["camera_holder"]["flip"]
    R_fj_mj = UP["Jaw_L"]["T"][:3, :3]
    out = R_fj_mj.T @ np.array([0, -1, 0])              # J5 roll axis, pointing away from the wrist, in Moving_Jaw
    alpha = np.arctan2(out[1], out[0])
    return translation_matrix([0, 0, HOLDER_AXIAL_SHIFT]) @ rotation_matrix(alpha, [0, 0, 1]) @ \
        rotation_matrix(HOLDER_FLIP * np.pi / 2, [1, 0, 0])  # head tilt axis y -> J6 axis z


def build(out=OUT, cfg=None):
    cfg = cfg or robot_config.load()
    nk = cfg["neck"]
    UP = upstream_joints(robot_config.path(cfg["upstream"]["urdf"]))
    up_mesh = robot_config.path(cfg["upstream"]["meshes"])
    link34_src = robot_config.path(nk["link34"]["stl"]); LINK34_STL = link34_src.name
    J5_ZERO_SHIFT = np.radians(nk["j5_zero_shift_deg"])
    PLACEHOLDER_LIMIT = tuple(nk["placeholder_limit_rad"])
    T_BODY_NECKBASE = T(nk["mount"]["xyz"], np.radians(nk["mount"]["rpy_deg"]))
    cam_off = cfg["cameras"]["head"]["offset_in_holder_m"]
    jd = cfg["joint_defaults"]

    T_L_stl, motor4_visual, T_L_j4 = link34_frames(UP, nk)
    T_h = holder_frame(UP, nk)
    head_visual = T([0.16265, -0.006, -1.16815])        # tophead5/6 visual origin in upstream head_tilt_link
    T_cam = T_h @ T(cam_off)                            # upstream head_camera_link (x = view direction)
    if nk["camera_holder"]["camera_flip"]:
        T_cam = T_cam @ rotation_matrix(np.pi, [1, 0, 0])
    cam_visual = T_cam @ np.linalg.inv(T_h @ T(cam_off)) @ T_h @ head_visual  # D435i body follows the camera
    I = np.eye(4)
    links = {
        "wallerobot_body": [(f, I, 0.001) for f in BASE_STLS] +  # upstream top_base_link frame
                           [("topbase2.stl", TOPBASE_VIS["topbase2.stl"], 0.001)],
        "neck_base": [],
        "neck_shoulder": [("Rotation_Pitch.stl", I, None), ("Rotation_Pitch_Motor.stl", I, None)],
        "neck_upper_arm": [("Upper_Arm.stl", I, None), ("Upper_Arm_Motor.stl", I, None)],
        "neck_link34": [(LINK34_STL, T_L_stl, 0.001), ("Lower_Arm_Motor.stl", motor4_visual, None)],
        "neck_wrist": [("Wrist_Pitch_Roll.stl", I, None), ("Wrist_Pitch_Roll_Motor.stl", I, None)],
        "neck_cam_tilt_housing": [(HOUSING_STL, T_FJ_HOUSING, 0.001), ("Fixed_Jaw_Motor.stl", I, None)],
        "neck_cam_holder": [("tophead5.stl", T_h @ head_visual, 0.001), ("tophead6.stl", cam_visual, 0.001)],
        "neck_camera_link": [],
        "neck_camera_color_optical_frame": [],
    }
    joints = [  # name, parent, child, origin, axis, limit (None = fixed)
        ("neck_mount", "wallerobot_body", "neck_base", T_BODY_NECKBASE, None, None),
        ("neck_j1", "neck_base", "neck_shoulder", UP["Rotation_L"]["T"], UP["Rotation_L"]["axis"], UP["Rotation_L"]["limit"]),
        ("neck_j2", "neck_shoulder", "neck_upper_arm", UP["Pitch_L"]["T"], UP["Pitch_L"]["axis"], UP["Pitch_L"]["limit"]),
        ("neck_j3", "neck_upper_arm", "neck_link34", UP["Elbow_L"]["T"], UP["Elbow_L"]["axis"], UP["Elbow_L"]["limit"]),
        ("neck_j4", "neck_link34", "neck_wrist", T_L_j4, UP["Wrist_Pitch_L"]["axis"], PLACEHOLDER_LIMIT),
        ("neck_j5", "neck_wrist", "neck_cam_tilt_housing",
         UP["Wrist_Roll_L"]["T"] @ rotation_matrix(J5_ZERO_SHIFT, [float(v) for v in UP["Wrist_Roll_L"]["axis"].split()]),
         UP["Wrist_Roll_L"]["axis"], None if UP["Wrist_Roll_L"]["limit"] is None else
         tuple(v - J5_ZERO_SHIFT for v in UP["Wrist_Roll_L"]["limit"])),
        ("neck_j6", "neck_cam_tilt_housing", "neck_cam_holder", UP["Jaw_L"]["T"], UP["Jaw_L"]["axis"], PLACEHOLDER_LIMIT),
        ("neck_camera_joint", "neck_cam_holder", "neck_camera_link", T_cam, None, None),
        ("neck_camera_color_optical_joint", "neck_camera_link", "neck_camera_color_optical_frame",
         T(rpy=(-np.pi / 2, 0, -np.pi / 2)), None, None),
    ]

    robot = ET.Element("robot", name="wallerobot_neck")
    mj = ET.SubElement(robot, "mujoco")
    ET.SubElement(mj, "compiler", meshdir="meshes/", strippath="true", discardvisual="false", balanceinertia="true")
    for name, visuals in links.items():
        link = ET.SubElement(robot, "link", name=name)
        inertial = ET.SubElement(link, "inertial")  # placeholder inertia so simulators accept the draft
        ET.SubElement(inertial, "origin", xyz="0 0 0", rpy="0 0 0")
        ET.SubElement(inertial, "mass", value="0.05" if visuals else "0.001")
        ET.SubElement(inertial, "inertia", ixx="1e-5", ixy="0", ixz="0", iyy="1e-5", iyz="0", izz="1e-5")
        for mesh, M, scale in visuals:
            vis = ET.SubElement(link, "visual")
            ET.SubElement(vis, "origin", **origin(M))
            geo = ET.SubElement(ET.SubElement(vis, "geometry"), "mesh", filename=f"meshes/{mesh}")
            if scale:
                geo.set("scale", f"{scale} {scale} {scale}")
    calib_json = robot_config.path(cfg["calibration"]["neck"])  # measured limits replace the placeholders
    cal = json.loads(calib_json.read_text(encoding="utf-8"))["joints"] if calib_json.exists() else {}
    for name, parent, child, M, axis, limit in joints:
        if limit and name in cal:
            limit = (cal[name]["lower_rad"], cal[name]["upper_rad"])
        j = ET.SubElement(robot, "joint", name=name, type="revolute" if limit else "fixed")
        ET.SubElement(j, "parent", link=parent)
        ET.SubElement(j, "child", link=child)
        ET.SubElement(j, "origin", **origin(M))
        if limit:
            ET.SubElement(j, "axis", xyz=axis)
            ET.SubElement(j, "limit", lower=f"{limit[0]:.4f}", upper=f"{limit[1]:.4f}", effort=str(jd["effort"]), velocity=str(jd["velocity"]))

    (out / "meshes").mkdir(parents=True, exist_ok=True)
    for visuals in links.values():
        for mesh, _, _ in visuals:
            src = link34_src if mesh == LINK34_STL else HERE / mesh if mesh in (HOUSING_STL, *BASE_STLS) else \
                up_mesh / mesh
            shutil.copy(src, out / "meshes" / mesh)
    ET.indent(robot)
    path = out / "wallerobot_neck.urdf"
    header = ("<?xml version=\"1.0\"?>\n<!-- GENERATED by hardware/neck_v0_1/build_urdf.py - edit the script, not this file. "
              "Draft v0.1: see hardware/neck_v0_1/README.md for assumptions. -->\n")
    path.write_text(header + ET.tostring(robot, encoding="unicode") + "\n")
    print("wrote", path)
    return path


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=OUT, help="output dir (default urdf/; use a scratch dir for variants)")
    ap.add_argument("--config", type=Path, help="override file merged over hardware/wallerobot_v0_1/robot.yaml")
    ap.add_argument("--side", type=int, help="override neck.link34.side")
    ap.add_argument("--j4-sign", type=int, help="override neck.link34.j4_shaft_sign")
    ap.add_argument("--holder-flip", type=int, help="override neck.camera_holder.flip")
    ap.add_argument("--j5-shift-deg", type=float, help="override neck.j5_zero_shift_deg")
    a = ap.parse_args()
    cfg = robot_config.load(a.config)
    for v, keys in ((a.side, ("link34", "side")), (a.j4_sign, ("link34", "j4_shaft_sign")),
                    (a.holder_flip, ("camera_holder", "flip")), (a.j5_shift_deg, ("j5_zero_shift_deg",))):
        if v is not None:
            d = cfg["neck"]
            for k in keys[:-1]:
                d = d[k]
            d[keys[-1]] = v
    p = build(a.out, cfg)
    import mujoco
    m = mujoco.MjModel.from_xml_path(str(p))
    print("MuJoCo load OK:", m.njnt, "joints:", [m.joint(i).name for i in range(m.njnt)])
