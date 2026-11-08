"""Build the MuJoCo twin from the full URDF (hardware/wallerobot_v0_1/urdf/wallerobot_static.urdf).

    .venv/Scripts/python.exe projects/sim_mirror/scripts/build_mjcf.py      # -> models/wallerobot_static.xml + models/scene.xml

- URDF -> MJCF with MuJoCo's own importer (mj_saveLastXML), then post-processed:
  * geoms: `*.hull` / `*convex*` meshes = collision (group 3, invisible), everything else = visual (group 2, no contact)
  * 18 position actuators named like the joints (= plugin motor names), kp 20 N*m/rad, |force| <= 3.35 N*m (STS3215
    stall ~2.9-3.4 N*m @ 12 V), joint damping 1, armature 0.01 (same drive numbers as projects/sim_isaac)
  * cameras: `neck_color` at the URDF neck_camera_color_optical_frame, D435i colour 640x480 vfov 43.2 deg
    (measured intrinsics, hardware.yaml); `left_wrist` / `right_wrist` at the upstream wrist camera links
    looking along the gripper, image up = camera side; fovy / pitch fitted by eye to the real frames, NOT measured
  * contact excludes for body pairs that touch in the rest pose (folded arms / neck on their stops)
- scene.xml: body standing on a desk plane (body bottom z = -0.034 m in the body frame), light, sky.
- Drive, camera and desk numbers come from hardware/wallerobot_v0_1/robot.yaml (`sim`, `cameras`);
  `--config my.yaml --urdf <variant urdf> --out <dir>` builds a variant.
"""
import math
import os
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
from trimesh.transformations import euler_matrix, quaternion_from_matrix, translation_matrix

HERE = Path(__file__).resolve().parent
PROJ = HERE.parent
ROOT = PROJ.parents[1]
sys.path.insert(0, str(ROOT / "hardware"))
import robot_config  # noqa: E402

URDF = ROOT / "hardware/wallerobot_v0_1/urdf/wallerobot_static.urdf"
OUT = PROJ / "models"


def vis_rgba(mesh: str) -> str:
    """Render colour for URDF visuals without a material (body, tower, neck): like the real robot (photo 2026-11-05):
    black PLA prints, black STS3215 motors, grey D435i. Arms keep the upstream SO101 colours. Cosmetic only."""
    if "Motor" in mesh or mesh == "topbase2":        # topbase2 = J1 motor
        return "0.07 0.07 0.08 1"
    if mesh == "tophead6":                            # D435i body
        return "0.32 0.33 0.35 1"
    return "0.17 0.17 0.18 1"


def T_of(o):
    if o is None:
        return np.eye(4)
    return translation_matrix([float(v) for v in o.get("xyz", "0 0 0").split()]) @ \
        euler_matrix(*[float(v) for v in o.get("rpy", "0 0 0").split()], axes="sxyz")


def fixed_chain(urdf_root, body, link):
    """Pose of `link` in movable `body` through fixed joints (URDF)."""
    par = {j.find("child").get("link"): j for j in urdf_root.findall("joint")}
    M = np.eye(4)
    while link != body:
        j = par[link]
        assert j.get("type") == "fixed", f"{link}: non-fixed joint between it and {body}"
        M = T_of(j.find("origin")) @ M; link = j.find("parent").get("link")
    return M


def quat_str(R):
    M = np.eye(4); M[:3, :3] = R
    return " ".join(f"{v:.6f}" for v in quaternion_from_matrix(M))  # w x y z (MuJoCo order)


def add_camera(body_el, name, M, vfov, w, h, optical):
    """optical=True: M is an optical frame (z view, y down); else SAPIEN frame (x view, z up)."""
    R = M[:3, :3]
    if optical:
        Rc = R @ np.diag([1, -1, -1])                 # MuJoCo camera: looks -z, image up = +y
    else:
        Rc = R @ np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], float)  # cam x = -y_link, cam y = z_link, cam -z = x_link
    ET.SubElement(body_el, "camera", name=name, pos=" ".join(f"{v:.6f}" for v in M[:3, 3]), quat=quat_str(Rc),
                  fovy=f"{vfov:.3f}", resolution=f"{w} {h}")


def build(urdf_path=URDF, out=OUT, cfg=None):
    cfg = cfg or robot_config.load()
    sim, ncam, wcam = cfg["sim"], cfg["cameras"]["head"], cfg["cameras"]["wrist"]
    sides = robot_config.enabled_arms(cfg)
    out = Path(out); out.mkdir(parents=True, exist_ok=True)
    m = mujoco.MjModel.from_xml_path(str(urdf_path))
    with tempfile.TemporaryDirectory() as td:
        raw = Path(td) / "raw.xml"; mujoco.mj_saveLastXML(str(raw), m)
        root = ET.parse(raw).getroot()
    urdf = ET.parse(urdf_path).getroot()
    root.set("model", "wallerobot_static")
    comp = root.find("compiler")
    comp.set("meshdir", Path(os.path.relpath(Path(urdf_path).parent / "meshes", out)).as_posix()); comp.set("angle", "radian")
    for k in ("strippath", "discardvisual", "balanceinertia"):
        comp.attrib.pop(k, None)
    ET.SubElement(root, "option", timestep=str(sim["timestep"]), integrator="implicitfast")
    default = root.find("default") if root.find("default") is not None else ET.SubElement(root, "default")
    ET.SubElement(ET.SubElement(default, "default", {"class": "visual"}), "geom", contype="0", conaffinity="0", group="2")
    ET.SubElement(ET.SubElement(default, "default", {"class": "collision"}), "geom", contype="1", conaffinity="1",
                  group="3", rgba="0.8 0.2 0.2 0.3")
    ET.SubElement(default, "joint", damping=str(sim["damping"]), armature=str(sim["armature"]))

    wb = root.find("worldbody")
    for g in wb.iter("geom"):
        mesh = g.get("mesh", "")
        g.set("class", "collision" if ("hull" in mesh or "convex" in mesh) else "visual")
        for k in ("contype", "conaffinity", "group"):
            g.attrib.pop(k, None)
        if g.get("class") == "collision":
            g.attrib.pop("rgba", None)                 # class colour: translucent red when group 3 is shown
        elif g.get("rgba") is None and g.get("material") is None:
            g.set("rgba", vis_rgba(mesh))              # uncoloured URDF parts (body, tower, neck) -> real colours
    bodies = {b.get("name"): b for b in wb.iter("body")}

    # cameras
    T_neck = fixed_chain(urdf, "neck_cam_holder", "neck_camera_color_optical_frame")
    add_camera(bodies["neck_cam_holder"], "neck_color", T_neck, ncam["vfov_deg"], ncam["width"], ncam["height"], True)
    for side in sides:
        # position = upstream wrist camera link; orientation from the real images (mirror 2026-11-05): the camera
        # looks along the gripper (Fixed_Jaw -Y, towards the jaw tip), image up = camera side (+Z, camera sits on
        # top of the gripper), tilted cameras.wrist.pitch_deg towards the jaws so they show at the bottom of the image
        T_w = fixed_chain(urdf, f"{side}_arm_fixed_jaw", f"{side}_wrist_camera")
        p = np.radians(wcam["pitch_deg"])
        f = np.array([0, -np.cos(p), -np.sin(p)]); u = np.array([0, -np.sin(p), np.cos(p)])  # view dir, image up
        Rc = np.column_stack([np.cross(u, -f), u, -f])     # MuJoCo camera axes: x right, y up, z backwards
        M = np.eye(4); M[:3, :3] = Rc; M[:3, 3] = T_w[:3, 3]
        ET.SubElement(bodies[f"{side}_arm_fixed_jaw"], "camera", name=f"{side}_wrist",
                      pos=" ".join(f"{v:.6f}" for v in M[:3, 3]), quat=quat_str(Rc),
                      fovy=f"{wcam['vfov_deg']:.3f}", resolution=f"{wcam['width']} {wcam['height']}")
    # tip sites (for IK / gripper checks)
    for side in sides:
        for jaw in ("fixed_jaw", "moving_jaw"):
            M = fixed_chain(urdf, f"{side}_arm_{jaw}", f"{side}_arm_{jaw}_tip")
            ET.SubElement(bodies[f"{side}_arm_{jaw}"], "site", name=f"{side}_{jaw}_tip", size="0.004",
                          pos=" ".join(f"{v:.6f}" for v in M[:3, 3]), group="4")

    # actuators
    act = ET.SubElement(root, "actuator")
    joints = [j for j in wb.iter("joint")]
    for j in joints:
        lo, hi = (float(v) for v in j.get("range").split())
        ET.SubElement(act, "position", name=j.get("name"), joint=j.get("name"), kp=str(sim["kp"]),
                      ctrlrange=f"{lo:.4f} {hi:.4f}", forcerange=f"{-sim['force_limit']} {sim['force_limit']}")
    path = out / "wallerobot_static.xml"
    ET.indent(root); path.write_text("<!-- GENERATED by projects/sim_mirror/scripts/build_mjcf.py from "
                                     "hardware/wallerobot_v0_1/urdf/wallerobot_static.urdf - do not edit -->\n"
                                     + ET.tostring(root, encoding="unicode"))
    scene = out / "scene.xml"
    scene.write_text(f"""<mujoco model="wallerobot_scene">
  <include file="wallerobot_static.xml"/>
  <visual>
    <headlight ambient="0.25 0.25 0.25" diffuse="0.35 0.35 0.35" specular="0.05 0.05 0.05"/>
    <global offwidth="1280" offheight="960"/><quality shadowsize="4096"/>
  </visual>
  <asset>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.9 0.9 0.92" rgb2="0.75 0.78 0.82" width="256" height="256"/>
    <texture name="desk" type="2d" builtin="checker" rgb1="0.82 0.81 0.78" rgb2="0.76 0.75 0.72" width="256" height="256"/>
    <material name="desk" texture="desk" texrepeat="8 8" specular="0.1" shininess="0.1"/>
  </asset>
  <worldbody>
    <light pos="-0.9 -0.7 1.5" dir="0.5 0.4 -0.77" cutoff="40" exponent="2" diffuse="0.6 0.6 0.6" castshadow="true"/>
    <light pos="0.6 0.8 1.2" dir="-0.4 -0.5 -0.75" directional="true" diffuse="0.2 0.2 0.22" castshadow="false"/>
    <!-- placeholder desk: plane under the body (real desk size / height not measured) -->
    <geom name="desk" type="plane" pos="0 0 {sim['desk_z']}" size="1.5 1.5 0.05" material="desk" contype="1" conaffinity="1"/>
  </worldbody>
</mujoco>
""")
    return path, scene


def add_rest_excludes(path, scene, rest_q: dict):
    """Find body pairs whose collision hulls touch in the rest pose and exclude them (folded joints on stops)."""
    m = mujoco.MjModel.from_xml_path(str(scene)); d = mujoco.MjData(m)
    for n, v in rest_q.items():
        if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) < 0:
            continue                                  # joint not in this variant (e.g. an arm disabled)
        d.qpos[m.jnt_qposadr[m.joint(n).id]] = v
    mujoco.mj_forward(m, d)
    pairs = set()
    for c in d.contact[:d.ncon]:
        b1, b2 = m.body(m.geom_bodyid[c.geom1]).name, m.body(m.geom_bodyid[c.geom2]).name
        if "world" in (b1, b2) and "desk" in (m.geom(c.geom1).name, m.geom(c.geom2).name):
            continue
        pairs.add(tuple(sorted((b1, b2))))
    root = ET.parse(path).getroot()
    cont = root.find("contact") if root.find("contact") is not None else ET.SubElement(root, "contact")
    for b1, b2 in sorted(pairs):
        ET.SubElement(cont, "exclude", body1=b1, body2=b2)
    ET.indent(root); path.write_text(path.read_text().split("\n", 1)[0] + "\n" + ET.tostring(root, encoding="unicode"))
    return sorted(pairs)


def build_all(urdf_path=URDF, out=OUT, cfg=None):
    """build() + rest-pose contact excludes. Returns (robot xml, scene xml, excluded pairs)."""
    import json
    cfg = cfg or robot_config.load()
    path, scene = build(urdf_path, out, cfg)
    # URDF angles of the measured rest pose, not its raw ticks: ticks change meaning with every recalibration
    # (2026-11-07 J3/J4 reassembly: old ticks + new zeros put J3 8 deg into the fold -> spurious excludes)
    rest = json.loads(robot_config.path(cfg["sim"]["rest_pose"]).read_text())["q_deg"]
    pairs = add_rest_excludes(path, scene, {n: math.radians(v) for n, v in rest.items()})
    return path, scene, pairs


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", type=Path, help="override file merged over hardware/wallerobot_v0_1/robot.yaml")
    ap.add_argument("--urdf", type=Path, default=URDF)
    ap.add_argument("--out", type=Path, default=OUT)
    a = ap.parse_args()
    path, scene, pairs = build_all(a.urdf, a.out, robot_config.load(a.config))
    print("rest-pose contact excludes:", pairs)
    m = mujoco.MjModel.from_xml_path(str(scene))
    print(f"wrote {path}\n      {scene}\nMuJoCo OK: {m.njnt} joints, {m.nu} actuators, {m.ncam} cameras "
          f"{[m.camera(i).name for i in range(m.ncam)]}, {m.ngeom} geoms")
