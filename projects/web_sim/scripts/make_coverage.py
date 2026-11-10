"""Why active perception: how much of the desk scene can each camera setup see? Writes web/public/why/.

Six setups, all with the same RealSense D435i colour camera model (vfov 43.2 deg, 4:3, range 0.10-1.5 m), all
mounted at the same place (the XLeRobot head / LeRobotNeck J1 position on top of the tower), in the same desk scene
as the play/ demos (routes_neck/scene.json, from make_neck_routes.scene_props):

  fixed    the XLeRobot head camera with its motors frozen, tilted down at the scene
  head2    the XLeRobot 2-axis head (pan + tilt), upstream xlerobot.urdf head_pan / head_tilt joints
  so101    a stock SO101 arm (5 joints, gripper unused) with a camera at the XLeRobot wrist-camera mount on the fixed
           jaw (pose copied from the full wallerobot model's right_wrist camera)
  so101x   SO101-X: the stock SO101 with the LeRobotNeck camera holder + D435i in place of the moving jaw, so the
           gripper motor tilts the camera (6 joints, the holder's J6 range); the stock fixed jaw stays
  neck     LeRobotNeck (6 joints, D435i at neck_color)
  neck02   LeRobotNeck-02 (concept): the J3-J4 link as long along the arm as the stock lower arm (+7.4 cm), the J4
           motor back at its stock place, a box as the extra link

The scene surfaces are sampled as points (desk + every object). For every reachable camera pose (head: a pan x tilt
grid; arms: uniform random joint samples that keep every body clear of the desk, the objects and the tower) a point
counts as seen when it is in the image, 0.10-1.5 m away, faces the camera (< 80 deg from its normal) and the ray to
it hits nothing else (MuJoCo mj_multiRay against the desk, the objects, the tower and the setup's own moving parts;
the body carrying the camera is skipped). Outputs per setup: which points are seen, from how many different
directions (32 bins), the 10 views that cover the most (greedy), coverage per region, the same coverage using only
upright images (horizon within +-15 deg, the way most perception models are trained), how much of the image the
robot itself covers, and how many of 80 requested views (camera at P, looking at A; IK from 12 random starts) it can
take, with and without a level horizon. Run after make_neck_routes.py (about 5 min). Simulation only.
"""
import base64
import json
import shutil
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from make_neck_routes import DESK, NECK, scene_props  # noqa: E402

ROOT = Path(__file__).resolve().parents[3]
WEB = ROOT / "projects/web_sim/web/public"
OUT = WEB / "why"
URDF = ROOT / "hardware/upstream/xlerobot/xlerobot.urdf"
FULL = WEB / "model/scene.xml"

VFOV, ASPECT, NEAR, FAR = 43.2, 4 / 3, 0.10, 1.5
FACING = np.cos(np.radians(80))
N_ARM = 30000          # random joint samples per arm setup
SPACING = 0.008        # object surface point spacing (m); the desk uses DESK_STEP
DESK_STEP = 0.012
DESK_AREA = ((-0.70, -0.08), (-0.45, 0.45))
N_GREEDY = 10
UPRIGHT_DEG = 15       # "upright image": horizon tilted less than this
GROUP = 4              # scene geoms (desk + objects) live in this geom group
N_TASKS = 80           # requested views: camera within POS_TOL of P, looking at A within AIM_TOL
AIM_TOL, POS_TOL, LEVEL_TOL = 3.0, 1.5, 5.0   # deg, cm, deg (horizon)


def b64(a):
    return base64.b64encode(np.ascontiguousarray(a).tobytes()).decode()


def rpy_quat(r, p, y):
    q = np.zeros(4)
    R = (np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
         @ np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
         @ np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]]))
    mujoco.mju_mat2Quat(q, R.ravel())
    return q


def urdf_head():
    """Head joint frames, limits and visual meshes from the upstream XLeRobot URDF (frame = top_base_link = body)."""
    import xml.etree.ElementTree as ET
    root = ET.parse(URDF).getroot()
    J = {j.get("name"): j for j in root.iter("joint")}

    def origin(el):
        o = el.find("origin")
        return [float(v) for v in o.get("xyz").split()], [float(v) for v in o.get("rpy", "0 0 0").split()]

    out = {}
    for k, n in (("pan", "head_pan_joint"), ("tilt", "head_tilt_joint")):
        xyz, rpy = origin(J[n])
        lim = J[n].find("limit")
        out[k] = {"pos": xyz, "quat": rpy_quat(*rpy).tolist(), "axis": [float(v) for v in J[n].find("axis").get("xyz").split()],
                  "range": [float(lim.get("lower")), float(lim.get("upper"))]}
    xyz, rpy = origin(J["fixed_head_camera_link"])
    assert rpy == [0, 0, 0]
    out["camera_link_pos"] = xyz
    meshes = []
    for link, owner in (("head_pan_link_geom_1", "pan"), ("head_tilt_link", "tilt"), ("head_tilt_link_geom_1", "tilt")):
        el = next(e for e in root.iter("link") if e.get("name") == link)
        v = el.find("visual")
        xyz, rpy = origin(v)
        f = Path(v.find("geometry/mesh").get("filename")).name
        meshes.append({"file": f, "link": owner, "pos": xyz, "quat": rpy_quat(*rpy).tolist(), "scale": 0.001})
    out["meshes"] = meshes
    return out


def cam_quat_forward_up(f, u):
    """MuJoCo camera quat for view direction f and image-up u (camera looks along -z, image up = +y)."""
    f, u = np.asarray(f, float), np.asarray(u, float)
    R = np.column_stack([np.cross(f, u), u, -f])
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, R.ravel())
    return q


G_WORLD, G_NECK, G_STOCK, G_OTHER = 0, 1, 3, 5   # geom groups for the ray tests (scene = GROUP)
NECK_BODIES = ["neck_shoulder", "neck_upper_arm", "neck_link34", "neck_wrist", "neck_cam_tilt_housing", "neck_cam_holder"]
STOCK_BODIES = ["stock_shoulder", "stock_upper_arm", "stock_lower_arm", "stock_wrist", "stock_fixed_jaw", "stock_moving_jaw"]
STOCK_JOINTS = ["stock_shoulder_pan", "stock_shoulder_lift", "stock_elbow_flex", "stock_wrist_flex", "stock_wrist_roll"]
# LeRobotNeck-02 spacer (concept part filling the longer J3-J4 link), box in the neck_link34 frame: between the end of
# the printed link (z 0.041) and the J4 motor moved back to its stock place (z 0.0997)
SPACER = {"pos": [0.0, 0.014, 0.070], "half": [0.019, 0.017, 0.035]}


def build_model(props, head, variant=None):
    """variant None: LeRobotNeck + stock SO101 + XLeRobot head. 'x': SO101-X, the stock moving jaw replaced by the
    LeRobotNeck camera holder + D435i (gripper motor = camera tilt, with the holder's J6 range). 'long': LeRobotNeck-02,
    the J3-J4 link as long along the arm as the stock lower arm (J4 motor back at its stock place)."""
    spec = mujoco.MjSpec.from_file(str(WEB / "model_neck/scene.xml"))
    spec.geom("desk").group = GROUP
    extend = None
    if variant == "x":
        jaw = spec.body("stock_moving_jaw")
        for g in list(jaw.geoms):
            if g.meshname == "Moving_Jaw":
                spec.delete(g)
        for h in spec.body("neck_cam_holder").geoms:
            if h.meshname:
                g = jaw.add_geom(); g.type = mujoco.mjtGeom.mjGEOM_MESH; g.meshname = h.meshname
                g.pos, g.quat, g.contype, g.conaffinity = list(h.pos), list(h.quat), 0, 0
        nc = spec.camera("neck_color")
        jaw.add_camera(name="so101x_cam", pos=list(nc.pos), quat=list(nc.quat), fovy=VFOV)
        spec.joint("stock_gripper").range = list(spec.joint("neck_j6").range)
    elif variant == "long":
        w = spec.body("neck_wrist"); pos = list(w.pos)
        extend = float(spec.body("stock_wrist").pos[2] - pos[2])
        pos[2] += extend; w.pos = pos
        link = spec.body("neck_link34")
        for g in link.geoms:
            if g.meshname == "Lower_Arm_Motor":
                p = list(g.pos); p[2] += extend; g.pos = p
        g = link.add_geom(); g.name = "spacer"; g.type = mujoco.mjtGeom.mjGEOM_BOX
        g.pos, g.size, g.contype, g.conaffinity = SPACER["pos"], SPACER["half"], 0, 0
    wb = spec.worldbody
    for p in props:
        g = wb.add_geom()
        g.name = f"prop_{p['id']}"
        g.group, g.contype, g.conaffinity = GROUP, 0, 0
        g.pos = p["pos"]
        yaw = p.get("yaw", 0.0)
        g.quat = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
        s = p["size"]
        if p["shape"] == "box":
            g.type, g.size = mujoco.mjtGeom.mjGEOM_BOX, [s[0] / 2, s[1] / 2, s[2] / 2]
        elif p["shape"] == "sphere":
            g.type, g.size = mujoco.mjtGeom.mjGEOM_SPHERE, [s[0], 0, 0]
        else:
            g.type, g.size = mujoco.mjtGeom.mjGEOM_CYLINDER, [s[0], s[1] / 2, 0]
    # XLeRobot 2-axis head (pan about z, tilt about the tilt link y); camera at head_camera_link looking along +x
    pan = wb.add_body(name="xle_head_pan", pos=head["pan"]["pos"], quat=head["pan"]["quat"])
    pan.add_joint(name="head_pan", type=mujoco.mjtJoint.mjJNT_HINGE, axis=head["pan"]["axis"], range=head["pan"]["range"])
    tilt = pan.add_body(name="xle_head_tilt", pos=head["tilt"]["pos"], quat=head["tilt"]["quat"])
    tilt.add_joint(name="head_tilt", type=mujoco.mjtJoint.mjJNT_HINGE, axis=head["tilt"]["axis"], range=head["tilt"]["range"])
    for b in (pan, tilt):
        b.mass, b.inertia = 0.05, [1e-5] * 3
    tilt.add_camera(name="head_cam", pos=head["camera_link_pos"], quat=cam_quat_forward_up([1, 0, 0], [0, 0, 1]),
                    fovy=VFOV)
    # stock SO101 + camera at the XLeRobot wrist-camera mount (same Fixed_Jaw frame as the full model's arms)
    full = mujoco.MjModel.from_xml_path(str(FULL))
    wc = full.camera("right_wrist").id
    assert full.body(full.cam_bodyid[wc]).name == "right_arm_fixed_jaw"
    spec.body("stock_fixed_jaw").add_camera(name="so101_cam", pos=full.cam_pos[wc].tolist(),
                                            quat=full.cam_quat[wc].tolist(), fovy=VFOV)
    m = spec.compile()
    m.cam_fovy[m.camera("neck_color").id] = VFOV
    for g in range(m.ngeom):   # ray groups: scene / tower (world) / neck chain / stock chain
        if m.geom_group[g] != GROUP:
            b = m.body(m.geom_bodyid[g]).name
            m.geom_group[g] = (G_NECK if b.startswith("neck_") else G_STOCK if b.startswith("stock_")
                               else G_WORLD if b == "world" else G_OTHER)
    return m, full.cam_pos[wc].tolist(), full.cam_quat[wc].tolist()


def inside(p, prop, pad=0.0):
    """Points (N,3) inside a prop primitive (yaw-aware), grown by `pad`."""
    c = np.asarray(prop["pos"]); yaw = prop.get("yaw", 0.0); s = prop["size"]
    d = p - c
    x = d[:, 0] * np.cos(yaw) + d[:, 1] * np.sin(yaw); y = -d[:, 0] * np.sin(yaw) + d[:, 1] * np.cos(yaw); z = d[:, 2]
    if prop["shape"] == "box":
        return (abs(x) < s[0] / 2 + pad) & (abs(y) < s[1] / 2 + pad) & (abs(z) < s[2] / 2 + pad)
    if prop["shape"] == "sphere":
        return np.sqrt(x * x + y * y + z * z) < s[0] + pad
    return (np.hypot(x, y) < s[0] + pad) & (abs(z) < s[1] / 2 + pad)


def surface_points(props):
    """Points + outward normals + region on the desk and every object's visible faces."""
    P, N, R = [], [], []
    regions = ["desk", "box_inside", "box_outside", "block", "sticker", "objects"]
    box_c = next(np.array(p["pos"]) for p in props if p["id"] == "box_floor")

    def region(prop, pt, n):
        i = prop["id"]
        if i == "sticker":
            return 4
        if i == "block":
            return 3
        if i in ("ball", "cube", "can"):
            return 1
        if i == "box_floor":
            return 1
        if i.startswith("box_wall"):
            if n[2] > 0.5:
                return 2
            return 1 if (box_c[:2] - pt[:2]) @ n[:2] > 0 else 2
        return 5

    for p in props:
        c = np.asarray(p["pos"]); yaw = p.get("yaw", 0.0); s = p["size"]
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        pts, nrm = [], []
        if p["shape"] == "box":
            h = np.array(s) / 2
            for ax in range(3):
                for sg in (-1, 1):
                    if ax == 2 and sg == -1:
                        continue                         # bottom face sits on the desk
                    u, v = [k for k in range(3) if k != ax]
                    nu, nv = max(1, int(round(2 * h[u] / SPACING))), max(1, int(round(2 * h[v] / SPACING)))
                    for a in (np.arange(nu) + 0.5) / nu * 2 - 1:
                        for b in (np.arange(nv) + 0.5) / nv * 2 - 1:
                            q = np.zeros(3); q[ax] = sg * h[ax]; q[u] = a * h[u]; q[v] = b * h[v]
                            n = np.zeros(3); n[ax] = sg
                            pts.append(q); nrm.append(n)
        elif p["shape"] == "sphere":
            r = s[0]; k = max(20, int(4 * np.pi * r * r / SPACING ** 2))
            i = np.arange(k) + 0.5; phi = np.arccos(1 - 2 * i / k); th = np.pi * (1 + 5 ** 0.5) * i
            n = np.column_stack([np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)])
            pts, nrm = list(n * r), list(n)
        else:
            r, hh = s[0], s[1] / 2
            na = max(12, int(2 * np.pi * r / SPACING)); nz = max(2, int(2 * hh / SPACING))
            for a in np.arange(na) / na * 2 * np.pi:
                for z in (np.arange(nz) + 0.5) / nz * 2 - 1:
                    pts.append([r * np.cos(a), r * np.sin(a), z * hh]); nrm.append([np.cos(a), np.sin(a), 0])
            for rr in np.arange(0.5, r / SPACING) * SPACING:
                for a in np.arange(max(6, int(2 * np.pi * rr / SPACING))) / max(6, int(2 * np.pi * rr / SPACING)) * 2 * np.pi:
                    pts.append([rr * np.cos(a), rr * np.sin(a), hh]); nrm.append([0, 0, 1])
        pts = np.asarray(pts) @ Rz.T + c; nrm = np.asarray(nrm) @ Rz.T
        # drop points that are inside / under another object (box floor under the items, block face under the sticker)
        keep = pts[:, 2] > DESK + 1e-4
        probe = pts + 0.0015 * nrm
        for o in props:
            if o is not p:
                keep &= ~inside(probe, o)
        for q, n in zip(pts[keep], nrm[keep]):
            P.append(q); N.append(n); R.append(region(p, q, n))
    (x0, x1), (y0, y1) = DESK_AREA
    gx, gy = np.meshgrid(np.arange(x0, x1, DESK_STEP), np.arange(y0, y1, DESK_STEP), indexing="ij")
    d = np.column_stack([gx.ravel(), gy.ravel(), np.full(gx.size, DESK)])
    keep = np.ones(len(d), bool)
    for o in props:
        keep &= ~inside(d + [0, 0, 0.002], o, pad=0.002)
    for q in d[keep]:
        P.append(q); N.append([0, 0, 1]); R.append(0)
    return np.array(P), np.array(N), np.array(R, np.uint8), regions


# ---------------------------------------------------------------------------------------------- visibility
class Scene:
    """Ray tests against the desk, the objects, the tower and the setup's own moving parts (`own` geom group), except
    the body that carries the camera (`excl`: the camera sits inside its housing)."""

    def __init__(self, m, P, N, own=None, excl=None):
        self.m, self.d = m, mujoco.MjData(m)
        self.P, self.N = P, N
        self.ty = np.tan(np.radians(VFOV) / 2)
        self.tx = self.ty * ASPECT
        self.gg = np.zeros(6, np.uint8); self.gg[GROUP] = self.gg[G_WORLD] = 1
        if own is not None:
            self.gg[own] = 1
        self.excl = m.body(excl).id if excl else -1
        self.robot = self.gg.copy(); self.robot[GROUP] = 0          # only the robot's own parts
        u, w = np.meshgrid(np.linspace(-0.9, 0.9, 5), np.linspace(-0.9, 0.9, 4))
        self.grid = np.column_stack([u.ravel() * self.tx, w.ravel() * self.ty, -np.ones(u.size)])
        self.grid /= np.linalg.norm(self.grid, axis=1, keepdims=True)
        mujoco.mj_forward(m, self.d)

    def own_body_in_image(self, c, R):
        """Share of a 5 x 4 ray grid over the image that hits the robot itself within FAR."""
        dirs = (self.grid @ R.T).ravel()
        gid = np.zeros(len(self.grid), np.int32); hit = np.zeros(len(self.grid))
        mujoco.mj_multiRay(self.m, self.d, c, dirs, self.robot, 1, self.excl, gid, hit, None, len(self.grid), FAR)
        return float(((gid >= 0) & (hit < FAR)).mean())

    def ray_clear(self, c, a):
        v = a - c; dd = np.linalg.norm(v)
        gid = np.zeros(1, np.int32)
        h = mujoco.mj_ray(self.m, self.d, c, v / dd, self.gg, 1, self.excl, gid)
        return gid[0] < 0 or h > dd - 0.004

    def pose(self, qadr, q, cam):
        self.d.qpos[qadr] = q
        mujoco.mj_kinematics(self.m, self.d)
        mujoco.mj_camlight(self.m, self.d)
        return self.d.cam_xpos[cam].copy(), self.d.cam_xmat[cam].reshape(3, 3).copy()

    def visible(self, c, R):
        v = self.P - c
        dist = np.linalg.norm(v, axis=1)
        pc = v @ R
        z = -pc[:, 2]
        ok = (z > 1e-6) & (dist > NEAR) & (dist < FAR)
        ok &= (np.abs(pc[:, 0]) < self.tx * z) & (np.abs(pc[:, 1]) < self.ty * z)
        ok &= np.einsum("ij,ij->i", self.N, -v) > FACING * dist
        idx = np.flatnonzero(ok)
        if len(idx) == 0:
            return idx
        dirs = (v[idx] / dist[idx, None]).ravel()
        gid = np.zeros(len(idx), np.int32); hit = np.zeros(len(idx))
        mujoco.mj_multiRay(self.m, self.d, c, dirs, self.gg, 1, self.excl, gid, hit, None, len(idx), FAR + 0.1)
        return idx[(gid < 0) | (hit > dist[idx] - 0.004)]


def view_tasks(rng):
    """80 requested views: camera at P (in front of the robot, 12-32 cm above the desk), looking at A on the desk."""
    tasks = []
    while len(tasks) < N_TASKS:
        p = np.array([rng.uniform(-0.42, -0.16), rng.uniform(-0.18, 0.18), DESK + rng.uniform(0.12, 0.32)])
        a = np.array([p[0] + rng.uniform(-0.2, 0.2), p[1] + rng.uniform(-0.2, 0.2), DESK])
        if np.linalg.norm(a - p) > 0.15:
            tasks.append((p, a))
    return tasks


def solve_view(sc, qadr, cam, lo, hi, p, a, level, rng):
    """Damped least squares from 12 random starts; a solution whose line of sight to A is blocked by the robot itself
    loses. Returns (aim error deg, horizon tilt deg (upside down counts as level), position error cm, sight clear)."""
    def res(q):
        c, R = sc.pose(qadr, q, cam); v = a - c
        r = [3.0 * (-R[:, 2] - v / np.linalg.norm(v)), 3.0 * (c - p)]
        if level:
            r.append([R[2, 0]])
        r.append([10 * max(0.0, DESK + 0.02 - c[2])])
        return np.concatenate(r)

    best = None
    for _ in range(12):
        q = rng.uniform(lo, hi)
        for _ in range(150):
            r0 = res(q); J = np.empty((len(r0), len(q)))
            for k in range(len(q)):
                dq = np.zeros(len(q)); dq[k] = 1e-5; J[:, k] = (res(q + dq) - r0) / 1e-5
            step = np.linalg.solve(J.T @ J + 1e-3 * np.eye(len(q)), -J.T @ r0)
            q = np.clip(q + np.clip(step, -0.2, 0.2), lo, hi)
            if np.linalg.norm(step) < 1e-6:
                break
        e = float(res(q) @ res(q))
        c, _ = sc.pose(qadr, q, cam)
        if not sc.ray_clear(c, a):
            e += 1e3
        if best is None or e < best[0]:
            best = (e, q)
    c, R = sc.pose(qadr, best[1], cam); v = a - c; v /= np.linalg.norm(v)
    return (np.degrees(np.arccos(np.clip(-R[:, 2] @ v, -1, 1))), abs(np.degrees(np.arcsin(np.clip(R[2, 0], -1, 1)))),
            100 * np.linalg.norm(c - p), sc.ray_clear(c, a))


def body_points(m, d, bodies, cam):
    pts = [d.xpos[b] for b in bodies[1:]]   # the J1 body sits on top of the tower
    mids = [(pts[i] + pts[i + 1]) / 2 for i in range(len(pts) - 1)]
    return np.array(pts + mids + [d.cam_xpos[cam]])


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    props = scene_props()
    head = urdf_head()
    m, so_cam_pos, so_cam_quat = build_model(props, head)
    models = {None: m, "x": build_model(props, head, "x")[0], "long": build_model(props, head, "long")[0]}
    extend = float(m.body_pos[m.body("stock_wrist").id][2] - m.body_pos[m.body("neck_wrist").id][2])
    print(f"LeRobotNeck-02: J3-J4 link {100 * m.body_pos[m.body('neck_wrist').id][2]:.1f} -> "
          f"{100 * models['long'].body_pos[models['long'].body('neck_wrist').id][2]:.1f} cm along the arm (+{100 * extend:.1f})")
    P, N, REG, regions = surface_points(props)
    print(f"{len(P)} surface points: " + ", ".join(f"{r} {int((REG == k).sum())}" for k, r in enumerate(regions)))
    sc = Scene(m, P, N)

    jadr = lambda names: np.array([m.joint(n).qposadr[0] for n in names])
    jrng = lambda names: np.array([m.joint(n).range for n in names])
    # keep-out boxes: every object (1 cm margin) and the tower
    keep = []
    for p in props:
        lo, hi = np.array(p["pos"]), np.array(p["pos"])
        r = (np.array(p["size"]) / 2 if p["shape"] == "box" else np.array([p["size"][0]] * 3) if p["shape"] == "sphere"
             else np.array([p["size"][0], p["size"][0], p["size"][1] / 2]))
        r = np.full(3, np.linalg.norm(r[:2])) * [1, 1, 0] + r * [0, 0, 1]   # yaw-safe in x / y
        keep.append((lo - r - 0.01, hi + r + 0.01))
    tw = next(i for i in range(m.ngeom) if m.geom_dataid[i] >= 0 and m.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH
              and m.mesh(m.geom_dataid[i]).name == "base_tower")
    c = sc.d.geom_xpos[tw] + m.geom_aabb[tw][:3]; h = m.geom_aabb[tw][3:]
    keep.append((c - h - 0.01, c + h + 0.01))

    def clear(pts):
        if (pts[:, 2] < DESK + 0.015).any():
            return False
        for lo, hi in keep:
            if ((pts > lo) & (pts < hi)).all(axis=1).any():
                return False
        return True

    rng = np.random.default_rng(0)
    pan_j, tilt_j = jadr(["head_pan"]), jadr(["head_tilt"])
    cam_head = m.camera("head_cam").id
    # fixed camera: pan 0, the tilt that looks best at the middle of the objects
    centre = np.mean([p["pos"] for p in props], axis=0)
    centre[2] = DESK
    best = None
    for t in np.linspace(*head["tilt"]["range"], 400):
        cc, R = sc.pose(np.r_[pan_j, tilt_j], [0.0, t], cam_head)
        dd = centre - cc
        s = -R[:, 2] @ dd / np.linalg.norm(dd)
        if best is None or s > best[0]:
            best = (s, t)
    fixed_tilt = best[1]
    print(f"fixed camera tilt {np.degrees(fixed_tilt):.1f} deg (looks at the objects' centre)")

    pr, tr = head["pan"]["range"], head["tilt"]["range"]
    setups = [
        {"id": "fixed", "dof": 0, "color": "#8b5cf6", "joints": ["head_pan", "head_tilt"], "cam": "head_cam",
         "samples": [[0.0, fixed_tilt]],
         "name": {"en": "Fixed camera", "zh": "固定相机"},
         "short": {"en": "Ego-centric camera, no motion", "zh": "以自我为中心的固定相机，不能动"}},
        {"id": "head2", "dof": 2, "color": "#3b82f6", "joints": ["head_pan", "head_tilt"], "cam": "head_cam",
         "samples": [[a, b] for a in np.arange(pr[0], pr[1] + 1e-9, np.radians(2)) for b in np.arange(tr[0], tr[1] + 1e-9, np.radians(2))],
         "name": {"en": "XLeRobot 2-axis head", "zh": "XLeRobot 两轴头"},
         "short": {"en": "Pan + tilt, the camera turns in place", "zh": "左右转 + 上下点头，相机原地转"}},
        {"id": "so101", "dof": 5, "color": "#f59e0b", "joints": STOCK_JOINTS, "cam": "so101_cam", "bodies": STOCK_BODIES,
         "own": G_STOCK,
         "name": {"en": "SO101 5-axis + wrist camera", "zh": "SO101 五轴 + 腕部相机"},
         "short": {"en": "A stock arm with a camera on the jaw", "zh": "标准机械臂，相机装在夹爪上"}},
        {"id": "so101x", "dof": 6, "color": "#dc2626", "model": "x", "joints": STOCK_JOINTS + ["stock_gripper"],
         "cam": "so101x_cam", "bodies": STOCK_BODIES, "own": G_STOCK, "excl": "stock_moving_jaw",
         "name": {"en": "SO101-X", "zh": "SO101-X"},
         "short": {"en": "No new parts: the D435i holder replaces the moving jaw, the gripper motor tilts the camera",
                   "zh": "不加新零件：相机支架换掉活动夹爪，夹爪电机变成相机俯仰"}},
        {"id": "neck", "dof": 6, "color": "#16a34a", "joints": NECK, "cam": "neck_color", "bodies": NECK_BODIES,
         "own": G_NECK, "excl": "neck_cam_holder",
         "name": {"en": "LeRobotNeck 6-axis", "zh": "LeRobotNeck 六轴"},
         "short": {"en": "Moves the D435i: where it looks from and where it looks", "zh": "带着 D435i 走：换位置，也换朝向"}},
        {"id": "neck02", "dof": 6, "color": "#0891b2", "model": "long", "joints": NECK, "cam": "neck_color",
         "bodies": NECK_BODIES, "own": G_NECK, "excl": "neck_cam_holder",
         "name": {"en": "LeRobotNeck-02 (concept)", "zh": "LeRobotNeck-02（设想）"},
         "short": {"en": f"J3-J4 link {100 * extend:.1f} cm longer, as long as the stock lower arm",
                   "zh": f"J3–J4 连杆加长 {100 * extend:.1f} cm，和标准小臂一样长"}},
    ]
    tasks = view_tasks(np.random.default_rng(3))
    # bins for "from how many different directions is a point seen"
    k = 32; i = np.arange(k) + 0.5
    phi, th = np.arccos(1 - 2 * i / k), np.pi * (1 + 5 ** 0.5) * i
    BINS = np.column_stack([np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)])

    out_setups = []
    for S in setups:
        mm = models[S.get("model")]
        sc = Scene(mm, P, N, S.get("own"), S.get("excl"))
        qadr, cam = np.array([mm.joint(n).qposadr[0] for n in S["joints"]]), mm.camera(S["cam"]).id
        lo, hi = np.array([mm.joint(n).range for n in S["joints"]]).T
        if "samples" in S:
            samples = np.array(S["samples"])
        else:
            samples, tried = [], 0
            bodies = [mm.body(b).id for b in S["bodies"]]
            while len(samples) < N_ARM:
                q = rng.uniform(lo, hi); tried += 1
                sc.pose(qadr, q, cam)
                if clear(body_points(mm, sc.d, bodies, cam)):
                    samples.append(q)
            samples = np.array(samples)
            print(f"{S['id']}: {N_ARM} clear poses from {tried} random samples")
        n = len(samples)
        bits = np.zeros((n, (len(P) + 7) // 8), np.uint8)
        dirmask = np.zeros(len(P), np.uint64)
        cams, upright = [], np.zeros(n, bool)
        half = None; own_img = []
        for j, q in enumerate(samples):
            c, R = sc.pose(qadr, q, cam)
            cams.append((c, R))
            if j % 10 == 0:
                own_img.append(sc.own_body_in_image(c, R))
            # upright image: horizon within +-UPRIGHT_DEG and image up pointing up (camera x / y axes vs world z)
            upright[j] = abs(R[2, 0]) < np.sin(np.radians(UPRIGHT_DEG)) and R[2, 1] > 0
            vis = sc.visible(c, R)
            row = np.zeros(len(P), bool); row[vis] = True
            bits[j] = np.packbits(row)
            if len(vis):
                u = c - P[vis]; u /= np.linalg.norm(u, axis=1, keepdims=True)
                b = np.argmax(u @ BINS.T, axis=1).astype(np.uint64)
                np.bitwise_or.at(dirmask, vis, np.left_shift(np.uint64(1), b))
            if j == n // 2 - 1:
                half = np.unpackbits(np.bitwise_or.reduce(bits[: j + 1], axis=0))[: len(P)].astype(bool)
        seen = np.unpackbits(np.bitwise_or.reduce(bits, axis=0))[: len(P)].astype(bool)
        if half is None:
            half = seen
        seen_up = (np.unpackbits(np.bitwise_or.reduce(bits[upright], axis=0))[: len(P)].astype(bool) if upright.any()
                   else np.zeros(len(P), bool))
        ndirs = np.array([bin(int(x)).count("1") for x in dirmask], np.uint8)
        # greedy: the views that add the most new points
        POP = np.array([bin(x).count("1") for x in range(256)], np.uint16)
        covered = np.zeros(bits.shape[1], np.uint8); chosen = []
        for _ in range(min(N_GREEDY, n)):
            gain = POP[bits & ~covered].sum(axis=1)
            j = int(np.argmax(gain))
            if gain[j] == 0:
                break
            chosen.append(j); covered |= bits[j]
        # order the chosen views as a short path through joint space (start nearest the first sample)
        order, rest = [chosen[0]], chosen[1:]
        while rest:
            j = min(rest, key=lambda r: np.linalg.norm(samples[r] - samples[order[-1]])); order.append(j); rest.remove(j)
        first = np.full(len(P), -1, np.int8)
        first[seen] = len(order)
        for kk in reversed(range(len(order))):
            row = np.unpackbits(bits[order[kk]])[: len(P)].astype(bool)
            first[row] = kk
        stats = {r: round(100 * float(seen[REG == k].mean()), 1) for k, r in enumerate(regions)}
        stats["all"] = round(100 * float(seen.mean()), 1)
        stats_up = {r: round(100 * float(seen_up[REG == k].mean()), 1) for k, r in enumerate(regions)}
        stats_up["all"] = round(100 * float(seen_up.mean()), 1)
        greedy_pct = round(100 * float((first >= 0)[first < len(order)].sum() / len(P)), 1)
        print(f"{S['id']:<6} poses {n:6d}  seen {stats['all']:5.1f}% (half the poses {100 * half.mean():5.1f}%)  "
              f"{len(order)} best views {greedy_pct:5.1f}%  mean directions per seen point "
              f"{ndirs[seen].mean() if seen.any() else 0:.1f}  " + " ".join(f"{r} {v}" for r, v in stats.items())
              + f"\n       upright poses {int(upright.sum())} ({100 * upright.mean():.0f}%), upright images see {stats_up['all']}%: "
              + " ".join(f"{r} {v}" for r, v in stats_up.items()))
        # requested views (the same 80 for every setup; IK seeded per setup so the order does not matter)
        trng = np.random.default_rng(7)
        reached = level = blocked = 0
        for p, a in tasks:
            if n == 1:   # cannot move: just check the one pose
                c, R = cams[0]; v = (a - c) / np.linalg.norm(a - c)
                r = (np.degrees(np.arccos(np.clip(-R[:, 2] @ v, -1, 1))), 0.0, 100 * np.linalg.norm(c - p), sc.ray_clear(c, a))
                rl = r
            else:
                r = solve_view(sc, qadr, cam, lo, hi, p, a, False, trng)
                rl = solve_view(sc, qadr, cam, lo, hi, p, a, True, trng)
            if r[0] < AIM_TOL and r[2] < POS_TOL:
                reached += r[3]; blocked += not r[3]
            level += rl[0] < AIM_TOL and rl[2] < POS_TOL and rl[1] < LEVEL_TOL and rl[3]
        own_img = np.array(own_img)
        print(f"       requested views: {reached}/{len(tasks)} (+{blocked} with its own body in the line of sight), "
              f"with a level horizon {level}/{len(tasks)}; own body in the image: mean {100 * own_img.mean():.1f}%, "
              f"poses with > 25% covered {100 * (own_img > 0.25).mean():.1f}%")
        views = []
        for j in order:
            c, R = cams[j]
            qq = np.zeros(4); mujoco.mju_mat2Quat(qq, R.ravel())
            views.append({"q": dict(zip(S["joints"], np.round(samples[j], 4).tolist())),
                          "cam": {"pos": np.round(c, 4).tolist(), "quat": np.round(qq, 5).tolist()}})
        out_setups.append({k2: S[k2] for k2 in ("id", "dof", "color", "name", "short")} | {
            "poses": int(n), "stats": stats, "stats_upright": stats_up, "upright_poses": int(upright.sum()), "stats_half": round(100 * float(half.mean()), 1),
            "dirs_mean": round(float(ndirs[seen].mean()) if seen.any() else 0.0, 2),
            "own_img": round(100 * float(own_img.mean()), 1), "own_img25": round(100 * float((own_img > 0.25).mean()), 1),
            "tasks": {"n": len(tasks), "reached": int(reached), "level": int(level), "blocked": int(blocked)},
            "views": views, "first": b64(first), "ndirs": b64(ndirs), "upright": b64(np.packbits(seen_up))})

    # only-by stats: what the neck sees that each other setup does not, and the other way round
    seen_by = {s["id"]: np.frombuffer(base64.b64decode(s["first"]), np.int8) >= 0 for s in out_setups}
    diff = {f"{a}_not_{b}": round(100 * float((seen_by[a] & ~seen_by[b]).mean()), 1)
            for a in seen_by for b in seen_by if a != b}
    print("seen by A but not B (% of all points):", {k: v for k, v in diff.items() if k.startswith("neck") or k.endswith("neck")})

    data = {
        "about": "generated by projects/web_sim/scripts/make_coverage.py - do not edit",
        "camera": {"vfov": VFOV, "aspect": ASPECT, "near": NEAR, "far": FAR, "facing_deg": 80},
        "points": {"n": int(len(P)), "pos_mm": b64(np.round(P * 1000).astype(np.int16)),
                   "normal": b64(np.round(N * 127).astype(np.int8)), "region": b64(REG)},
        "regions": regions, "setups": out_setups, "diff": diff,
        "head": {"pan": head["pan"], "tilt": head["tilt"], "camera_link_pos": head["camera_link_pos"],
                 "meshes": head["meshes"], "fixed_tilt": round(float(fixed_tilt), 4)},
        "so101_cam": {"body": "stock_fixed_jaw", "pos": so_cam_pos, "quat": so_cam_quat},
        # how the page draws the two variants on the LeRobotNeck model (it has no extra bodies for them)
        "so101x": {"jaw": "stock_moving_jaw", "holder": "neck_cam_holder", "hide": "Moving_Jaw"},
        "neck02": {"link": "neck_link34", "extend": [0.0, 0.0, round(extend, 5)], "spacer": SPACER,
                   "moved": ["neck_wrist", "neck_cam_tilt_housing", "neck_cam_holder"], "motor": "Lower_Arm_Motor"},
        "tasks": {"n": N_TASKS, "aim_deg": AIM_TOL, "pos_cm": POS_TOL, "level_deg": LEVEL_TOL},
    }
    (OUT / "coverage.json").write_text(json.dumps(data, ensure_ascii=False, separators=(",", ":")) + "\n", encoding="utf-8")
    for mh in head["meshes"]:
        src = ROOT / "hardware/upstream/xlerobot/meshes" / mh["file"]
        shutil.copyfile(src, OUT / mh["file"])
    print("wrote", sorted(p.name for p in OUT.iterdir()), f"{(OUT / 'coverage.json').stat().st_size / 1024:.0f} KB")


if __name__ == "__main__":
    main()
