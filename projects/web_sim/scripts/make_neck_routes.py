"""Write the LeRobotNeck demo routes and desk scene (projects/web_sim/web/public/routes_neck/*.json, labels in English
and Chinese).

    .venv/Scripts/python.exe projects/web_sim/scripts/make_neck_routes.py

The routes are data for the showcase website; this script is their source (edit the poses here, then re-run).
Angles in degrees, partial waypoints keep the other joints (format: web/src/routes.js).

The two active-perception demos (circle the green block, look inside the box) are not hand-written: their camera
viewpoints are solved with inverse kinematics on the exported neck model (public/model_neck, made by build_all.py),
and the script checks with ray tests that the things in the box are hidden from the ready pose and seen from the
solved viewpoints. The desk objects are written to routes_neck/scene.json (the website draws them; they are not part
of the MuJoCo model, the web neck is kinematic only).
"""
import json
from pathlib import Path

import mujoco
import numpy as np

WEB = Path(__file__).resolve().parents[1] / "web/public"
OUT = WEB / "routes_neck"
MODEL = WEB / "model_neck/scene.xml"
NECK = [f"neck_j{i}" for i in range(1, 7)]
READY = {"neck_j1": 0, "neck_j2": 90, "neck_j3": 150, "neck_j4": 0, "neck_j5": 0, "neck_j6": -25}
REST = {"neck_j1": 6.8, "neck_j2": 186.9, "neck_j3": 180.7, "neck_j4": 0.2, "neck_j5": 0, "neck_j6": 10.7}


def L(en, zh):
    return {"en": en, "zh": zh}


def wp(en, zh, q, **kw):
    return {"label": L(en, zh), "q": q, **kw}


ROUTES = {
    "wake_up": {
        "name": L("Wake up", "起身唤醒"),
        "description": L("From the folded rest pose to standing: lift, stretch, shake it off.",
                         "从折叠的休息位站起来：抬头、伸展、甩一甩。"),
        "from": "home", "speed_dps": 40, "dwell_s": 0.4, "waypoints": [
            wp("Lift the head", "抬起头", {"neck_j2": 150, "neck_j3": 178, "neck_j6": -5}),
            wp("Stand up", "站起来", {"neck_j2": 100, "neck_j3": 160, "neck_j6": -15}),
            wp("Stretch", "伸个懒腰", {"neck_j2": 80, "neck_j3": 175, "neck_j6": -50}, dwell_s=0.8),
            wp("Shake", "甩一甩", {"neck_j5": 18, "neck_j4": 10}, duration_s=0.45, dwell_s=0),
            wp("Shake", "甩一甩", {"neck_j5": -18, "neck_j4": -10}, duration_s=0.5, dwell_s=0),
            wp("Shake", "甩一甩", {"neck_j5": 0, "neck_j4": 0}, duration_s=0.45),
            wp("Ready", "准备好了", READY, dwell_s=1.0)]},
    "look_around": {
        "name": L("Look around", "环顾四周"),
        "description": L("Scan the room with J1 and tilt the D435i with J6. Watch the camera inset.",
                         "用 J1 扫视、J6 俯仰 D435i，看看相机小窗里的画面。"),
        "from": "home", "speed_dps": 45, "dwell_s": 0.7, "waypoints": [
            wp("Ready", "准备姿势", READY),
            wp("Look left", "看左边", {"neck_j1": -55, "neck_j6": -20}),
            wp("Look up", "抬头", {"neck_j6": -55}),
            wp("Look right", "看右边", {"neck_j1": 50, "neck_j6": -30}),
            wp("Look down", "低头", {"neck_j6": 10}),
            wp("Back to centre", "回到正前方", {"neck_j1": 0, "neck_j6": -25})]},
    "nod_shake": {
        "name": L("Yes and no", "点头和摇头"),
        "description": L("A quick nod (J6) and a head shake (J1).", "快速点头（J6），再摇头（J1）。"),
        "from": "home", "speed_dps": 90, "dwell_s": 0, "waypoints": [
            wp("Ready", "准备姿势", READY, dwell_s=0.5),
            wp("Nod", "点头", {"neck_j6": 5}, duration_s=0.4),
            wp("Nod", "点头", {"neck_j6": -40}, duration_s=0.45),
            wp("Nod", "点头", {"neck_j6": 5}, duration_s=0.45),
            wp("Nod", "点头", {"neck_j6": -25}, duration_s=0.4, dwell_s=0.7),
            wp("Shake", "摇头", {"neck_j1": -25}, duration_s=0.4),
            wp("Shake", "摇头", {"neck_j1": 25}, duration_s=0.55),
            wp("Shake", "摇头", {"neck_j1": -20}, duration_s=0.55),
            wp("Shake", "摇头", {"neck_j1": 0}, duration_s=0.45, dwell_s=0.6)]},
    "curious_tilt": {
        "name": L("Curious tilt", "好奇地歪头"),
        "description": L("Side tilt with J4, the motion the new J3-J4 link adds, plus a J5 roll.",
                         "用 J4 侧摆（新 J3–J4 连杆带来的动作），再加上 J5 滚转。"),
        "from": "home", "speed_dps": 35, "dwell_s": 1.0, "waypoints": [
            wp("Ready", "准备姿势", READY, dwell_s=0.4),
            wp("Tilt left", "向左歪头", {"neck_j4": 32, "neck_j5": -12, "neck_j6": -15}),
            wp("Tilt right", "向右歪头", {"neck_j4": -32, "neck_j5": 12}),
            wp("Roll", "滚转", {"neck_j4": 0, "neck_j5": -35}),
            wp("Roll back", "滚回来", {"neck_j5": 30}),
            wp("Upright", "回正", {"neck_j5": 0, "neck_j6": -25})]},
    "peek_desk": {
        "name": L("Peek at the desk", "探身看桌面"),
        "description": L("Lean forward with J2/J3 and look down at the desk in front of the tower.",
                         "用 J2/J3 前倾，低头看竖塔前面的桌面。"),
        "from": "home", "speed_dps": 35, "dwell_s": 0.9, "waypoints": [
            wp("Ready", "准备姿势", READY, dwell_s=0.3),
            wp("Lean forward", "向前探身", {"neck_j2": 55, "neck_j3": 115, "neck_j6": 15}),
            wp("Look left", "往左看", {"neck_j1": -25}),
            wp("Look right", "往右看", {"neck_j1": 25}),
            wp("Back up", "收回来", {"neck_j1": 0, "neck_j2": 90, "neck_j3": 150, "neck_j6": -25})]},
}

# ---------------------------------------------------------------- desk scene (MuJoCo world: metres, robot faces -x)
DESK = 0.1059                       # desk plane z (hardware/variants/lerobotneck.yaml sim.desk_z)
BLOCK = {"c": (-0.31, 0.09), "s": (0.04, 0.04, 0.075)}             # green block, a sticker on its outer (+y) side
BOX = {"c": (-0.35, -0.10), "s": (0.10, 0.12, 0.11), "t": 0.006}   # open-top box: outer size, wall thickness
ITEMS = [  # inside the box, close to the near wall (hidden behind it from the ready pose)
    {"id": "ball", "shape": "sphere", "pos": (-0.324, -0.128), "size": (0.016,), "color": "#e5484d"},
    {"id": "cube", "shape": "box", "pos": (-0.322, -0.097), "size": (0.024, 0.024, 0.024), "color": "#2a6df4", "yaw": 0.4},
    {"id": "can", "shape": "cylinder", "pos": (-0.32, -0.066), "size": (0.012, 0.032), "color": "#ff6a13"},
]


def scene_props():
    """Every desk object as a primitive {shape, pos (centre), size, color, yaw}: box size = full extents,
    sphere = (r,), cylinder = (r, h) with a vertical axis."""
    P = []
    (bx, by), (sx, sy, sz) = BLOCK["c"], BLOCK["s"]
    P.append({"id": "block", "shape": "box", "pos": [bx, by, DESK + sz / 2], "size": [sx, sy, sz], "color": "#1a9e6b"})
    P.append({"id": "sticker", "shape": "box", "pos": [bx, by + sy / 2 + 0.001, DESK + sz * 0.55],
              "size": [0.026, 0.002, 0.026], "color": "#f2c230"})
    (cx, cy), (sx, sy, sz), t = BOX["c"], BOX["s"], BOX["t"]
    brown = "#c49a6c"
    P.append({"id": "box_floor", "shape": "box", "pos": [cx, cy, DESK + 0.002], "size": [sx, sy, 0.004], "color": brown})
    for k, (dx, dy, wx, wy) in enumerate([(sx / 2 - t / 2, 0, t, sy), (-sx / 2 + t / 2, 0, t, sy),
                                          (0, sy / 2 - t / 2, sx, t), (0, -sy / 2 + t / 2, sx, t)]):
        P.append({"id": f"box_wall{k}", "shape": "box", "pos": [cx + dx, cy + dy, DESK + sz / 2], "size": [wx, wy, sz],
                  "color": brown})
    for it in ITEMS:
        h = it["size"][0] * 2 if it["shape"] == "sphere" else it["size"][-1]
        P.append({**it, "pos": [*it["pos"], DESK + 0.004 + h / 2], "size": list(it["size"])})
    # two more things to look at in the other demos
    P.append({"id": "flat", "shape": "box", "pos": [-0.30, -0.2, DESK + 0.01], "size": [0.07, 0.045, 0.02], "color": "#2a6df4", "yaw": 0.5})
    P.append({"id": "cube2", "shape": "box", "pos": [-0.42, 0.26, DESK + 0.025], "size": [0.05, 0.05, 0.05], "color": "#ff6a13", "yaw": -0.6})
    return P


def aabb(p):
    """Axis-aligned box of a prop (yaw ignored: used for occlusion / clearance checks only)."""
    c = np.array(p["pos"])
    if p["shape"] == "box":
        h = np.array(p["size"]) / 2
    elif p["shape"] == "sphere":
        h = np.full(3, p["size"][0])
    else:
        h = np.array([p["size"][0], p["size"][0], p["size"][1] / 2])
    return c - h, c + h


# ---------------------------------------------------------------- kinematics on the exported web model
class Neck:
    HEAD = ["neck_wrist", "neck_cam_tilt_housing", "neck_cam_holder"]

    def __init__(self):
        self.m = mujoco.MjModel.from_xml_path(str(MODEL))
        self.d = mujoco.MjData(self.m)
        self.adr = [self.m.joint(n).qposadr[0] for n in NECK]
        self.lo = np.array([self.m.joint(n).range[0] for n in NECK])
        self.hi = np.array([self.m.joint(n).range[1] for n in NECK])
        self.cam = self.m.camera("neck_color").id
        self.fovy = np.radians(self.m.cam_fovy[self.cam])
        self.head = [self.m.body(b).id for b in self.HEAD]

    def fk(self, q):
        self.d.qpos[:] = 0
        self.d.qpos[self.adr] = q
        mujoco.mj_kinematics(self.m, self.d)
        mujoco.mj_camlight(self.m, self.d)
        R = self.d.cam_xmat[self.cam].reshape(3, 3).copy()
        pts = [self.d.xpos[b].copy() for b in self.head] + [self.d.cam_xpos[self.cam].copy()]
        return self.d.cam_xpos[self.cam].copy(), R, pts

    def sees(self, q, p, occluders=()):
        """Point p inside the 4:3 camera image and not hidden behind any occluder AABB."""
        c, R, _ = self.fk(q)
        pc = R.T @ (np.asarray(p) - c)
        if pc[2] >= 0:
            return False
        ty = np.tan(self.fovy / 2)
        if abs(pc[1] / -pc[2]) > ty * 0.92 or abs(pc[0] / -pc[2]) > ty * 4 / 3 * 0.92:
            return False
        d = np.asarray(p) - c
        for lo, hi in occluders:   # slab test on the segment camera -> p
            with np.errstate(divide="ignore", invalid="ignore"):
                t1, t2 = (lo - c) / d, (hi - c) / d
            tn, tf = np.nanmax(np.minimum(t1, t2)), np.nanmin(np.maximum(t1, t2))
            if tn < tf and tf > 0 and tn < 0.999:
                return False
        return True

    def solve(self, aim, seed, pos=None, w_pos=3.0, keep_out=()):
        """Joint angles (rad) that point the D435i at `aim` (and bring it near `pos`), horizon level, near `seed`,
        with the head 6 cm above the desk and 2.5 cm clear of the `keep_out` boxes (damped least squares)."""
        aim = np.asarray(aim, float)
        seed = np.asarray(seed, float)

        def res(q):
            c, R, pts = self.fk(q)
            d = aim - c
            r = [2.0 * (-R[:, 2] - d / np.linalg.norm(d)), [0.8 * R[:, 0] @ np.array([0, 0, 1.0])], 0.03 * (q - seed)]
            if pos is not None:
                r.append(w_pos * (c - np.asarray(pos)))
            pen = []
            for p in pts:
                pen.append(max(0.0, DESK + 0.06 - p[2]))
                for lo, hi in keep_out:
                    gap = np.max(np.maximum(lo - 0.025 - p, p - hi - 0.025))
                    pen.append(max(0.0, -gap))
            r.append(20.0 * np.array(pen))
            return np.concatenate([np.ravel(x) for x in r])

        def run(q):
            for _ in range(200):
                r0 = res(q)
                Jm = np.empty((len(r0), 6))
                for k in range(6):
                    dq = np.zeros(6); dq[k] = 1e-5
                    Jm[:, k] = (res(q + dq) - r0) / 1e-5
                step = np.linalg.solve(Jm.T @ Jm + 1e-3 * np.eye(6), -Jm.T @ r0)
                q = np.clip(q + np.clip(step, -0.15, 0.15), self.lo, self.hi)
                if np.linalg.norm(step) < 1e-6:
                    break
            return q, float(res(q) @ res(q))

        # a few starts (the seed, J1 turned either way, fixed random ones) against local minima; keep the best
        rng = np.random.default_rng(1)
        starts = [seed, seed + [0.6, 0, 0, 0, 0, 0], seed - [0.6, 0, 0, 0, 0, 0]] + \
                 [np.clip(seed + rng.normal(0, 0.5, 6), self.lo, self.hi) for _ in range(6)]
        return min((run(np.clip(s, self.lo, self.hi)) for s in starts), key=lambda x: x[1])[0]


def qdeg(q):
    return {n: round(float(np.degrees(v)), 1) for n, v in zip(NECK, q)}


def perception_routes():
    nk = Neck()
    props = scene_props()
    by = {p["id"]: p for p in props}
    occ = [aabb(p) for p in props if p["id"].startswith(("box_wall", "block"))]
    ready = np.radians([READY[n] for n in NECK])
    out, report, problems = {}, [], []

    # 1) circle the green block: views from the front, its outer side (the sticker), above and the inner side
    (bx, byy), (_, _, bz) = BLOCK["c"], BLOCK["s"]
    centre, top = np.array([bx, byy, DESK + bz / 2]), np.array([bx, byy, DESK + bz])
    sticker = np.array(by["sticker"]["pos"]) + [0, 0.0015, 0]
    block_box = aabb(by["block"])
    views = [("Front", "正面", 0, 0.13, 0.10), ("Outer side: a sticker!", "外侧：发现一张贴纸", 80, 0.13, 0.06),
             ("From above", "从上面看", None, 0, 0.16), ("Inner side", "内侧", -75, 0.12, 0.08)]
    wps, q = [wp("Ready", "准备姿势", READY, dwell_s=0.3)], ready
    for en, zh, az, r, h in views:
        p = top + ([0.015, 0, h] if az is None else [r * np.cos(np.radians(az)), r * np.sin(np.radians(az)), h])
        q = nk.solve(centre, q, pos=p, keep_out=[block_box])
        c, _, _ = nk.fk(q)
        report.append(f"block {en:<24} cam {np.round(c, 3)} (wanted {np.round(p, 3)}) sees block {nk.sees(q, centre)} "
                      f"sticker {nk.sees(q, sticker, [block_box])}")
        wps.append(wp(en, zh, qdeg(q), dwell_s=1.2))
    wps.append(wp("Ready", "准备姿势", READY))
    out["inspect_block"] = {
        "name": L("Circle the green block", "环绕观察绿色方块"),
        "description": L("Active perception: the neck moves the D435i around the block to see its sides and top, and "
                         "finds the sticker on the side it could not see.",
                         "主动感知：脖子带着 D435i 绕方块转，从侧面和上面看它，找到原来看不到的那一面上的贴纸。"),
        "from": "home", "speed_dps": 35, "dwell_s": 1.0, "camera_inset": "big", "waypoints": wps}
    # the sticker faces away from the ready pose
    if nk.sees(ready, sticker, [block_box]):
        problems.append("sticker already visible from the ready pose")

    # 2) look inside the box: the walls hide what is in it; lean over it and scan the items one by one
    (cx, cy), (_, _, sz) = BOX["c"], BOX["s"]
    box_box = (np.array([cx - BOX["s"][0] / 2, cy - BOX["s"][1] / 2, DESK]), np.array([cx + BOX["s"][0] / 2, cy + BOX["s"][1] / 2, DESK + sz]))
    items = {it["id"]: np.array(by[it["id"]]["pos"]) for it in ITEMS}
    hidden = [k for k, p in items.items() if not nk.sees(ready, p, occ)]
    report.append(f"box: hidden from the ready pose: {hidden}")
    if len(hidden) != len(items):
        problems.append("an item in the box is visible from the ready pose")
    q = nk.solve([cx, cy, DESK + sz], ready, pos=None)          # turn to the box first (aim only)
    wps = [wp("Ready", "准备姿势", READY, dwell_s=0.3), wp("Is anything in there?", "里面有东西吗？", qdeg(q), dwell_s=1.0)]
    report.append(f"box: from facing it, sees {[k for k, p in items.items() if nk.sees(q, p, occ)]}")
    mid = np.mean(list(items.values()), axis=0)
    over = np.array([cx + 0.005, mid[1], DESK + sz + 0.165])
    q = nk.solve(mid, q, pos=over, w_pos=8.0, keep_out=[box_box])
    seen = [k for k, p in items.items() if nk.sees(q, p, occ)]
    report.append(f"box: leaning over, cam {np.round(nk.fk(q)[0], 3)}, sees {seen}")
    wps.append(wp("Lean over and look in", "探身往里看", qdeg(q), dwell_s=1.0))
    for k, (en, zh) in {"ball": ("A red ball", "一个红球"), "cube": ("A blue cube", "一个蓝色方块"),
                        "can": ("An orange can", "一个橙色罐子")}.items():
        q = nk.solve(items[k], q, pos=over + [0, (items[k][1] - mid[1]) * 0.5, 0], w_pos=1.5, keep_out=[box_box])
        report.append(f"box: look at {k:<5} sees {[j for j, p in items.items() if nk.sees(q, p, occ)]}")
        if not nk.sees(q, items[k], occ):
            problems.append(f"{k} not visible from its viewpoint")
        wps.append(wp(en, zh, qdeg(q), dwell_s=1.0))
    wps.append(wp("Found 3 things", "找到 3 样东西", qdeg(nk.solve(mid, q, pos=over, keep_out=[box_box])), dwell_s=1.2))
    wps.append(wp("Back up", "收回来", READY))
    out["peek_box"] = {
        "name": L("What's in the box?", "盒子里有什么？"),
        "description": L("Active perception: from where it stands the box walls hide what is inside, so the neck leans "
                         "over the box and looks at each thing in it.",
                         "主动感知：站在原地只能看到盒子的墙，于是脖子探身到盒子上方，逐个看清里面的东西。"),
        "from": "home", "speed_dps": 35, "dwell_s": 1.0, "camera_inset": "big", "waypoints": wps}
    return out, props, report, problems


# the real robot's demo (projects/torque_bringup neck_home.py --demo), bilingual copy of routes/neck_demo.json
EN = {"站起": "Stand up", "向左转头": "Turn left", "向右转头": "Turn right", "回中": "Centre", "侧摆 A": "Side tilt A",
      "侧摆 B": "Side tilt B", "滚转 A": "Roll A", "滚转 B": "Roll B", "侧摆 + 滚转": "Tilt + roll", "前倾": "Lean forward",
      "回 home": "Back to rest"}


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    routes = dict(ROUTES)
    src = json.loads((WEB / "routes/neck_demo.json").read_text(encoding="utf-8"))
    routes["robot_demo"] = {
        "name": L("The real robot's demo", "真机上的演示"),
        "description": L("The waypoints the real neck ran on 2026-11-06 (stand up, turn, side tilt, roll, lean, rest), "
                         "at 25°/s here.", "真机 2026-11-06 跑过的路点（站起、转头、侧摆、滚转、前倾、回休息位），这里用 25°/s。"),
        "from": "home", "speed_dps": 25, "dwell_s": 1.0,
        "waypoints": [{"label": L(EN[w["label"]], w["label"]), "q": w["q"]} for w in src["waypoints"]]}
    r = ROUTES
    show = ([wp("Rest (zero position)", "静止（零位）", REST, duration_s=0.5, dwell_s=1.5)] + r["wake_up"]["waypoints"][:3] + [wp("Ready", "准备姿势", READY, dwell_s=0.6)]
            + r["look_around"]["waypoints"][1:] + r["curious_tilt"]["waypoints"][1:3]
            + [wp("Upright", "回正", {"neck_j4": 0, "neck_j5": 0, "neck_j6": -25})]
            + r["peek_desk"]["waypoints"][1:2] + [wp("Back to rest", "回到休息位", REST, dwell_s=2.0)])
    routes["showcase"] = {"name": L("Homepage loop", "首页循环"),
                          "description": L("Wake up, look around, tilt, peek, rest.", "起身、环顾、歪头、探身、休息。"),
                          "from": "home", "speed_dps": 40, "dwell_s": 0.6, "waypoints": show}
    perc, props, report, problems = perception_routes()
    print("\n".join(report))
    if problems:
        raise SystemExit("perception demos: " + "; ".join(problems))
    routes.update(perc)
    (OUT / "scene.json").write_text(json.dumps({"props": props}, indent=1) + "\n", encoding="utf-8")
    for k, v in routes.items():
        (OUT / f"{k}.json").write_text(json.dumps(v, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    idx = {"demos": ["wake_up", "look_around", "inspect_block", "peek_box", "nod_shake", "curious_tilt", "peek_desk",
                     "robot_demo"], "home": "showcase"}
    (OUT / "index.json").write_text(json.dumps(idx, indent=1) + "\n", encoding="utf-8")
    print("wrote", sorted(p.name for p in OUT.iterdir()))


if __name__ == "__main__":
    main()
