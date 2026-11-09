"""Write the LeRobotNeck demo routes (projects/web_sim/web/public/routes_neck/*.json, labels in English and Chinese).

    .venv/Scripts/python.exe projects/web_sim/scripts/make_neck_routes.py

The routes are data for the showcase website; this script is their source (edit the poses here, then re-run).
Angles in degrees, partial waypoints keep the other joints (format: web/src/routes.js).
"""
import json
from pathlib import Path

WEB = Path(__file__).resolve().parents[1] / "web/public"
OUT = WEB / "routes_neck"
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
        "from": "current", "speed_dps": 45, "dwell_s": 0.7, "waypoints": [
            wp("Ready", "准备姿势", READY),
            wp("Look left", "看左边", {"neck_j1": -55, "neck_j6": -20}),
            wp("Look up", "抬头", {"neck_j6": -55}),
            wp("Look right", "看右边", {"neck_j1": 50, "neck_j6": -30}),
            wp("Look down", "低头", {"neck_j6": 10}),
            wp("Back to centre", "回到正前方", {"neck_j1": 0, "neck_j6": -25})]},
    "nod_shake": {
        "name": L("Yes and no", "点头和摇头"),
        "description": L("A quick nod (J6) and a head shake (J1).", "快速点头（J6），再摇头（J1）。"),
        "from": "current", "speed_dps": 90, "dwell_s": 0, "waypoints": [
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
        "from": "current", "speed_dps": 35, "dwell_s": 1.0, "waypoints": [
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
        "from": "current", "speed_dps": 35, "dwell_s": 0.9, "waypoints": [
            wp("Ready", "准备姿势", READY, dwell_s=0.3),
            wp("Lean forward", "向前探身", {"neck_j2": 55, "neck_j3": 115, "neck_j6": 15}),
            wp("Look left", "往左看", {"neck_j1": -25}),
            wp("Look right", "往右看", {"neck_j1": 25}),
            wp("Back up", "收回来", {"neck_j1": 0, "neck_j2": 90, "neck_j3": 150, "neck_j6": -25})]},
}

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
    show = (r["wake_up"]["waypoints"][:3] + [wp("Ready", "准备姿势", READY, dwell_s=0.6)]
            + r["look_around"]["waypoints"][1:] + r["curious_tilt"]["waypoints"][1:3]
            + [wp("Upright", "回正", {"neck_j4": 0, "neck_j5": 0, "neck_j6": -25})]
            + r["peek_desk"]["waypoints"][1:2] + [wp("Back to rest", "回到休息位", REST, dwell_s=2.0)])
    routes["showcase"] = {"name": L("Homepage loop", "首页循环"),
                          "description": L("Wake up, look around, tilt, peek, rest.", "起身、环顾、歪头、探身、休息。"),
                          "from": "home", "speed_dps": 40, "dwell_s": 0.6, "waypoints": show}
    for k, v in routes.items():
        (OUT / f"{k}.json").write_text(json.dumps(v, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    idx = {"demos": ["wake_up", "look_around", "nod_shake", "curious_tilt", "peek_desk", "robot_demo"], "home": "showcase"}
    (OUT / "index.json").write_text(json.dumps(idx, indent=1) + "\n", encoding="utf-8")
    print("wrote", sorted(p.name for p in OUT.iterdir()))


if __name__ == "__main__":
    main()
