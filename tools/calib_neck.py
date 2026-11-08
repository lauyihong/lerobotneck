"""Hand-guided calibration of neck_v0_1 (COM3 IDs 7..12 = URDF neck_j1..neck_j6). Torque stays OFF the whole time:
nothing moves by itself, you move the neck by hand.

    .venv/Scripts/python.exe tools/calib_neck.py guide       # offline: hardware/neck_v0_1/calib_guide.png (zero + '+' directions)
    .venv/Scripts/python.exe tools/calib_neck.py calibrate   # interactive, COM3 must be free (close other scripts)
    .venv/Scripts/python.exe tools/calib_neck.py show        # read encoders -> URDF angles -> hardware/neck_v0_1/live_pose.png
    .venv/Scripts/python.exe tools/calib_neck.py show --raw  # just print Present_Position / Homing_Offset / limits

calibrate (all 6 joints, or --joints neck_j5,neck_j6 to redo some):
 1. zero      per joint: put it at its URDF zero (calib_guide.png, 'zero' panels), ENTER, hold it for the countdown
              -> homing so that pose reads 2047. Each zero is relative to the previous link, so the rest of the neck
              may hang anywhere (only J2 = 'upper arm horizontal' is absolute).
 2. direction per joint: turn it ~30 deg the way its calib_guide.png panel shows (colour = +30 deg, grey = zero), ENTER
              -> sign between URDF angle and encoder ticks.
 3. range     move every joint slowly through its full range, ENTER. Encoder wrap is unwrapped continuously, so J5 may
              do > 180 deg; a range over ~1 rev is clipped symmetrically around the zero.
 4. write     EEPROM Homing_Offset (re-centred: range midpoint = 2047) + Min/Max_Position_Limit (recorded range minus
              --margin), and hardware/neck_v0_1/neck_calibration.json. The old EEPROM values are backed up first and
              written back if anything fails or you press Ctrl+C.
URDF angle [rad] = sign * (Present_Position - zero_tick) * 2*pi / 4096   (Present_Position already includes Homing_Offset)
"""
import argparse
import json
import math
import msvcrt
import sys
import threading
import time
from dataclasses import asdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
NECK = ROOT / "hardware/neck_v0_1"
URDF = NECK / "urdf/wallerobot_neck.urdf"
CALIB = NECK / "neck_calibration.json"
BACKUPS = NECK / "calib_backups"
PORT = "COM3"
JOINTS = {f"neck_j{i}": 6 + i for i in range(1, 7)}  # URDF joint -> motor ID
TICKS, HALF = 4096, 2047
MIN_TRAVEL = 200        # ticks (~18 deg): less = joint not moved during the range step
GLITCH = 400            # ticks per 20 ms sample: impossible by hand -> treat as a bad read
ZERO_HINT = {
    "neck_j1": "J1 (ID 7) 底座转动：肩部支架朝正前方，左右居中",
    "neck_j2": "J2 (ID 8)：上臂水平向前伸（绝对水平，其余关节随便）",
    "neck_j3": "J3 (ID 9)：J4 电机和上臂成一直线",
    "neck_j4": "J4 (ID 10) 侧摆：腕部不向左右摆，和 J4 电机成一直线",
    "neck_j5": "J5 (ID 11) 滚转：常态 C，相机在棕色壳下方、横放、正立",
    "neck_j6": "J6 (ID 12) 俯仰：相机光轴和 J5 滚转轴平行，朝前",
}
# render views for the guide: name:azimuth:elevation:distance[:lookat_link]  (az 90 = robot's left side, 0 = facing it)
GUIDE_VIEW = {"neck_j1": "top:90:-89:0.75", "neck_j2": "side:90:0:0.75", "neck_j3": "side:90:0:0.75",
              "neck_j4": "top:90:-89:0.75", "neck_j5": "front:0:0:0.35:neck_wrist", "neck_j6": "side:90:0:0.35:neck_wrist"}
GUIDE_DEG = 30


def rel(p):
    p = Path(p)
    return (str(p.relative_to(ROOT)) if p.is_relative_to(ROOT) else str(p)).replace("\\", "/")


def deg(ticks):
    return ticks * 360.0 / TICKS


# ---------------------------------------------------------------- rendering (offline)
def _render_mod():
    sys.path.insert(0, str(NECK))
    import render_mj
    return render_mj


def guide(a):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import mujoco
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    rm = _render_mod()
    m = mujoco.MjModel.from_xml_path(str(URDF)); d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    look0 = d.xpos[1:].mean(0).copy()
    panels = [(f"零位 zero — {v.split(':')[0]}", img) for v, (_, img) in
              zip(["side", "front", "iso"], rm.render(URDF, [0] * 6, ["side:90:0:0.8", "front:0:0:0.8", "iso:135:-25:0.9"],
                                                       (500, 500), lookat=look0))]
    for i, (j, view) in enumerate(GUIDE_VIEW.items()):
        q = [0.0] * 6; q[i] = GUIDE_DEG
        (_, z), = rm.render(URDF, [0] * 6, [view], (500, 500), lookat=look0)
        (_, p), = rm.render(URDF, q, [view], (500, 500), lookat=look0)
        bg_z, bg_p = (z >= 250).all(-1), (p >= 250).all(-1)
        out = p.copy()
        ghost = ~bg_z & bg_p
        out[ghost] = (0.3 * z[ghost] + 0.7 * 255).astype(np.uint8)
        panels.append((f"{j} (ID {JOINTS[j]})  +{GUIDE_DEG}°：彩色 = 转过去，浅灰 = 零位", out))
    fig, axs = plt.subplots(3, 3, figsize=(15, 15.5))
    for ax, (t, img) in zip(axs.flat, panels):
        ax.imshow(img); ax.set_title(t, fontsize=12); ax.axis("off")
    fig.suptitle("脖子标定指引：第一行 = 零位（参考姿态），其余 = 每个关节的 + 方向（红箭头 = 相机视线）", fontsize=15)
    plt.tight_layout(rect=(0, 0, 1, 0.97))
    out = NECK / "calib_guide.png"
    plt.savefig(out, dpi=72)
    print("saved", out)


# ---------------------------------------------------------------- bus
def open_bus():
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus
    bus = FeetechMotorsBus(PORT, {n: Motor(i, "sts3215", MotorNormMode.RANGE_M100_100) for n, i in JOINTS.items()})
    # LeRobot's handshake uses one broadcast ping; with 12 motors on COM3 (left arm + neck) replies get lost
    # (2026-11-04: 9/10/11 'missing' while all answer 10/10 individually) -> ping each neck ID instead.
    bus.connect(handshake=False)
    missing, flagged = [], {}
    for n, i in JOINTS.items():
        for _ in range(3):
            model, comm, err = bus.packet_handler.ping(bus.port_handler, i)
            if comm == 0:
                break
        if comm != 0:
            missing.append(n)
        elif err:
            flagged[n] = err
    if missing or flagged:
        bus.disconnect(disable_torque=False)
        msg = f"no answer from {missing} (IDs {[JOINTS[n] for n in missing]}); check cables / power. " if missing else ""
        if flagged:  # LeRobot refuses reads/writes while a motor reports a status error
            bits = {0: "input voltage", 1: "angle sensor", 2: "overheat", 3: "overcurrent", 5: "overload"}
            msg += "status error: " + ", ".join(
                f"{n} (ID {JOINTS[n]}): {'/'.join(v for b, v in bits.items() if e >> b & 1) or e}" for n, e in flagged.items())
            msg += (". 'input voltage' = supply above the motor's EEPROM Max_Voltage_Limit (see hardware.yaml neck_v0_1)."
                    if any(e & 1 for e in flagged.values()) else "")
        sys.exit(msg)
    return bus


class Tracker(threading.Thread):
    """Polls Present_Position of all neck motors and keeps an unwrapped (multi-turn) position + min/max per joint."""

    def __init__(self, bus):
        super().__init__(daemon=True)
        self.bus, self.lock, self.stop_ev = bus, threading.Lock(), threading.Event()
        self.names, self.errors, self.glitches = list(JOINTS), 0, 0
        raw = self._read()
        self.last = dict(raw)
        self.u = {n: float(raw[n]) for n in self.names}
        self.lo, self.hi = dict(self.u), dict(self.u)
        self.bad = {n: 0 for n in self.names}

    def _read(self):
        return self.bus.sync_read("Present_Position", self.names, normalize=False, num_retry=3)

    def run(self):
        while not self.stop_ev.is_set():
            with self.lock:
                try:
                    raw = self._read()
                except Exception:
                    self.errors += 1
                    raw = None
                if raw:
                    for n in self.names:
                        dt = (raw[n] - self.last[n] + TICKS // 2) % TICKS - TICKS // 2
                        if abs(dt) > GLITCH and self.bad[n] < 5:  # one-off bad value: skip it
                            self.bad[n] += 1; self.glitches += 1
                            continue
                        self.bad[n] = 0
                        self.last[n] = raw[n]; self.u[n] += dt
                        self.lo[n] = min(self.lo[n], self.u[n]); self.hi[n] = max(self.hi[n], self.u[n])
            time.sleep(0.02)

    def rehome(self, n):
        """Call with self.lock held, right after changing n's Homing_Offset."""
        raw = self.bus.read("Present_Position", n, normalize=False, num_retry=3)
        self.last[n], self.u[n] = raw, float(raw)
        self.lo[n] = self.hi[n] = float(raw)

    def reset_minmax(self):
        with self.lock:
            self.lo, self.hi = dict(self.u), dict(self.u)

    def snap(self):
        with self.lock:
            return dict(self.u)

    def stop(self):
        self.stop_ev.set()
        if self.is_alive():
            self.join(1)


def wait_enter():
    while msvcrt.kbhit():
        msvcrt.getwch()
    input()


def calibrate(a):
    from lerobot.motors import MotorCalibration
    sel = a.joints.split(",") if a.joints else list(JOINTS)
    assert all(j in JOINTS for j in sel), f"--joints must be from {list(JOINTS)}"
    prev = json.loads(CALIB.read_text()) if CALIB.exists() else None
    if prev is None and set(sel) != set(JOINTS):
        sys.exit("no neck_calibration.json yet -> calibrate all 6 joints first")
    print(f"Neck calibration on {PORT}, joints {sel}. Torque stays OFF; you move the neck by hand.")
    print(f"Open {NECK / 'calib_guide.png'} (run 'guide' first if it is missing).")

    bus = open_bus()
    tr = None
    try:
        bus.disable_torque()  # also unlocks EEPROM (Lock = 0)
        old = bus.read_calibration()
        BACKUPS.mkdir(exist_ok=True)
        bk = BACKUPS / f"eeprom_{time.strftime('%Y%m%d-%H%M%S')}.json"
        bk.write_text(json.dumps({n: asdict(c) for n, c in old.items()}, indent=2))
        print("old EEPROM calibration backed up to", bk)
        tr = Tracker(bus); tr.start()
        try:
            # ---- 1. zero
            print(f"\n== 1/3 零位：每个关节单独摆到零位，按 ENTER，然后保持 {a.countdown:.0f} 秒 ==")
            zero_u = {}
            for n in sel:
                print(f"[{n}] {ZERO_HINT[n]}  -> ENTER", end=" ", flush=True)
                wait_enter()
                for k in range(int(a.countdown), 0, -1):
                    print(f"{k}..", end="", flush=True); time.sleep(1)
                with tr.lock:
                    bus.set_half_turn_homings([n])
                    tr.rehome(n)
                    zero_u[n] = tr.u[n]
                print(f" captured (reads {zero_u[n]:.0f})\a")

            # ---- 2. direction
            print(f"\n== 2/3 方向：按 calib_guide.png 里每个关节那一格，往'彩色'那边转约 {GUIDE_DEG}°，按 ENTER ==")
            print("   先把脖子放成一个松手也不会自己掉的姿势（比如托住/靠住），每次只转提示的那一个关节，转完再按 ENTER")
            sign = {}
            for n in sel:
                while True:
                    s0 = tr.snap()
                    print(f"[{n}] 从当前位置往 + 方向转约 {GUIDE_DEG}° -> ENTER", end=" ", flush=True)
                    wait_enter()
                    s1 = tr.snap()
                    dl = {k: s1[k] - s0[k] for k in JOINTS}
                    top = max(dl, key=lambda k: abs(dl[k]))
                    ok = abs(dl[n]) >= 120
                    if ok and top != n:  # another joint moved more (e.g. the neck sagged under gravity): ask
                        others = ", ".join(f"{k} {deg(v):+.0f}°" for k, v in dl.items() if k != n and abs(v) >= 120)
                        ok = input(f"\n  {n} 转了 {deg(dl[n]):+.0f}°，但别的关节也动了（{others}）。"
                                   f"刚才转的确实是 {n} 吗？[y/N] ").strip().lower() == "y"
                    if ok:
                        sign[n] = 1 if dl[n] > 0 else -1
                        print(f"  {dl[n]:+.0f} ticks ({deg(dl[n]):+.0f}°) -> sign {sign[n]:+d}")
                        break
                    print(f"  !! {n} moved {deg(dl[n]):+.0f}° (need >= 11°), biggest motion {top} "
                          f"({deg(dl[top]):+.0f}°). Try again.")

            # ---- 3. range
            print("\n== 3/3 范围：把每个关节慢慢转到两端（不要硬顶），全部做完按 ENTER ==")
            print("   显示的是 URDF 角度（度），方括号 = 记录到的最小..最大")
            tr.reset_minmax()
            while msvcrt.kbhit():
                msvcrt.getwch()
            while not (msvcrt.kbhit() and msvcrt.getwch() in "\r\n"):
                with tr.lock:
                    u, lo, hi = dict(tr.u), dict(tr.lo), dict(tr.hi)
                parts = []
                for n in sel:
                    f = lambda t: sign[n] * deg(t - zero_u[n])
                    a1, a2 = sorted((f(lo[n]), f(hi[n])))
                    parts.append(f"j{n[-1]} {f(u[n]):+4.0f}[{a1:+4.0f},{a2:+4.0f}]")
                print("\r" + "  ".join(parts) + "   ", end="", flush=True)
                time.sleep(0.1)
            print()
            with tr.lock:
                lo, hi, u_end = dict(tr.lo), dict(tr.hi), dict(tr.u)
                homing_now = {n: bus.read("Homing_Offset", n, normalize=False) for n in sel}
            tr.stop()
            if tr.errors or tr.glitches:
                print(f"   (bus read errors {tr.errors}, skipped glitch samples {tr.glitches})")

            # ---- 4. compute
            maxspan = TICKS - 1 - 2 * a.margin - 10
            new, info = {}, {}
            for n in sel:
                z, l, h = zero_u[n], min(lo[n], zero_u[n]), max(hi[n], zero_u[n])
                if hi[n] - lo[n] < MIN_TRAVEL:
                    raise RuntimeError(f"{n}: recorded travel {deg(hi[n] - lo[n]):.0f}° < {deg(MIN_TRAVEL):.0f}° "
                                       "-> range step not done for this joint")
                clipped = h - l > maxspan
                if clipped:  # > ~1 rev (J5): keep a symmetric window around the zero
                    l, h = max(l, z - maxspan / 2), min(h, z + maxspan / 2)
                    if h - l > maxspan:
                        h = l + maxspan
                    print(f"  {n}: recorded {deg(hi[n] - lo[n]):.0f}° > 1 rev -> limited to {deg(h - l):.0f}°")
                # Present = (Actual - Homing_Offset) mod 4096, so raising the offset by s lowers every reading by s
                # (mod 4096). Shift so the range midpoint reads 2047; the offset itself is wrapped into the
                # sign-magnitude range |offset| <= 2047 (a 4096 change is invisible in the mod-4096 reading).
                s = int(round((l + h) / 2)) - HALF
                hom = (homing_now[n] + s + TICKS // 2) % TICKS - TICKS // 2
                if hom == -TICKS // 2:  # -2048 not representable -> one tick off
                    hom, s = -(TICKS // 2 - 1), s + 1
                zt = int(round(z - s))
                rmin, rmax = int(math.ceil(l - s + a.margin)), int(math.floor(h - s - a.margin))
                if not (0 <= rmin < rmax <= TICKS - 1):
                    raise RuntimeError(f"{n}: inconsistent range {rmin}..{rmax} (please report)")
                if not rmin <= zt <= rmax:
                    print(f"  !! {n}: zero pose ({zt}) is within --margin of a range end -> check the zero / range")
                d_lo, d_hi = (rmin - zt) * 2 * math.pi / TICKS, (rmax - zt) * 2 * math.pi / TICKS
                lower, upper = (d_lo, d_hi) if sign[n] > 0 else (-d_hi, -d_lo)
                new[n] = MotorCalibration(id=JOINTS[n], drive_mode=0, homing_offset=int(hom), range_min=rmin, range_max=rmax)
                info[n] = {"id": JOINTS[n], "homing_offset": int(hom), "range_min": rmin, "range_max": rmax,
                           "zero_tick": zt, "sign": sign[n], "lower_rad": round(lower, 4), "upper_rad": round(upper, 4),
                           "recorded_travel_ticks": int(hi[n] - lo[n]), "clipped": bool(clipped),
                           "expected_now_tick": int(round(u_end[n] - s)) % TICKS}
                print(f"  {n}: homing {hom:+5d}  range {rmin:4d}..{rmax:4d}  zero {zt:4d}  sign {sign[n]:+d}  "
                      f"URDF {math.degrees(lower):+5.0f}..{math.degrees(upper):+5.0f}°")

            if input("\nWrite this to the motors' EEPROM + neck_calibration.json? [y/N] ").strip().lower() != "y":
                raise KeyboardInterrupt("not confirmed")
            bus.write_calibration(new)
            # ---- verify
            back = bus.read_calibration()
            pos = bus.sync_read("Present_Position", sel, normalize=False, num_retry=5)
            for n in sel:
                b = back[n]
                if (b.homing_offset, b.range_min, b.range_max) != (new[n].homing_offset, new[n].range_min, new[n].range_max):
                    raise RuntimeError(f"{n}: EEPROM read-back {b} != written {new[n]}")
                err = (pos[n] - info[n]["expected_now_tick"] + TICKS // 2) % TICKS - TICKS // 2
                print(f"  {n}: now {pos[n]} (expected {info[n]['expected_now_tick']}, diff {err:+d})"
                      + ("  !! moved since ENTER?" if abs(err) > 25 else ""))
                info[n].pop("expected_now_tick")
        except BaseException as e:
            if tr:
                tr.stop()
            print(f"\n!! {type(e).__name__}: {e} -> restoring the old EEPROM calibration for {sel}")
            bus.write_calibration({n: old[n] for n in sel})
            raise
        out = prev or {}
        out.update({"robot": "wallerobot neck_v0_1", "port": PORT, "urdf": rel(URDF),
                    "convention": "q_rad = sign * (Present_Position - zero_tick) * 2*pi/4096; Present_Position includes "
                                  "the EEPROM Homing_Offset; motor limits = range_min..range_max",
                    "margin_ticks": a.margin, "eeprom_backup": rel(bk)})
        out.setdefault("joints", {}).update(info)
        out.setdefault("history", []).append({"date": time.strftime("%Y-%m-%d %H:%M:%S"), "joints": sel})
        CALIB.write_text(json.dumps(out, indent=2, ensure_ascii=False))
        print("saved", CALIB)
        print("Next: tools/calib_neck.py show   (renders the URDF at the measured angles)")
    finally:
        if tr:
            tr.stop()
        bus.disconnect()  # torque off


def angles(cal, pos):
    return {n: c["sign"] * (pos[n] - c["zero_tick"]) * 2 * math.pi / TICKS for n, c in cal.items()}


def show(a):
    cal = None if a.raw else json.loads(CALIB.read_text())["joints"]
    bus = open_bus()
    try:
        pos = bus.sync_read("Present_Position", list(JOINTS), normalize=False, num_retry=5)
        eep = bus.read_calibration()
    finally:
        bus.disconnect()
    for n in JOINTS:
        e = eep[n]
        line = f"{n} (ID {JOINTS[n]}): pos {pos[n]:4d}  homing {e.homing_offset:+5d}  limits {e.range_min:4d}..{e.range_max:4d}"
        if cal:
            c = cal[n]
            if (e.homing_offset, e.range_min, e.range_max) != (c["homing_offset"], c["range_min"], c["range_max"]):
                line += "  !! EEPROM differs from neck_calibration.json"
            q = c["sign"] * (pos[n] - c["zero_tick"]) * 360 / TICKS
            line += f"  ->  {q:+6.1f}° (URDF {math.degrees(c['lower_rad']):+.0f}..{math.degrees(c['upper_rad']):+.0f})"
        print(line)
    if not cal:
        return
    q = [math.degrees(v) for v in angles(cal, pos).values()]
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    rm = _render_mod()
    imgs = rm.render(URDF, q, ["side:90:0:0.9", "front:0:0:0.9", "top:90:-89:0.9", "iso:135:-25:1.0"])
    fig, axs = plt.subplots(2, 2, figsize=(11, 11))
    for ax, (name, img) in zip(axs.flat, imgs):
        ax.imshow(img); ax.set_title(name); ax.axis("off")
    fig.suptitle("live encoders: q = " + ", ".join(f"{v:+.0f}" for v in q) + " deg  " + time.strftime("%H:%M:%S"))
    plt.tight_layout()
    out = NECK / "live_pose.png"
    plt.savefig(out, dpi=72)
    print("saved", out)


def save_pose(a):
    """Read the encoders (torque untouched) and store the pose as poses[<name>] in neck_calibration.json.
    q_deg is the primary value (survives a re-calibration only if the zeros are re-done the same way); ticks are for reference."""
    data = json.loads(CALIB.read_text(encoding="utf-8"))
    cal = data["joints"]
    bus = open_bus()
    try:
        pos = bus.sync_read("Present_Position", list(JOINTS), normalize=False, num_retry=5)
    finally:
        bus.disconnect()
    q = angles(cal, pos)
    pose = {"date": time.strftime("%Y-%m-%d %H:%M:%S"), "q_deg": {}, "ticks": {}, "outside_limits": [],
            "note": "q_deg/ticks = measured rest pose; target_* = clamped into the motor limits (use these as the goal: "
                    "a joint resting on a mechanical stop must not be commanded further into it)",
            "target_ticks": {}, "target_q_deg": {}}
    for n in JOINTS:
        c = cal[n]
        pose["q_deg"][n] = round(math.degrees(q[n]), 2)
        pose["ticks"][n] = int(pos[n])
        t = min(max(int(pos[n]), c["range_min"]), c["range_max"])
        pose["target_ticks"][n] = t
        pose["target_q_deg"][n] = round(c["sign"] * (t - c["zero_tick"]) * 360 / TICKS, 2)
        flag = ""
        if t != pos[n]:
            pose["outside_limits"].append(n)
            flag = (f"  !! outside motor limits {c['range_min']}..{c['range_max']} -> target {t} "
                    f"({pose['target_q_deg'][n]:+.2f}°)")
        print(f"{n}: {pos[n]:4d} ticks  {math.degrees(q[n]):+7.2f}°{flag}")
    data.setdefault("poses", {})[a.name] = pose
    CALIB.write_text(json.dumps(data, indent=2, ensure_ascii=False))
    print(f"saved pose '{a.name}' to", CALIB)


def main():
    for f in (sys.stdout, sys.stderr):  # Chinese prompts also when piped / on a cp1252 console
        f.reconfigure(encoding="utf-8", errors="replace")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("guide")
    c = sub.add_parser("calibrate")
    c.add_argument("--joints", default="", help="comma list, e.g. neck_j5,neck_j6 (default all)")
    c.add_argument("--margin", type=int, default=40, help="ticks kept away from each recorded end (40 = 3.5 deg)")
    c.add_argument("--countdown", type=float, default=3, help="seconds to hold the zero pose after ENTER")
    s = sub.add_parser("show")
    s.add_argument("--raw", action="store_true", help="no calibration file needed; just print registers")
    p = sub.add_parser("save-pose", help="store the current encoder pose in neck_calibration.json")
    p.add_argument("name", help="e.g. home")
    a = ap.parse_args()
    {"guide": guide, "calibrate": calibrate, "show": show, "save-pose": save_pose}[a.cmd](a)


if __name__ == "__main__":
    main()
