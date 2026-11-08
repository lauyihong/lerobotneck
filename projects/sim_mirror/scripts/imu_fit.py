"""Neck camera orientation calibration from several hand-posed neck poses: D435i gravity vs the MuJoCo twin.

    .venv/Scripts/python.exe projects/sim_mirror/scripts/imu_fit.py              # interactive (run in your terminal)
    .venv/Scripts/python.exe projects/sim_mirror/scripts/imu_fit.py --fit-only   # re-fit outputs/imu_samples.json
    .venv/Scripts/python.exe projects/sim_mirror/scripts/imu_fit.py --selftest   # synthetic data, no hardware

READ-ONLY (encoder reads + RealSense accelerometer). Pose the neck by hand, hold it still, press ENTER -> one sample
(1 s accel mean + joint ticks). Aim for 8+ poses that differ in J4 (side swing), J5 (roll) and J6 (tilt), each hold
for 1 s; keep the neck supported so J2 doesn't sag. 'f' = fit now, 'q' = fit and quit.

Model: up_optical = R_mount(roll)^T * R_optical(q + delta)^T * z_world. Fitted: delta J2 (J2 and J3 axes are always
parallel -> only their SUM is observable from gravity; J3 is pinned by the fold contact, so it goes into J2),
delta J4, J5, J6 and the camera roll in the holder. J1 (vertical axis) is unobservable from gravity. Camera pitch in
the holder = J6 offset (same axis) -> reported together.
"""
import argparse
import json
import sys
import time
from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import least_squares

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import calib  # noqa: E402

SCENE = HERE.parent / "models/scene.xml"
SAMPLES = HERE.parent / "outputs/imu_samples.json"
PARAMS = ["neck_j2", "neck_j4", "neck_j5", "neck_j6", "cam_roll"]
D435_SERIAL = "261222077675"
CHECK = HERE.parent / "outputs/imu_check.json"
# hand-held neck: ~0.1 m/s^2 tremor is fine (1 s mean of ~100 readings -> ~0.1 deg); real motion shows as > 0.3
ACC_STD_MAX = 0.25


def load_samples(include_check=True):
    samples = json.loads(SAMPLES.read_text())["samples"] if SAMPLES.exists() else []
    if include_check and CHECK.exists():
        c = json.loads(CHECK.read_text())
        samples = samples + [{"time": c["date"], "ticks": c["ticks"], "up": c["real_up_optical"], "source": "imu_check"}]
    return samples


class Twin:
    def __init__(self):
        self.m = mujoco.MjModel.from_xml_path(str(SCENE)); self.d = mujoco.MjData(self.m)
        self.cid = self.m.camera("neck_color").id

    def up_optical(self, q: dict, p) -> np.ndarray:
        for n, v in q.items():
            self.d.qpos[self.m.jnt_qposadr[self.m.joint(n).id]] = v + (p[PARAMS.index(n)] if n in PARAMS else 0.0)
        mujoco.mj_kinematics(self.m, self.d); mujoco.mj_camlight(self.m, self.d)
        R_opt = self.d.cam_xmat[self.cid].reshape(3, 3) @ np.diag([1, -1, -1])
        up = R_opt.T @ np.array([0, 0, 1.0])
        c, s = np.cos(p[4]), np.sin(p[4])           # camera rotated in its holder about the optical z axis
        return np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]]) @ up


def fit(samples, twin=None, free=PARAMS):
    twin = twin or Twin()
    idx = [PARAMS.index(n) for n in free]
    qs = [calib.ticks_to_q({k: v for k, v in s["ticks"].items() if v is not None}) for s in samples]
    ups = [np.array(s["up"]) / np.linalg.norm(s["up"]) for s in samples]

    def full(x):
        p = np.zeros(len(PARAMS)); p[idx] = x; return p

    def res(x):
        return np.concatenate([twin.up_optical(q, full(x)) - u for q, u in zip(qs, ups)])

    k = len(idx); r0 = res(np.zeros(k)); sol = least_squares(res, np.zeros(k), x_scale=0.1)
    # each sample constrains 2 DoF (unit vector) -> dof = 2n - k; floor at 1 so sigma stays defined
    J = sol.jac; cov = np.linalg.pinv(J.T @ J) * (sol.fun @ sol.fun) / max(1, 2 * len(samples) - k)
    err = lambda r: np.degrees(np.arcsin(np.clip(np.linalg.norm(r.reshape(-1, 3), axis=1), 0, 1)))  # noqa: E731
    out = {"n": len(samples), "free": list(free), "delta_deg": {n: round(float(np.degrees(v)), 2) for n, v in zip(free, sol.x)},
           "sigma_deg": {n: round(float(np.degrees(np.sqrt(max(c, 0)))), 2) for n, c in zip(free, np.diag(cov))},
           "corr": np.round(cov / np.sqrt(np.outer(np.diag(cov), np.diag(cov)) + 1e-30), 2).tolist(),
           "rms_before_deg": round(float(np.sqrt(np.mean(err(r0) ** 2))), 2),
           "rms_after_deg": round(float(np.sqrt(np.mean(err(sol.fun) ** 2))), 2),
           "per_sample_after_deg": [round(float(e), 2) for e in err(sol.fun)]}
    return out


def collect():
    import pyrealsense2 as rs
    from real_reader import RealReader
    reader = RealReader(); reader.assert_torque_off()
    p = rs.pipeline(); c = rs.config(); c.enable_device(D435_SERIAL); c.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 100)
    p.start(c)
    samples = json.loads(SAMPLES.read_text())["samples"] if SAMPLES.exists() and "--append" in sys.argv else []
    try:
        while True:
            cmd = input(f"[{len(samples)} samples] pose the neck, hold still, ENTER = sample, f = fit, q = quit: ").strip().lower()
            if cmd in ("f", "q"):
                if len(samples) >= 3:
                    print(json.dumps(fit(samples), indent=2))
                else:
                    print("need >= 3 samples")
                if cmd == "q":
                    break
                continue
            t0 = time.time(); acc = []; tk0 = reader.ticks()
            while time.time() - t0 < 1.0:
                for f in p.wait_for_frames(1000):
                    if f.is_motion_frame():
                        d = f.as_motion_frame().get_motion_data(); acc.append((d.x, d.y, d.z))
            tk1 = reader.ticks(); acc = np.array(acc)
            moved = max(abs(tk1[k] - tk0[k]) for k in tk0 if k.startswith("neck") and tk0[k] is not None and tk1[k] is not None)
            if moved > 4 or acc.std(0).max() > ACC_STD_MAX:
                print(f"  moved during the sample (encoders {moved} ticks, accel spread {acc.std(0).max():.3f}) -> discarded, hold still")
                continue
            q = calib.ticks_to_q(tk1)
            samples.append({"time": time.strftime("%H:%M:%S"), "ticks": tk1, "up": (acc.mean(0) / np.linalg.norm(acc.mean(0))).tolist(),
                            "q_deg": {k: round(float(np.degrees(v)), 1) for k, v in q.items() if k.startswith("neck")}})
            print("  sample", len(samples), samples[-1]["q_deg"])
            SAMPLES.write_text(json.dumps({"samples": samples}, indent=2))
    finally:
        p.stop(); reader.assert_torque_off(); reader.close(); print("torque still off on all 18 motors; samples in", SAMPLES)


def selftest():
    twin = Twin(); rng = np.random.default_rng(0)
    true = np.radians([-6.0, 2.0, -3.0, 5.0, 1.5])
    base = {n: c["zero_tick"] for n, c in calib.CAL.items() if c["type"] == "revolute"}
    base.update({n: c["closed_tick"] for n, c in calib.CAL.items() if c["type"] == "gripper"})
    samples = []
    for _ in range(10):
        q_deg = {"neck_j1": rng.uniform(-40, 40), "neck_j2": rng.uniform(20, 150), "neck_j3": rng.uniform(20, 150),
                 "neck_j4": rng.uniform(-80, 80), "neck_j5": rng.uniform(-150, 60), "neck_j6": rng.uniform(-60, 60)}
        ticks = dict(base); ticks.update({n: calib.rad_to_tick(n, np.radians(v)) for n, v in q_deg.items()})
        q = calib.ticks_to_q(ticks)
        up = twin.up_optical(q, true) + rng.normal(0, 0.003, 3)
        samples.append({"ticks": ticks, "up": up.tolist()})
    out = fit(samples, twin)
    print("true deg  ", dict(zip(PARAMS, np.degrees(true).round(2))))
    print("fitted    ", out["delta_deg"], "sigma", out["sigma_deg"], "rms", out["rms_before_deg"], "->", out["rms_after_deg"])
    ok = all(abs(out["delta_deg"][n] - np.degrees(t)) < 0.5 for n, t in zip(PARAMS, true))
    print("SELFTEST", "PASS" if ok else "FAIL"); return ok


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--fit-only", action="store_true"); ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--append", action="store_true")
    ap.add_argument("--params", default=",".join(PARAMS), help="comma list of free parameters (others fixed at 0)")
    ap.add_argument("--no-check", action="store_true", help="don't add outputs/imu_check.json as an extra sample")
    a = ap.parse_args()
    if a.selftest:
        raise SystemExit(0 if selftest() else 1)
    if a.fit_only:
        free = [n.strip() for n in a.params.split(",") if n.strip()]
        print(json.dumps(fit(load_samples(not a.no_check), free=free), indent=2))
    else:
        collect()
