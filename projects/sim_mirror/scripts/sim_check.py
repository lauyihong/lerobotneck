"""Headless smoke test of the MuJoCo twin (no hardware).

    .venv/Scripts/python.exe projects/sim_mirror/scripts/sim_check.py [--pose outputs/rest_pose_20261105.json]

1. teleport to the real rest pose (ticks from a real_reader.py --json read), hold it under physics for 3 s -> drift
2. sinusoid +/-0.3 rad around a mid pose on every joint for 3 s -> tracking error, NaN check
3. renders outputs/sim_cams.png (neck_color + wrist cams at the rest pose) and outputs/sim_overview.png
"""
import argparse
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from sim_robot import SimWallerobot  # noqa: E402

OUT = HERE.parent / "outputs"


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--pose", type=Path, default=OUT / "rest_pose_20261105.json")
    a = ap.parse_args()
    ticks = json.loads(a.pose.read_text())["ticks"]
    sim = SimWallerobot(); sim.connect()
    sim.set_state_ticks(ticks); q_rest = sim.qpos()
    cams = {c: sim.render(c).copy() for c in sim.cam_names}
    # 1) hold
    for _ in range(1500):
        sim.step(1)
    drift = {n: abs(sim.qpos()[n] - q_rest[n]) for n in q_rest}
    worst = max(drift, key=drift.get)
    print(f"hold 3 s at rest: max drift {np.degrees(drift[worst]):.2f} deg ({worst}), contacts {sim.d.ncon}")
    # 2) sinusoid around the middle of each joint range
    mid = {n: float(np.mean(sim.m.jnt_range[sim.m.joint(n).id])) for n in sim.joints}
    sim.set_state(mid); err = []
    for k in range(1500):
        t = k * sim.m.opt.timestep
        sim.send_action({n: mid[n] + 0.3 * np.sin(2 * np.pi * 0.5 * t) for n in sim.joints}); sim.step(1)
        if k > 250:
            err.append(max(abs(sim.qpos()[n] - (mid[n] + 0.3 * np.sin(2 * np.pi * 0.5 * t))) for n in sim.joints))
    assert np.isfinite(sim.d.qpos).all(), "NaN in qpos"
    print(f"sinusoid 0.5 Hz +/-0.3 rad: max tracking error {np.degrees(max(err)):.1f} deg, no NaN")
    # 3) renders
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei"]
    fig, axs = plt.subplots(1, 3, figsize=(15, 4.2))
    for ax, (c, im) in zip(axs, cams.items()):
        ax.imshow(im); ax.set_title(f"sim {c} (rest pose)"); ax.axis("off")
    plt.tight_layout(); plt.savefig(OUT / "sim_cams.png", dpi=70); plt.close(fig)
    sim.set_state_ticks(ticks)
    r = mujoco.Renderer(sim.m, 600, 800); cam = mujoco.MjvCamera(); cam.lookat[:] = (-0.1, 0, 0.15)
    cam.distance, cam.azimuth, cam.elevation = 1.1, 35, -20
    r.update_scene(sim.d, cam); plt.imsave(OUT / "sim_overview.png", r.render())
    print("saved", OUT / "sim_cams.png", OUT / "sim_overview.png")
    sim.disconnect()


if __name__ == "__main__":
    main()
