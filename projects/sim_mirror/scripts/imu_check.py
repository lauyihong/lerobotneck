"""Check the neck camera orientation: D435i accelerometer (gravity) vs the MuJoCo twin at the same joint read.

    .venv/Scripts/python.exe projects/sim_mirror/scripts/imu_check.py [--seconds 2]

READ-ONLY (encoders + RealSense). Pitch / roll of the colour optical frame (x right, y down, z forward):
pitch > 0 = looking down, roll > 0 = image rotated clockwise. The D435i motion data is reported in the depth/IMU frame,
which is aligned with the optical axes (x right, y down, z forward); at rest it reads the reaction to gravity (= "up").
Colour vs depth/IMU rotation is ~0.1 deg (factory extrinsics), ignored here.
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np

import calib
from real_reader import RealReader
from sim_robot import SimWallerobot

D435_SERIAL = "261222077675"


def pitch_roll(up):
    """up = unit 'up' vector in the optical frame -> (pitch, roll) deg."""
    x, y, z = up
    pitch = np.degrees(np.arctan2(z, -y))   # camera looking down -> up vector has +z component
    roll = np.degrees(np.arctan2(-x, -y))
    return pitch, roll


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--seconds", type=float, default=2.0); a = ap.parse_args()
    import pyrealsense2 as rs
    p = rs.pipeline(); c = rs.config(); c.enable_device(D435_SERIAL)
    c.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 100)
    p.start(c); acc = []
    try:
        t0 = time.time()
        while time.time() - t0 < a.seconds:
            fs = p.wait_for_frames(1000)
            for f in fs:
                if f.is_motion_frame():
                    d = f.as_motion_frame().get_motion_data(); acc.append((d.x, d.y, d.z))
    finally:
        p.stop()
    acc = np.array(acc); g = acc.mean(0); up_real = g / np.linalg.norm(g)
    r = RealReader()
    try:
        r.assert_torque_off(); ticks = r.ticks(); r.assert_torque_off()
    finally:
        r.close()
    sim = SimWallerobot(cameras=()); sim.connect(); sim.set_state_ticks({k: v for k, v in ticks.items() if v is not None})
    cid = sim.m.camera("neck_color").id
    R_cam = sim.d.cam_xmat[cid].reshape(3, 3)          # MuJoCo camera axes in world (x right, y up, z back)
    R_opt = R_cam @ np.diag([1, -1, -1])               # optical: x right, y down, z forward
    up_sim = R_opt.T @ np.array([0, 0, 1.0])
    pr_r, pr_s = pitch_roll(up_real), pitch_roll(up_sim)
    print(f"accel samples {len(acc)}, |g| {np.linalg.norm(g):.2f} m/s^2, spread {acc.std(0).max():.3f}")
    print(f"real D435i : pitch {pr_r[0]:+6.1f} deg  roll {pr_r[1]:+6.1f} deg")
    print(f"sim  model : pitch {pr_s[0]:+6.1f} deg  roll {pr_s[1]:+6.1f} deg")
    print(f"difference : pitch {pr_r[0] - pr_s[0]:+6.1f} deg  roll {pr_r[1] - pr_s[1]:+6.1f} deg   "
          f"(angle between up vectors {np.degrees(np.arccos(np.clip(up_real @ up_sim, -1, 1))):.1f} deg)")
    out = Path(__file__).resolve().parents[1] / "outputs/imu_check.json"
    out.write_text(json.dumps({"date": time.strftime("%Y-%m-%d %H:%M:%S"), "ticks": ticks,
                               "q_deg": {k: round(float(np.degrees(v)), 2) for k, v in calib.ticks_to_q(ticks).items()},
                               "real_up_optical": up_real.round(4).tolist(), "sim_up_optical": up_sim.round(4).tolist(),
                               "real_pitch_roll": [round(v, 2) for v in pr_r], "sim_pitch_roll": [round(v, 2) for v in pr_s]}, indent=2))
    print("saved", out)


if __name__ == "__main__":
    main()
