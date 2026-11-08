"""Live sim-real mirror, READ-ONLY on the robot: real encoders drive the MuJoCo twin, cameras shown side by side.

    .venv/Scripts/python.exe projects/sim_mirror/scripts/mirror.py [--left-wrist 0 --right-wrist 2] [--viewer] [--no-cams]

Window: top row = real (D435i colour | left wrist | right wrist), bottom row = sim cameras at the same joint state.
Keys (in the window): s = save snapshot (outputs/mirror/<time>_*.jpg + .json), q / Esc = quit.
--viewer also opens the interactive MuJoCo viewer.

Safety: only READ instructions go to the motors (tools/sts.py reads of Present_Position / Torque_Enable); torque is
checked OFF at start and end. Close every other program that uses COM3/COM4, the D435i or the wrist cams first.
Wrist cam indices change after re-plugging -> projects/sim_mirror/scripts/probe_cams.py (2026-11-05: 0 = left, 2 = right, user-confirmed;
1 = D435i UVC - never open index 1 here, it freezes the pyrealsense colour stream).
"""
import argparse
import json
import threading
import time
from pathlib import Path

import cv2
import numpy as np

import calib
from real_reader import RealReader
from sim_robot import SimWallerobot

OUT = Path(__file__).resolve().parents[1] / "outputs/mirror"
D435_SERIAL = "261222077675"
TILE = (320, 427)  # h, w


class Cam(threading.Thread):
    """Latest-frame grabber for an OpenCV camera."""

    def __init__(self, index):
        super().__init__(daemon=True)
        self.cap = cv2.VideoCapture(index, cv2.CAP_MSMF)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640); self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.frame, self.run_ = None, True

    def run(self):
        while self.run_:
            ok, f = self.cap.read()
            if ok:
                self.frame = f
            else:
                time.sleep(0.01)

    def stop(self):
        self.run_ = False; self.join(1); self.cap.release()


class RealSense:
    def __init__(self):
        import pyrealsense2 as rs
        self.rs = rs; self.p = rs.pipeline(); c = rs.config(); c.enable_device(D435_SERIAL)
        c.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.p.start(c); self.frame = None

    def latest(self):
        fs = self.p.poll_for_frames()
        if fs:
            cf = fs.get_color_frame()
            if cf:
                self.frame = np.asanyarray(cf.get_data()).copy()
        return self.frame

    def stop(self):
        self.p.stop()


def tile(img, label, bgr=True):
    h, w = TILE
    t = np.full((h, w, 3), 30, np.uint8) if img is None else cv2.resize(img if bgr else img[:, :, ::-1], (w, h))
    cv2.putText(t, label, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
    return t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--left-wrist", type=int, default=0); ap.add_argument("--right-wrist", type=int, default=2)
    ap.add_argument("--no-cams", action="store_true"); ap.add_argument("--viewer", action="store_true")
    ap.add_argument("--once", type=Path, help="headless: warm up 3 s, save one composite image here, exit")
    a = ap.parse_args()
    if 1 in (a.left_wrist, a.right_wrist):
        raise SystemExit("index 1 is the D435i UVC device (2026-11-05 probe) - re-run probe_cams.py and pick the wrist cams")
    OUT.mkdir(parents=True, exist_ok=True)
    reader = RealReader(); reader.assert_torque_off()
    sim = SimWallerobot(); sim.connect()
    rs = cams = None; viewer = None
    try:
        if not a.no_cams:
            rs = RealSense()
            cams = {"left_wrist": Cam(a.left_wrist), "right_wrist": Cam(a.right_wrist)}
            for c in cams.values():
                c.start()
        if a.viewer:
            import mujoco.viewer
            viewer = mujoco.viewer.launch_passive(sim.m, sim.d)
        print("mirror running - s = snapshot, q = quit (focus the image window)")
        n, t0, hz = 0, time.perf_counter(), 0.0; t_start = time.perf_counter()
        while True:
            ticks = reader.ticks()
            sim.set_state_ticks({k: v for k, v in ticks.items() if v is not None})
            if viewer is not None:
                viewer.sync()
            real = {"neck_color": rs.latest() if rs else None,
                    "left_wrist": cams["left_wrist"].frame if cams else None,
                    "right_wrist": cams["right_wrist"].frame if cams else None}
            simimg = {c: sim.render(c) for c in sim.cam_names}
            top = np.hstack([tile(real[c], f"real {c}") for c in sim.cam_names])
            bot = np.hstack([tile(simimg[c], f"sim {c}", bgr=False) for c in sim.cam_names])
            q = calib.ticks_to_q({k: v for k, v in ticks.items() if v is not None})
            line = "  ".join(f"{k.replace('_arm_', '.').replace('neck_', 'n')} {np.degrees(v):+.0f}" for k, v in q.items())
            info = np.full((44, top.shape[1], 3), 20, np.uint8)
            cv2.putText(info, f"{hz:4.1f} Hz  misses {reader.misses}   s=snapshot q=quit", (6, 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(info, line[:200], (6, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.33, (200, 200, 200), 1, cv2.LINE_AA)
            frame = np.vstack([top, bot, info])
            if a.once:
                if time.perf_counter() - t_start > 3:
                    cv2.imwrite(str(a.once), frame); print("saved", a.once, f"{hz:.1f} Hz"); break
                k = 0
            else:
                cv2.imshow("wallerobot mirror (top real / bottom sim)", frame)
                k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
            if k == ord("s"):
                stamp = time.strftime("%Y%m%d-%H%M%S")
                for c in sim.cam_names:
                    if real[c] is not None:
                        cv2.imwrite(str(OUT / f"{stamp}_real_{c}.jpg"), real[c])
                    cv2.imwrite(str(OUT / f"{stamp}_sim_{c}.jpg"), simimg[c][:, :, ::-1])
                (OUT / f"{stamp}.json").write_text(json.dumps({"ticks": ticks, "q_deg": {k: round(float(np.degrees(v)), 2)
                                                                                        for k, v in q.items()}}, indent=2))
                print("snapshot", stamp)
            n += 1
            if n % 20 == 0:
                hz = 20 / (time.perf_counter() - t0); t0 = time.perf_counter()
    finally:
        cv2.destroyAllWindows()
        if viewer is not None:
            viewer.close()
        if cams:
            for c in cams.values():
                c.stop()
        if rs:
            rs.stop()
        try:
            reader.assert_torque_off(); print("torque still off on all 18 motors")
        finally:
            reader.close(); sim.disconnect()


if __name__ == "__main__":
    main()
