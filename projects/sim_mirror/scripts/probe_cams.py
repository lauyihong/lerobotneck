"""Grab one frame from every OpenCV camera index (MSMF) -> outputs/cam_probe.png, to identify the wrist cams by eye.

    .venv/Scripts/python.exe projects/sim_mirror/scripts/probe_cams.py [--max 5]

No motors are touched. Run it while nothing else uses the cameras (the D435i also shows up as an OpenCV index; opening
it here while pyrealsense streams would freeze the RealSense colour stream, so don't run the mirror at the same time).
"""
import argparse
import time
from pathlib import Path

import cv2
import numpy as np

OUT = Path(__file__).resolve().parents[1] / "outputs"


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--max", type=int, default=5); a = ap.parse_args()
    tiles = []
    for i in range(a.max):
        cap = cv2.VideoCapture(i, cv2.CAP_MSMF)
        f = None
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            t0 = time.time()
            while time.time() - t0 < 3:
                ok, fr = cap.read()
                if ok and fr is not None and fr.mean() > 1:
                    f = fr
                    for _ in range(5):          # let exposure settle
                        ok, fr = cap.read()
                        f = fr if ok else f
                    break
        cap.release()
        info = "no frame" if f is None else f"{f.shape[1]}x{f.shape[0]} mean {f.mean():.0f}"
        print(f"index {i}: {info}")
        t = np.full((240, 320, 3), 40, np.uint8) if f is None else cv2.resize(f, (320, 240))
        cv2.putText(t, f"index {i}: {info}", (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        tiles.append(t)
    while len(tiles) % 3:
        tiles.append(np.zeros_like(tiles[0]))
    grid = np.vstack([np.hstack(tiles[k:k + 3]) for k in range(0, len(tiles), 3)])
    out = OUT / "cam_probe.png"; cv2.imwrite(str(out), grid); print("saved", out)


if __name__ == "__main__":
    main()
