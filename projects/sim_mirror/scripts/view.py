"""Open the wallerobot twin in the interactive MuJoCo viewer (no hardware, no COM port).

    .venv/Scripts/python.exe projects/sim_mirror/scripts/view.py                      # real rest pose, kinematic
    .venv/Scripts/python.exe projects/sim_mirror/scripts/view.py --pose home          # rest arms + neck `home` (neck_calibration.json)
    .venv/Scripts/python.exe projects/sim_mirror/scripts/view.py --pose zero          # all URDF joint angles 0
    .venv/Scripts/python.exe projects/sim_mirror/scripts/view.py --pose pose.json     # {"ticks": {...}} (real_reader.py --json) or {"q_deg": {...}}
    .venv/Scripts/python.exe projects/sim_mirror/scripts/view.py --physics            # run the simulation (gravity + position actuators)
    .venv/Scripts/python.exe projects/sim_mirror/scripts/view.py --snapshot out.png   # headless: 4-view render, no window

Kinematic mode (default): no physics, joint angle = actuator slider. Right panel -> "Control": one slider per joint
(radians, clamped to the URDF limits), the robot follows exactly the URDF kinematics. Left panel -> "Rendering" -> Camera
selects the neck_color / left_wrist / right_wrist cameras. Group toggles: 2 = visual meshes, 3 = collision hulls,
4 = jaw tip sites. Tab / Shift+Tab hide the panels; Backspace resets everything to joint angle 0.
The model is models/scene.xml (build_mjcf.py; check it against the URDF with check_mjcf.py).
"""
import argparse
import json
import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

HERE = Path(__file__).resolve().parent
PROJ = HERE.parent
SCENE = PROJ / "models/scene.xml"
REST = PROJ / "outputs/rest_pose_20261105.json"
sys.path.insert(0, str(HERE))
import calib  # noqa: E402

# robot forward = body -X, left arm = -Y; MuJoCo azimuth 0 = camera on -X looking along +X = in front of the robot
VIEWS = [("front", 0, -12), ("left side", 90, -10), ("front-left 3/4", 35, -22), ("top", 0, -88)]


def pose_q(spec: str) -> dict:
    """Joint angles (rad) for 'zero' | 'rest' | 'home' | path to a pose json."""
    if spec == "zero":
        return {}
    if spec in ("rest", "home"):
        q = calib.ticks_to_q(json.loads(REST.read_text())["ticks"])
        if spec == "home":
            home = json.loads(calib.NECK_CAL.read_text(encoding="utf-8"))["poses"]["home"]["target_ticks"]
            q.update(calib.ticks_to_q(home))
        return q
    data = json.loads(Path(spec).read_text(encoding="utf-8"))
    if "ticks" in data:
        return calib.ticks_to_q(data["ticks"])
    return {n: np.radians(v) for n, v in data["q_deg"].items()}


def apply(m, d, q):
    mujoco.mj_resetData(m, d)
    for n, v in q.items():
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
        if jid >= 0:
            v = float(np.clip(v, *m.jnt_range[jid]))   # rest J2 sits a little past its limit -> shown at the limit
            d.qpos[m.jnt_qposadr[jid]] = v
            aid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
            if aid >= 0:
                d.ctrl[aid] = v
    mujoco.mj_forward(m, d)


def snapshot(m, d, out: Path, size=(720, 720)):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    m.vis.global_.offwidth, m.vis.global_.offheight = max(size[1], 1280), max(size[0], 960)
    r = mujoco.Renderer(m, *size); cam = mujoco.MjvCamera()
    cam.lookat[:] = d.xpos[1:].mean(0); cam.distance = 1.15
    opt = mujoco.MjvOption()   # default: groups 0-2 visible -> visual meshes, no collision hulls (group 3)
    fig, axs = plt.subplots(1, len(VIEWS) + 1, figsize=(4.2 * (len(VIEWS) + 1), 4.6))
    for ax, (name, az, el) in zip(axs, VIEWS):
        cam.azimuth, cam.elevation = az, el
        r.update_scene(d, cam, opt); ax.imshow(r.render()); ax.set_title(name); ax.axis("off")
    r.close(); r = mujoco.Renderer(m, 480, 640)
    r.update_scene(d, "neck_color", opt); axs[-1].imshow(r.render()); axs[-1].set_title("camera neck_color"); axs[-1].axis("off")
    r.close(); plt.tight_layout(); plt.savefig(out, dpi=80); plt.close(fig)
    print("saved", out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pose", default="rest"); ap.add_argument("--physics", action="store_true")
    ap.add_argument("--collision", action="store_true", help="also show the collision hulls (group 3)")
    ap.add_argument("--snapshot", type=Path, help="render 4 views + neck camera to this PNG and exit")
    ap.add_argument("--scene", type=Path, default=SCENE)
    ap.add_argument("--seconds", type=float, help="kinematic viewer: close by itself after this long (smoke test)")
    a = ap.parse_args()
    m = mujoco.MjModel.from_xml_path(str(a.scene)); d = mujoco.MjData(m)
    q0 = pose_q(a.pose); apply(m, d, q0)
    if a.snapshot:
        return snapshot(m, d, a.snapshot)
    qadr = [m.jnt_qposadr[m.actuator_trnid[i, 0]] for i in range(m.nu)]
    print(f"{a.scene.name}: {m.njnt} joints, {m.nu} sliders, cameras {[m.camera(i).name for i in range(m.ncam)]}; "
          f"pose '{a.pose}', {'physics' if a.physics else 'kinematic (slider = joint angle)'} - close the window to quit")
    if a.physics:
        mujoco.viewer.launch(m, d)   # managed viewer: simulates, sliders are position targets
        return
    with mujoco.viewer.launch_passive(m, d) as v:
        v.opt.geomgroup[3] = int(a.collision)
        v.cam.lookat[:] = d.xpos[1:].mean(0); v.cam.distance, v.cam.azimuth, v.cam.elevation = 1.2, 35, -22
        t_end = time.time() + a.seconds if a.seconds else float("inf")
        while v.is_running() and time.time() < t_end:
            with v.lock():
                d.qpos[qadr] = d.ctrl        # kinematic: sliders drive the joints directly
                d.qvel[:] = 0
                mujoco.mj_forward(m, d)
            v.sync(); time.sleep(1 / 60)


if __name__ == "__main__":
    main()
