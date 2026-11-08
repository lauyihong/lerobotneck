"""MuJoCo twin of the real wallerobot (dual SO101 + neck v0.1), plugin-like interface.

    from sim_robot import SimWallerobot
    sim = SimWallerobot(); sim.connect()
    sim.set_state_ticks(real_ticks)          # teleport to a real encoder read (mirror)
    obs = sim.get_observation()              # {"<joint>.pos": rad, ..., "neck_color": HxWx3, ...}
    sim.send_action({"neck_j1.pos": 0.1})    # position targets (rad), physics via step()

Units: radians (URDF joint angles). Real ticks <-> rad via calib.py (neck_calibration.json + arm_calibration.json).
"""
import sys
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import calib  # noqa: E402

SCENE = HERE.parent / "models/scene.xml"
CAMERAS = ("neck_color", "left_wrist", "right_wrist")


class SimWallerobot:
    def __init__(self, scene: Path = SCENE, cameras=CAMERAS, size=(480, 640), substeps: int = 17):
        self.scene, self.cam_names, self.size, self.substeps = scene, cameras, size, substeps
        self.m = self.d = self.renderer = None

    # ---- lifecycle --------------------------------------------------------------------------
    def connect(self, calibrate: bool = True) -> None:
        self.m = mujoco.MjModel.from_xml_path(str(self.scene)); self.d = mujoco.MjData(self.m)
        self.joints = [self.m.joint(i).name for i in range(self.m.njnt)]
        self.qadr = {n: self.m.jnt_qposadr[self.m.joint(n).id] for n in self.joints}
        self.act = {self.m.actuator(i).name: i for i in range(self.m.nu)}
        self.renderer = mujoco.Renderer(self.m, *self.size)
        mujoco.mj_forward(self.m, self.d)

    def disconnect(self) -> None:
        if self.renderer is not None:
            self.renderer.close()
        self.renderer = None

    @property
    def action_features(self) -> dict:
        return {f"{n}.pos": float for n in self.joints}

    # ---- state ------------------------------------------------------------------------------
    def set_state(self, q: dict) -> None:
        """Teleport (rad) and hold there: qpos = ctrl = q, velocities zero."""
        for n, v in q.items():
            n = n.removesuffix(".pos")
            if n in self.qadr:
                self.d.qpos[self.qadr[n]] = v
                if n in self.act:
                    self.d.ctrl[self.act[n]] = v
        self.d.qvel[:] = 0
        mujoco.mj_forward(self.m, self.d)

    def set_state_ticks(self, ticks: dict) -> None:
        self.set_state(calib.ticks_to_q(ticks))

    def send_action(self, action: dict) -> dict:
        sent = {}
        for k, v in action.items():
            n = k.removesuffix(".pos")
            if n in self.act:
                lo, hi = self.m.actuator_ctrlrange[self.act[n]]
                self.d.ctrl[self.act[n]] = sent[k] = float(np.clip(v, lo, hi))
        return sent

    def step(self, n: int | None = None) -> None:
        for _ in range(n or self.substeps):
            mujoco.mj_step(self.m, self.d)

    def qpos(self) -> dict:
        return {n: float(self.d.qpos[a]) for n, a in self.qadr.items()}

    # ---- observation ------------------------------------------------------------------------
    def render(self, cam: str) -> np.ndarray:
        self.renderer.update_scene(self.d, camera=cam)
        return self.renderer.render()

    def get_observation(self) -> dict:
        obs = {f"{n}.pos": v for n, v in self.qpos().items()}
        for c in self.cam_names:
            obs[c] = self.render(c)
        return obs
