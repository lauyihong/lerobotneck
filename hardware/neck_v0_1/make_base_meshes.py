"""Make the neck base visuals (body frame = upstream top_base_link, mm) from upstream XLeRobot CAD.

    .venv/Scripts/python.exe hardware/neck_v0_1/make_base_meshes.py      # needs cascadio (uv pip install cascadio)

User 2026-11-04/05: the real tower is the truss tower and the part housing J1 (ID 7) is candidate 4 in
guess_base_candidates.png = the top body of third_party/XLeRobot/hardware/step/Archiv/Neck.step ("Folder 136").
Neck.step shares its frame with Assembly_RGBD_Gimbal.step (identical tower bounds), which sits in top_base_link up to a
pure translation T_BODY_ASM (asm yaw motor ICP-matched onto upstream topbase2 = old head-pan motor, 0.65 mm median).

Writes
  base_body_lower.stl  upstream topbase1 cut below the truss tower (chassis plate + riser; the solid upstream tower removed)
  base_tower.stl       truss tower (Neck.step main body)
  base_j1_housing.stl  J1 housing (Neck.step top body)"""
import sys
import tempfile
from pathlib import Path

import numpy as np
import trimesh

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from build_urdf import HEAD_MESH, ROOT, TOPBASE_VIS  # noqa: E402

NECK_STEP = ROOT / "third_party/XLeRobot/hardware/step/Archiv/Neck.step"
T_BODY_ASM = trimesh.transformations.translation_matrix([0.110354, -0.006524, -0.716479])
CUT_Z = 0.1065  # m, top of the chassis riser = bottom of the truss tower (105.9 mm)


def main():
    import cascadio
    with tempfile.TemporaryDirectory() as td:
        glb = Path(td) / "neck.glb"
        cascadio.step_to_glb(str(NECK_STEP), str(glb))
        s = trimesh.load(glb)
        parts = {}
        for n in s.graph.nodes_geometry:
            M, g = s.graph[n]
            m = s.geometry[g].copy()
            m.apply_transform(T_BODY_ASM @ M)
            parts[g] = m
    housing, tower = parts["Folder 136"], parts["SOLID"]
    body = trimesh.load(HEAD_MESH / "topbase1.stl", force="mesh")
    body.apply_scale(0.001)
    body.apply_transform(TOPBASE_VIS["topbase1.stl"])
    keep = (body.vertices[body.faces][:, :, 2] < CUT_Z).all(axis=1)
    body = body.submesh([np.nonzero(keep)[0]], append=True)
    for name, m in [("base_body_lower.stl", body), ("base_tower.stl", tower), ("base_j1_housing.stl", housing)]:
        m = m.copy()
        m.apply_scale(1000)
        m.export(HERE / name)
        print(name, len(m.faces), "faces, bounds mm", np.round(m.bounds, 1).tolist())


if __name__ == "__main__":
    main()
