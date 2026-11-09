"""Rebuild the whole robot description from hardware/wallerobot_v0_1/robot.yaml in one go.

    python hardware/build_all.py                                    # default config -> the committed model files
    python hardware/build_all.py --config my.yaml --out build/my    # a variant into its own folder
    python hardware/build_all.py --check                            # rebuild, then fail if a committed file changed

Steps (default config):
  1. neck URDF            hardware/neck_v0_1/urdf/wallerobot_neck.urdf
  2. full URDF            hardware/wallerobot_v0_1/urdf/wallerobot_static.urdf (+ meshes/, convex hulls)
  3. MJCF                 projects/sim_mirror/models/wallerobot_static.xml + scene.xml
  4. joint table          hardware/wallerobot_v0_1/JOINTS.md (+ docs/robot_design.md section 4 if that doc exists)
  5. web model            projects/web_sim/web/public/model/ (needs both arms; skipped with --no-web)
  6. LeRobotNeck web model hardware/variants/lerobotneck.yaml -> build/lerobotneck -> projects/web_sim/web/public/model_neck/
                          (the showcase website; skipped with --no-web)

A variant (--out) gets steps 2-3 only: <out>/urdf/wallerobot_static.urdf and <out>/mjcf/scene.xml. The joint table and
the web export describe the reference robot and are not built for variants.
Needs: numpy, trimesh, mujoco, pyyaml (requirements-model.txt).
"""
import argparse
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for p in ("hardware", "hardware/neck_v0_1", "hardware/wallerobot_v0_1", "projects/sim_mirror/scripts"):
    sys.path.insert(0, str(ROOT / p))
import robot_config  # noqa: E402

GENERATED = ["hardware/neck_v0_1/urdf", "hardware/wallerobot_v0_1/urdf", "hardware/wallerobot_v0_1/JOINTS.md",
             "projects/sim_mirror/models", "projects/web_sim/web/public/model",
             "projects/web_sim/web/public/model_neck"]


def run(script, *args):
    print(f"\n== {script} {' '.join(args)}", flush=True)
    subprocess.run([sys.executable, str(ROOT / script), *args], check=True, cwd=ROOT)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", type=Path, help="override file merged over hardware/wallerobot_v0_1/robot.yaml")
    ap.add_argument("--out", type=Path, help="build a variant into this folder instead of the committed locations")
    ap.add_argument("--no-web", action="store_true", help="skip the web_sim export")
    ap.add_argument("--check", action="store_true", help="fail if the rebuild changed any committed generated file")
    a = ap.parse_args()
    if a.config:
        a.config = a.config.resolve()
        os.environ["WALLEROBOT_CONFIG"] = str(a.config)   # child scripts (calib.py, export) see the same config
    robot_config.use(a.config)
    cfg = robot_config.load()

    if a.out:
        import build_full_urdf
        import build_mjcf
        out = a.out.resolve()
        urdf = build_full_urdf.build(out / "urdf", cfg)
        robot, scene, pairs = build_mjcf.build_all(urdf, out / "mjcf", cfg)
        import mujoco
        m = mujoco.MjModel.from_xml_path(str(scene))
        print(f"\nvariant OK: {m.njnt} joints, {m.nu} actuators, {m.ncam} cameras, excludes {pairs}\n  {urdf}\n  {scene}")
        return

    run("hardware/neck_v0_1/build_urdf.py")
    run("hardware/wallerobot_v0_1/build_full_urdf.py")
    run("projects/sim_mirror/scripts/build_mjcf.py")
    run("hardware/wallerobot_v0_1/joint_table.py", "--write")
    if not a.no_web:
        if robot_config.enabled_arms(cfg) == ["left", "right"]:
            run("projects/web_sim/scripts/export_assets.py")
        else:
            print("\n== web export skipped (the web app needs both arms)")
        if not a.config:
            run("hardware/build_all.py", "--config", "hardware/variants/lerobotneck.yaml", "--out", "build/lerobotneck")
            run("projects/web_sim/scripts/export_assets.py", "--neck", "build/lerobotneck/mjcf")
    if a.check:
        r = subprocess.run(["git", "status", "--porcelain", "--", *GENERATED], cwd=ROOT, capture_output=True, text=True)
        if r.stdout.strip():
            print("\nCHECK FAILED - generated files differ from the committed ones:\n" + r.stdout)
            sys.exit(1)
        print("\nCHECK OK - generated files match the committed ones")


if __name__ == "__main__":
    main()
