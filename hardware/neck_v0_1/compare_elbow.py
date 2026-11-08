"""Photo vs model elbow side view. usage: compare_elbow.py out.png q1,..,q6 (deg; e.g. from tools/calib_neck.py show)"""
import sys, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
from pathlib import Path
HERE = Path(__file__).resolve().parent; sys.path.insert(0, str(HERE)); from render_mj import render
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei"]
U = HERE / "photos"
ph = plt.imread(U / "home_side_20261104.jpg")[560:1300, 420:1420]
urdf = HERE / "urdf/wallerobot_neck.urdf"
q = [float(v) for v in sys.argv[2].split(",")]
v = ["side:90:5:0.36:neck_upper_arm"]
a = render(urdf, q, v, (700, 700))[0][1]
fig, axs = plt.subplots(1, 2, figsize=(13, 6.5))
for ax, im, t in zip(axs, [ph, a], ["照片：home 姿势，紫色块贴在绿色上臂上", f"模型（J3 零位修正后）：现在的编码器读数\nJ3 = {q[2]:+.1f}°"]):
    ax.imshow(im); ax.axis("off"); ax.set_title(t, fontsize=13)
fig.suptitle("肘部对比（侧面）— J3 零位修正后", fontsize=15)
plt.tight_layout(rect=(0, 0, 1, 0.93)); plt.savefig(sys.argv[1], dpi=80); print("saved", sys.argv[1])
