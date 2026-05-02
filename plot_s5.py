"""Visualise the RRT* tree on S5 (2-D top-down view)."""

import pickle as pkl, time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib import collections as mc
from scipy.spatial import ConvexHull

from run_benchmark import RRT_star, Graph, scenario_board_shape

with open("scenarios/S5.pkl", "rb") as f:
    data = pkl.load(f)

S          = data["S"]
T          = data["T"]
ground_obs = data["ground_obstacles"]
aerial_obs = data["aerial_obstacles"]
bshape     = scenario_board_shape(data)

print(f"Start : {S}")
print(f"Target: {T}")
print(f"Sampling box: x ± {bshape[0]:.0f},  y ± {bshape[1]:.0f}")
print()

# ----- run tree for 10s so it's fast to view -----
print("Running RRT* for 10s ...", flush=True)
t0 = time.time()
G = RRT_star(S[:2], T, ground_obs, aerial_obs,
             radius=10**6, k_length=26, n_iter=10**6, time_for_ending=10,
             board_shape=bshape)
print(f"Done in {time.time()-t0:.1f}s  |  vertices: {len(G.vertices)}  |  success: {G.success}")

# ----- plot -----
fig, ax = plt.subplots(figsize=(10, 12))

# sampling box
bx, by = bshape[0], bshape[1]
sampling_box = plt.Rectangle((-bx, -by), 2*bx, 2*by,
                              linewidth=2, edgecolor='orange', facecolor='none',
                              linestyle='--', label=f'Sampling region (±{bx:.0f}, ±{by:.0f})')
ax.add_patch(sampling_box)

# ground obstacles
for obs in ground_obs:
    pts = np.array([v[:2] for v in obs if v[-1] == 0])
    if len(pts) >= 3:
        try:
            h = ConvexHull(pts)
            poly = plt.Polygon(pts[h.vertices], closed=True,
                               facecolor='gray', edgecolor='black', alpha=0.5)
            ax.add_patch(poly)
        except Exception:
            ax.scatter(pts[:, 0], pts[:, 1], c='gray', s=10)

# RRT edges
lines = [(G.vertices[e[0]], G.vertices[e[1]]) for e in G.edges]
lc = mc.LineCollection(lines, colors='steelblue', linewidths=0.4, alpha=0.6)
ax.add_collection(lc)

# vertices
vx = [v[0] for v in G.vertices]
vy = [v[1] for v in G.vertices]
ax.scatter(vx, vy, c='steelblue', s=4, alpha=0.4, zorder=3)

# start and target
ax.scatter(*S[:2], c='green', s=120, zorder=5, label=f'Start {S[:2]}')
ax.scatter(*T[:2], c='red',   s=120, marker='*', zorder=5, label=f'Target {T[:2]}')

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_title(f'S5 — RRT* tree after 10s  (board ±{bx:.0f}, ±{by:.0f})')
ax.legend(loc='upper left')
ax.set_aspect('equal')
ax.autoscale()
ax.margins(0.05)
plt.tight_layout()
plt.savefig('s5_tree.png', dpi=150)
print("Saved: s5_tree.png")
plt.show()
