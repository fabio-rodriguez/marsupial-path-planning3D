"""Load cached plot data and show interactive 3D figures. No algorithms run."""

import os, pickle as pkl
import matplotlib.pyplot as plt
from plot_comparison import make_figure

SCENARIOS = ["S2", "S3", "S5"]

for sname in SCENARIOS:
    cache = f"scenarios/{sname}_plot_data.pkl"
    if not os.path.exists(cache):
        print(f"[{sname}] cache not found — run  python plot_comparison.py  first")
        continue
    with open(cache, "rb") as f:
        data = pkl.load(f)
    print(f"[{sname}] methods: {list(data['methods'].keys())}")
    make_figure(data)

plt.show()
