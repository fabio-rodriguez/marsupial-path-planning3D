"""Quick S5-only benchmark: 3 algorithms × 20 runs × 60 s."""

import contextlib, os, pickle as pkl
import numpy as np
from run_benchmark import RRT_star, Informed_RRT_star, Smart_RRT_star, dijkstra, scenario_board_shape

N_RUNS      = 1
TIME_BUDGET = 60
k_length    = 26
radius      = 10**6

with open("scenarios/S5.pkl", "rb") as f:
    data = pkl.load(f)

S          = data["S"]
T          = data["T"]
ground_obs = data["ground_obstacles"]
aerial_obs = data["aerial_obstacles"]
bshape     = scenario_board_shape(data)


def best_cost_at(log, t):
    best = None
    for (elapsed, cost, _) in log:
        if elapsed <= t:
            best = cost
    return best


def run_single(algo_key):
    with open(os.devnull, "w") as dev, contextlib.redirect_stdout(dev):
        if algo_key == "rrt":
            G = RRT_star(S[:2], T, ground_obs, aerial_obs,
                         radius, k_length, n_iter=10**6, time_for_ending=TIME_BUDGET,
                         board_shape=bshape)
        elif algo_key == "informed":
            G = Informed_RRT_star(S[:2], T, ground_obs, aerial_obs,
                                  radius, k_length, time_for_ending=TIME_BUDGET,
                                  board_shape=bshape)
        else:
            G = Smart_RRT_star(S[:2], T, ground_obs, aerial_obs,
                               radius, k_length, time_for_ending=TIME_BUDGET,
                               board_shape=bshape)
    return G._improve_log, G.success


algo_defs = [
    ("RRT*",          "rrt"),
    ("Informed RRT*", "informed"),
    ("Smart RRT*",    "smart"),
]

print(f"\nS5  —  {N_RUNS} runs × {TIME_BUDGET}s each")
print(f"Start: {S}   Target: {T}\n")

COL = 22
hdr = f"{'Algorithm':<16}  {'@20s':>{COL}}  {'@40s':>{COL}}  {'@60s':>{COL}}  {'solved':>8}"
bar = "-" * len(hdr)
print(bar)
print(hdr)
print(bar)

for aname, akey in algo_defs:
    costs_20, costs_40, costs_60 = [], [], []
    solved = 0

    for i in range(N_RUNS):
        print(f"  [{aname}] run {i+1:2d}/{N_RUNS}", end="\r", flush=True)
        log, success = run_single(akey)
        if success:
            solved += 1
        costs_20.append(best_cost_at(log, 20))
        costs_40.append(best_cost_at(log, 40))
        costs_60.append(best_cost_at(log, 60))

    print(" " * 40, end="\r")  # clear progress line

    def fmt(costs):
        valid = [c for c in costs if c is not None]
        sr = 100 * len(valid) / N_RUNS
        if not valid:
            return f"{'N/A (0%)':>{COL}}"
        return f"{np.mean(valid):6.2f} ± {np.std(valid):5.2f}  ({sr:.0f}%)"

    print(f"{aname:<16}  {fmt(costs_20)}  {fmt(costs_40)}  {fmt(costs_60)}  {solved:>5}/{N_RUNS}")

print(bar)
