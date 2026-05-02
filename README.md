# Marsupial 3D Path Planning

A 3D path planner for a **marsupial robotic system** — a ground vehicle (UGV) carrying a tethered aerial vehicle (UAV). The planner finds collision-free paths for both vehicles from a start position **S** on the ground to an aerial target **T**, subject to a maximum tether length constraint.

## How it works

The problem is solved in two coupled sub-problems:

1. **Ground path** — the UGV navigates a 2D obstacle-free path on the ground from S to a *tether anchor point* (called the *top*), computed via a visibility graph and Dijkstra.
2. **Aerial tether** — from the anchor point, the UAV flies up to T along a catenary cable that must avoid 3D obstacles without exceeding `TETHER_LENGTH`.

The geometry-based **MASPA** planner slices the 3D space into vertical planes around T, samples candidate anchor points on each plane, runs a 2D visibility check, and evaluates the catenary for each surviving candidate. An **RRT\*** family of planners is also provided as a probabilistic alternative.

```
Ground plane          3D space
   S ──────► top ── ~ ~ ~ ► T
  (UGV path) (tether anchor)  (catenary)
```

---

## Project structure

```
marsupial-path-planning3D/
├── maspa_planning.py    # MASPA planner — main entry point
├── rrt_planning.py      # RRT* planners (single-target + sequential)
├── run_benchmark.py     # Benchmark comparing RRT*, Informed RRT*, Smart RRT*
├── cvisibility.py       # Constrained visibility computation (MASPA core)
├── cat2.py              # Catenary physics and tether collision detection
├── planners.py          # 2D visibility-graph path planning
├── tools.py             # Geometry utilities, Dijkstra, obstacle helpers
├── drawing.py           # 3D/2D visualisation
├── generate_scenarios.py# Scenario generators (S1–S5, random)
├── metrics.py           # Statistics over random experiment results
├── constants.py         # Global parameters
└── scenarios/           # Pickled scenario and result files
    ├── S1.pkl  S2.pkl  S3.pkl  S5.pkl   # Test scenarios
    ├── S1_maspa.pkl  S3_maspa.pkl       # MASPA solutions
    ├── S1_rrt.pkl   S3_rrt.pkl          # RRT* solutions
    └── random_scenarios.pkl             # 1 000 random instances
```

---

## Installation

Python 3.10+ is required. Create and activate a virtual environment, then install dependencies:

```bash
python -m venv .venv
source .venv/bin/activate        # macOS / Linux
# .venv\Scripts\activate         # Windows

pip install numpy scipy matplotlib pycatenary pyvisgraph trimesh
```

---

## Configuration — `constants.py`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MARSUPIAL_HEIGHT` | `3` m | Height of the UGV cylinder that houses the UAV at rest |
| `UAV_RADIUS` | `1` m | Safety radius around the UAV |
| `TETHER_LENGTH` | `50` m | Maximum tether length |
| `HTOP` | `2` m | Take-off height (`MARSUPIAL_HEIGHT − UAV_RADIUS`) |
| `EPSILON` | `1e-6` | Numerical precision guard |

---

## Running the MASPA planner

### Single target (S5 by default)

```bash
python maspa_planning.py
```

Internally calls `example()`, which loads `scenarios/S5.pkl`, runs both the visibility-graph planner (`path_planning_smpp`) and the brute-force variant (`path_planning_bf`), prints path lengths, and opens a 3D plot.

**Parameters inside `example()`:**

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `p` | `16` | Number of vertical planes sliced around T |
| `q` | `30` | Candidate anchor points sampled per plane |
| `k_length` | `26` | Catenary length samples between `d(top,T)` and `TETHER_LENGTH` |

Increasing `p` and `q` improves solution quality at the cost of computation time.

### Call from Python

```python
from maspa_planning import path_planning_smpp

ground_path, opt_info, elapsed, visibility = path_planning_smpp(
    S            = (12, 8, 0),          # UGV start (x, y, z=0)
    T            = (90, 130, 23),        # UAV target (x, y, z)
    ground_obs   = [...],               # list of obstacle vertex lists (ground)
    aerial_obs   = [...],               # list of obstacle vertex lists (aerial)
    p            = 16,                   # vertical planes
    q            = 30,                   # anchor candidates per plane
    k_length     = 26,                   # catenary length steps
    plot         = True,                 # open 3D plot when done
    visibility   = None,                 # pass previous cache to reuse
)

# opt_info = (top_point, {top: {length, tether}}, aerial_length)
# elapsed  = planning time in seconds (excluding catenary collision overhead)
```

The `visibility` cache can be reused across calls on the same environment (e.g. sequential targets), dramatically reducing repeated visibility checks.

### Sequential targets (S3 — two aerial targets)

```python
from maspa_planning import maspa_sequential

maspa_sequential(plot=True)
```

This loads `scenarios/S3.pkl`, plans to each target in sequence (the UGV start of target *i+1* is the anchor position of target *i*), saves the solution to `scenarios/S3_maspa.pkl`, and prints the total path length and planning time.

### Brute-force variant

`path_planning_bf` has the same signature as `path_planning_smpp` but skips the constrained-visibility filter — it evaluates the catenary for all `p×q` candidate anchor points. Slower, but useful as a reference for solution quality.

```python
from maspa_planning import path_planning_bf

ground_path, opt_info, elapsed, visibility = path_planning_bf(
    S, T, ground_obs, aerial_obs, p=16, q=30, k_length=26, plot=False
)
```

---

## Running the RRT\* planners

Three sampling-based planners are implemented in `rrt_planning.py`, all sharing the same stopping criterion: run until a solution is found **and** a time budget is exhausted.

| Planner | Strategy |
|---------|----------|
| `RRT_star` | Standard RRT\* — uniform random sampling |
| `Informed_RRT_star` | Restricts sampling to the prolate hyperellipsoid defined by the current best cost (Gammell et al., IROS 2014) |
| `Smart_RRT_star` | After a solution is found, biases 30 % of samples near the current best ground-path waypoints (Nasir et al., 2013) |

### Single target

```bash
python rrt_planning.py
```

Runs `example()`, which loads `scenarios/S2.pkl`, runs `RRT_star` for up to 20 seconds, plots the tree and best path, and saves the result to `scenarios/S1_rrt.pkl`.

**Key parameters inside `example()`:**

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `radius` | `10⁶` | Neighbourhood radius for nearest-node search (effectively ∞ → full RRT\*) |
| `k_length` | `26` | Catenary length steps (same as MASPA) |
| `time_for_ending` | `20` s | Wall-clock budget after first solution is found |

### Call from Python

```python
from rrt_planning import RRT_star, Informed_RRT_star, Smart_RRT_star, dijkstra

# Run planner
G = RRT_star(
    startpos       = (20, 40),       # UGV start (2-D, z is always 0)
    T              = (35, 8, 15.6),  # UAV target (3-D)
    ground_obs     = [...],
    aerial_obs     = [...],
    radius         = 10**6,          # neighbour search radius
    k_length       = 26,
    n_iter         = 10**6,          # max iterations (rarely the binding limit)
    time_for_ending= 60,             # seconds
)

if G.success:
    ground_path, path_length = dijkstra(G, T)
    print("Total cost:", G.distances[G.vex2idx[tuple(G.endpos)]])
    print("Improvement log:", G._improve_log)  # (elapsed_s, total_cost, aerial_length)
```

`G._improve_log` records every improvement as `(elapsed_seconds, total_cost, aerial_length)`, enabling post-hoc analysis at any time cutoff.

The same interface applies to `Informed_RRT_star` and `Smart_RRT_star` (they do not take `n_iter` or `G` arguments):

```python
G = Informed_RRT_star(startpos, T, ground_obs, aerial_obs, radius, k_length,
                      time_for_ending=60)

G = Smart_RRT_star(startpos, T, ground_obs, aerial_obs, radius, k_length,
                   time_for_ending=60, p_smart=0.3)  # p_smart: bias probability
```

### Sequential targets (S3)

```python
from rrt_planning import rrt_sequential

rrt_sequential(plotting=False)
```

Loads `scenarios/S3.pkl` and plans to each target in turn (20 s budget per target). Saves the solution to `scenarios/S3_rrt.pkl`.

---

## Benchmark

`run_benchmark.py` compares the three RRT\* variants across scenarios **S2**, **S3**, and **S5** over 20 independent runs each. Every run lasts 60 seconds; results are sampled at 20 s, 40 s, and 60 s from the improvement log.

```bash
python run_benchmark.py
```

The table printed at the end shows **mean path cost ± std (success rate)** for each algorithm × scenario × time-cutoff cell. For S3 (two sequential targets) the budget is split 30 s per target; cost entries reflect the total path cost only when both targets are solved.

---

## Generating scenarios

```python
from generate_scenarios import generate_S1, generate_S2, generate_S3
from generate_scenarios import get_random_scenarios

generate_S1("scenarios/S1.pkl")   # complex tower/building environment
generate_S2("scenarios/S2.pkl")   # mixed ground + aerial obstacles
generate_S3("scenarios/S3.pkl")   # two sequential aerial targets

# 1 000 random instances: 5 ground + 5 aerial obstacles each
get_random_scenarios(1000, ground_n=5, aerial_n=5, path="scenarios/random_scenarios.pkl")
```

Each scenario pickle contains:

```python
{
    "S": (x, y, 0),               # UGV start
    "T": (x, y, z),               # UAV target  (list of targets for S3)
    "ground_obstacles": [...],     # list of obstacle vertex arrays (z == 0 vertices on ground)
    "aerial_obstacles": [...],     # list of obstacle vertex arrays (floating boxes)
}
```

---

## Running random experiments (MASPA)

```python
from maspa_planning import run_random_experiments

run_random_experiments(n=1000)    # runs MASPA on scenarios/random_scenarios.pkl
```

Results are saved to `scenarios/random_results_16-30.pkl`. Metrics (mean/std of path length and planning time) can be printed with:

```bash
python metrics.py
```
