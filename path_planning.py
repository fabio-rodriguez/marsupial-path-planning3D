import pickle as pkl

from cvisibility import *
from drawing import *
from tools import *


def path_planning_smpp(S, T, ground_obs, aerial_obs, ground_vsgraph, p=5, q=5, k_length=10, k_collision=10):
    """
        Planning algorithm based on Dijkstra apprach to reach B with the aerial vehicle starting from A with the ground vehicle

        The visibility graph can be partially pre-computed
    """
        
    cvisible_points = get_cvisible_points(T, ground_obs, aerial_obs,  p, q, k_length, k_collision)

    exit()
    # checkpoint

    # Insert S and the catenary visible points into the visibility graph 
    ground_targets = []
    targets_min_tether = {}
    for p, l in cvis_points, min_tether_lengths:
        vgp = vg.Point(*p)
        ground_targets.append(vg.Point(*vgp))
        targets_min_tether[tuple(vgp)] = l 
        
    ground_vsgraph.update([vg.Point(*S[:2])] + ground_targets)

    # checkpoint

    # Search algorithms

    # checkpoint



if __name__ == "__main__":
    
    path = "scenarios/S1.pkl"

    with open(path, "rb") as f:
        s = pkl.loads(f.read())

    p=10 
    q=20 # should be even
    k_length=100
    k_collision=100
    path_planning_smpp(
        s["S"], 
        s["T"], 
        s["ground_obstacles"],
        s["aerial_obstacles"], 
        s["ground_vis_graph"],
        p, q, k_length, k_collision)

