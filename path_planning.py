import pickle as pkl

from tools import *
from cvisibility import *


def path_planning_smpp(S, T, aerial_obs, ground_obs, ground_vsgraph, p=5, q=5, k_length=10, k_collision=10):
    """
        Planning algorithm based on Dijkstra apprach to reach B with the aerial vehicle starting from A with the ground vehicle

        The visibility graph can be partially pre-computed
    """

    # Checks which of the obstacles lie within the 3D visibility area    
    gobs_in_C, aobs_in_C = obstacles_within_circle(T, ground_obs, aerial_obs)

    plot3D([{"point":S, "label":"S", "color":"k"}, {"point":T, "label":"T", "color":"r"}], ground_obs + aerial_obs)

    # checkpoint

    # 2D visibility query funcion
    cvis_points, min_tether_lengths = get_c_visible_points(T, gobs_in_C, aobs_in_C, p, q, k_length, k_collision)

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

    p=5 
    q=5
    k_length=10
    k_collision=10
    path_planning_smpp(
        s["S"], 
        s["T"], 
        s["aerial_obstacles"], 
        s["ground_obstacles"], 
        s["ground_vis_graph"],
        p, q, k_length, k_collision)

