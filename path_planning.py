import pickle as pkl

from cvisibility import *
from drawing import *
from tools import *


def path_planning_smpp(S, T, ground_obs, aerial_obs, ground_vsgraph, p=5, q=5, k_length=10, k_collision=10):
    """
        Planning algorithm based on Dijkstra apprach to reach B with the aerial vehicle starting from A with the ground vehicle

        The visibility graph can be partially pre-computed
    """

    cvisible_points = get_cvisible_tops(T, ground_obs, aerial_obs,  p, q, k_length, k_collision)
    
    # CHECKPOINT # plot3D([{"point":S, "label":"S", "color":"k"}, {"point":T, "label":"T", "color":"r"}], ground_obs + aerial_obs, tops = cvisible_points)

    ###### UNCOMMENT UNDER FLOATING POINT ERRORS #######

    # Insert S into the visibility graph 
    # ground_vsgraph.update([vg.Point(*S[:2])])

    # Insert cvisibile tops into the visibility graph 
    # targets_min_tether = {}
    # for top, values in cvisible_points.items():
    #     vgtop = vg.Point(*top[:2]) # In reallity this point is translated within the ground a UAV_RADIUS to the opposite direction of T    
    #     ground_vsgraph.update([vg.Point(*S[:2])])
    #     targets_min_tether[vgtop] = values

    # ground_vsgraph.update(targets_min_tether.keys())

    # CHECKPOINT # plot_visibility_graph(ground_vsgraph, ground_obs, path_to_image=None)

    ####################################################

    weigths, previous = upd_dijkstra_algorithm(S, cvisible_points, ground_vsgraph.visgraph.get_points(), ground_obs)

    # checkpoint

    # Search algorithms

    # checkpoint



if __name__ == "__main__":
    
    path = "scenarios/S1.pkl"

    with open(path, "rb") as f:
        s = pkl.loads(f.read())

    p=5
    q=10 # should be even
    k_length=100
    k_collision=100
    path_planning_smpp(
        s["S"], 
        s["T"], 
        s["ground_obstacles"],
        s["aerial_obstacles"], 
        s["ground_vis_graph"],
        p, q, k_length, k_collision)

