import pickle as pkl

from cvisibility import *
from drawing import *
from tools import *


def path_planning_smpp(S, T, ground_obs, aerial_obs, p=5, q=5, k_length=10, k_collision=10):
    """
        Planning algorithm based on Dijkstra apprach to reach B with the aerial vehicle starting from A with the ground vehicle

        The visibility graph can be partially pre-computed
    """

    cvisible_tops = get_cvisible_tops(T, ground_obs, aerial_obs,  p, q, k_length, k_collision)
    
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

    ground_obs_proj, ground_obs_vertices = get_obstacles_proj_vertices(ground_obs)
    
    ground_points = {k[:2]:v for k,v in cvisible_tops.items()}
    weigths, previous = upd_dijkstra_algorithm(S, ground_points, ground_obs_vertices, ground_obs_proj)

    # CHECKPOINT # plot_dijkstra_graph(S, previous, ground_obs_proj)

    minL = sys.maxsize
    min_ctop_d = None
    min_ctop = None    
    for ctop, v in cvisible_tops.items():
        w = weigths[tuple(ctop[:2])]
        if w < minL:
            min_ctop_d = {ctop: v}
            min_ctop = ctop
            minL = w

    ground_path = build_path_to_goal(tuple(min_ctop[:2]), previous)

    print(ground_path)
    # CHECKPOINT # 
    plot_optimal_solution(S, T, min_ctop_d, ground_path, ground_obs, aerial_obs)


def build_path_to_goal(point, previous):
    path = [point]
    while previous[path[-1]] != None:
        path.append(previous[path[-1]])
    return path


if __name__ == "__main__":
    
    path = "scenarios/S1.pkl"

    with open(path, "rb") as f:
        s = pkl.loads(f.read())

    p=3
    q=4 # should be even
    k_length=100
    k_collision=100
    path_planning_smpp(
        s["S"], 
        s["T"], 
        s["ground_obstacles"],
        s["aerial_obstacles"], 
        p, q, k_length, k_collision)

