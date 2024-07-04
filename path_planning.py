import pickle as pkl
import time

from cvisibility import *
from drawing import *
from tools import *


def path_planning_smpp(S, T, ground_obs, aerial_obs, p=5, q=5, k_length=10, k_collision=10, plot=False):
    """
        Planning algorithm based on Dijkstra apprach to reach B with the aerial vehicle starting from A with the ground vehicle

        The visibility graph can be partially pre-computed
    """
    tt = 0
    t = time.time()
    cvisible_tops, t_error = get_cvisible_tops(T, ground_obs, aerial_obs,  p, q, k_length, k_collision)
    tt += time.time()-t-t_error
    print("cvisible_tops", tt, "t error", t_error)

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
    t = time.time()
    Xopt, weigths, previous, t_error = upd_dijkstra_algorithm(S, ground_points, ground_obs_vertices, ground_obs_proj)
    tt += time.time()-t-t_error
    print("upd_dijkstra_algorithm", tt, "t error", t_error)

    # CHECKPOINT # plot_dijkstra_graph(S, previous, ground_obs_proj)
    if Xopt == None:
        minL = sys.maxsize
        min_ctop_d = None
        min_ctop = None    
        for ctop, v in cvisible_tops.items():
            w = weigths[tuple(ctop[:2])]
            if w < minL:
                min_ctop_d = {ctop: v}
                min_ctop = ctop
                minL = w
    else:
        minL = weigths[Xopt[:2]]
        min_ctop = (*Xopt, 0)
        min_ctop_d = {min_ctop: ground_points[Xopt]}

    ground_path = build_path_to_goal(tuple(min_ctop[:2]), previous)
    print("total time:", tt)
    print("gpath", path_length(ground_path), "apath", min_ctop_d[min_ctop]["length"])
    print("paths sum", path_length(ground_path) + min_ctop_d[min_ctop]["length"])

    # CHECKPOINT # 
    if plot:
        plot_optimal_solution(S, T, min_ctop_d, ground_path, ground_obs, aerial_obs)

    return path_length(ground_path), min_ctop_d[min_ctop]["length"], tt
    

def build_path_to_goal(point, previous):
    path = [point]
    while previous[path[-1]] != None:
        path.append(previous[path[-1]])
    return path


def path_length(path):
    length = 0
    for i in range(len(path)-1):
        length += euclidian_distance_lists(path[i], path[i+1])
    return length


def example():
    
    path = "scenarios/S1.pkl"

    with open(path, "rb") as f:
        s = pkl.loads(f.read())

    p=18
    q=80
    
    k_length=20
    k_collision=50
    path_planning_smpp(
        s["S"], 
        s["T"], 
        s["ground_obstacles"],
        s["aerial_obstacles"], 
        p, q, k_length, k_collision, True)


def run_random_experiments():
    
    path = "scenarios/random_scenarios.pkl"
    path_output = "scenarios/random_results_extended.pkl"

    with open(path, "rb") as f:
        s = pkl.loads(f.read())

    p = [180]
    q = [100]

    k_length= 20 
    k_collision = 50

    results = []
    for si in s:
        r = {}
        for pi in p:    
            for qi in q:
                S = si["S"] + np.random.rand(3) # Avoid numerical errors
                T = si["T"] + np.random.rand(3) # Avoid numerical errors
                
                try:
                    gl, al, tt = path_planning_smpp(
                        tuple(S), tuple(T), 
                        si["ground_obstacles"],
                        si["aerial_obstacles"], 
                        pi, qi, k_length, k_collision)

                    r[(pi,qi)] = {"gp_length": gl, "ap_length": al, "tt": tt}
                except:
                    continue

        results.append(r)

        with open(path_output, "wb") as f:
            f.write(pkl.dumps(results))

    with open(path_output, "rb") as f:
            r = pkl.loads(f.read())
    print(len(r))
    print(r)



if __name__ == "__main__":
    
    # example()

    run_random_experiments()

    