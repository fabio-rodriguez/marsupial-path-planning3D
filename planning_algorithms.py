import json
import math
import matplotlib.pyplot as plt
import numpy as np
import pickle as pkl
import pyvisgraph as vg
import random
import time

from itertools import combinations
from pyvisgraph.graph import Edge
from pyvisgraph.visible_vertices import visible_vertices
from scipy.spatial import ConvexHull
from sklearn.preprocessing import normalize

from constants import *
from decision_problem.Tests3D import main as dp3D
from tools import *
from obstacles.intersection3D import *
from obstacles.minkowski import minkowski_distance


PLANES_2D = {}

# NOTE: 
# 1) For algorithms, we are assumming that Minkowski sum is already performed in obstacles, so collision-free path can be tangent to them 
def visgraph_approach(S, T, aerial_obs, ground_obs, vsgraph, planes_number=5, gpoints_number=5, k_length=10, k_collision=10, plotting=False):

    """
        Planning algorithm based on Dijkstra apprach to reach B with the aerial vehicle starting from A with the ground vehicle

        The graph can be partially pre-computed

        Notes:        
        1- Se asume que el dron terrestre con el aereo encima y los obstaculos terrestres tienen la misma altura
        2- Se asume que los obstaculos aereos no tocan al dron terrestre con el dron aereo encima
        3- Se asume que si un dron toca a un obstaculo en un punto no ocurre colision
        4- El plano tangente para el decision problem debeconsiderar los obstaculos terrestres  y los obstaculos aereos
        5- Los vertices a considerar en el C-VIsibility son:
            a) los vertices que pueden ver la circunferencia de B
            b) los puntos vertices dentro de B
    """
    graph = vsgraph["graph"]
    graph_vert = graph.visgraph.get_points()
    ground_obs_in_C, aerial_obs_in_C = obstacles_within_circle(T, ground_obs, aerial_obs)


    # Implement decision problem: return valid Length    
    dp_function = lambda x: dp_query(x, T, ground_obs_in_C, aerial_obs_in_C, k_length, k_collision)    
    
    t_init = time.time()

    t = time.time()
    ground_points, time_mink = generate_ground_points(T, ground_obs_in_C, dp_function, planes_number, gpoints_number)
    print("Old version", time.time()-t, len(ground_points))

    t = time.time()
    ground_points = generate_ground_points_heuristic(T, ground_obs, aerial_obs, planes_number=planes_number, gpoints_number=gpoints_number)
    print("New version", time.time()-t, len(ground_points))
    

    ## Connect S to the graph
    starting_point = vg.Point(*S[:2])

    #Target structures for searching
    target_points = []
    targets_length = {}
    for gp in ground_points:
        gp2D = gp["take-off"][:2]
        target_points.append(vg.Point(*gp2D))
        targets_length[tuple(gp2D)] = gp["length"] 
    

    # e = Edge(vg.Point(0,1), vg.Point(5.5,8))
    # g.visgraph[vg.Point(0,1)].add(e)
    
    graph.update([starting_point] + target_points)
    graph_vert += [starting_point] + target_points

    t_first_step = time.time() - t_init - time_mink

    ## Dijkstra algorithm 
    t_init = time.time()
    goal, previous_nodes, shortest_path = dijkstra_algorithm(graph, starting_point, targets_length)
    dijkstra_time = time.time() - t_init
    
    ## A* algorithm 
    t_init = time.time()
    goal_a_star, previous_nodes_a_star, shortest_path_a_star = A_star_algorithm(graph, starting_point, targets_length, vg.Point(*T[:2]))
    astar_time = time.time() - t_init
    
    ## RRT* algorithm 
    # t_init = time.time()
    # G = RRT_star(A[:2], B[:2], hulls, 1*10**3, f_vis_B, 50)
    # print("RRT* time:", time.time() - t_init)
    # shortest_path_RRT = None
    # print("RRT* length:", shortest_path_RRT)
    # print()
    

    # print("first step time:", t_first_step)
    # print()

    # print("Dijkstra time:", dijkstra_time)
    # print("Dijkstra length:", shortest_path[goal])
    # print()
    
    # print("A* time:", astar_time) 
    # print("A* length:", shortest_path_a_star[goal_a_star])
    # print()


    if plotting:

        ground_hulls = [value[-1] for value in ground_obs.values()]    

        
        # Plot init point
        graph_vert = graph.visgraph.get_points()
        for point in graph_vert:
            for visible in graph.find_visible(point):
                plt.plot([point.x, visible.x], [point.y, visible.y], "--b")

        plot_2dhulls(ground_hulls)

        plt.plot([S[0]], [S[1]], "ok")
        plt.plot([T[0]], [T[1]], "or")
        
        for r in target_points:
            plt.plot([r.x, r.x], [r.y, r.y], ".r")

        ## Plot shortest path Dijkstra
        last_node = goal
        while True:
            if last_node == starting_point:
                break
            plt.plot([last_node.x, previous_nodes[last_node].x], [last_node.y, previous_nodes[last_node].y], ".-r")
            last_node = previous_nodes[last_node]

        ## Plot shortest path A*
        last_node = goal_a_star
        while True:
            if last_node == starting_point:
                break
            plt.plot([last_node.x, previous_nodes_a_star[last_node].x], [last_node.y, previous_nodes_a_star[last_node].y], ".-g")
            last_node = previous_nodes_a_star[last_node]

        # plt.show()
        plt.close()

        # plot 3D scenario
        fig = plt.figure()
        ax = plt.axes(projection='3d')
            
        # print("points_lists", points_lists)
        
        for index in aerial_obs:
            _, hull = aerial_obs[index]            
            for vi in hull.points[hull.vertices]:
                for vj in hull.points[hull.vertices]:
                    ax.plot([vi[0], vj[0]], [vi[1], vj[1]], [vi[2], vj[2]], '-k')

        for index in ground_obs:
            _, hull, _, _ = ground_obs[index]
            for vi in hull.points[hull.vertices]:
                for vj in hull.points[hull.vertices]:

                    ax.plot([vi[0], vj[0]], [vi[1], vj[1]], [vi[2], vj[2]], '-r')
        
        ax.plot([S[0]], [S[1]], [0], "ob")
        ax.plot([T[0]], [T[1]], [0], "ob")
        
        for r in target_points:
            plt.plot([r.x, r.x], [r.y, r.y], ".r")

        ## Plot shortest path Dijkstra
        last_node = goal
        while True:
            if last_node == starting_point:
                break
            plt.plot([last_node.x, previous_nodes[last_node].x], [last_node.y, previous_nodes[last_node].y], [0, 0], ".-y")
            last_node = previous_nodes[last_node]

        ## Plot shortest path A*
        last_node = goal_a_star
        while True:
            if last_node == starting_point:
                break
            plt.plot([last_node.x, previous_nodes_a_star[last_node].x], [last_node.y, previous_nodes_a_star[last_node].y], [0,0], ".--g")
            last_node = previous_nodes_a_star[last_node]

            # X, Y, Z = zip(*hull.points[hull.vertices])


        plt.show()
            # p, lbl = pl["points"], pl["label"]

            # try:
            #     X, Y, Z = zip(*p)
            # except:
            #     X, Y, Z = zip(*p.points[p.vertices])
            # # ax.contour3D(X, Y, Z, 50, cmap='binary')
            # if lbl:
            #     ax.scatter3D(X, Y, Z, label=lbl, color="black", s=40)
            # else:
            #     ax.scatter3D(X, Y, Z)

    try:
        return t_first_step, dijkstra_time, shortest_path[goal], astar_time, shortest_path_a_star[goal_a_star]
    except:
        print("**NO SOLUTION")
        return None, None, None, None, None

    for i in range(10):
        s=scenarios[i]
        for j in range(len(s["endpoints"])):
            endpoints = s["endpoints"][j]
            a = list(endpoints["A"])
            b = list(endpoints["B"])
            a_obs, g_obs = s["obstacles"]["aerial_obs"], s["obstacles"]["ground_obs"] 
            v_graph = s["visibility_graph"]

def dp_query(ground_point, target, ground_obs, aerial_obs, k_length, k_collision):
    
    T_projection = np.array(list(target[:2])+[0])
    v1 = target-ground_point
    v2 = T_projection-ground_point
    normal_vector = normalize(np.cross(v1, v2))

    if tuple(normal_vector) in PLANES_2D:
        transformed_obstacles = PLANES_2D[tuple(normal_vector)]
        time_mink = 0
    else:
        # Intervals can affect the computation of the intersections
        # intervals = plane_intervals(target, T_projection, ground_point) # Get the interval to know if a point is inside the vertical plane
        obstacles = []
        # for _,hull,_,_ in ground_obs.values():
        for _,hull,_ in ground_obs.values():
            
            edges = []    
            for simplex in hull.simplices:
                s = hull.points[simplex]
                edges += [(s[i], s[i+1]) for i in range(len(s)-1)] + [(s[-1], s[0])]
            
            intersections = plane_edges_collision_points_normal(ground_point, normal_vector, edges) # find intersections between obstacles and the vertical plane

            if len(intersections):
                obstacles.append(intersections)
            
        for _, hull in aerial_obs.values():
            
            edges = []
            for simplex in hull.simplices:
                s = hull.points[simplex]
                edges += [(s[i], s[i+1]) for i in range(len(s)-1)] + [(s[-1], s[0])]
            
            intersections = plane_edges_collision_points_normal(ground_point, normal_vector, edges) # find intersections between obstacles and the vertical plane

            if len(intersections):
                obstacles.append(intersections)

        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')
        # for oi in obstacles:
        #     X, Y, Z  = zip(*oi)
        #     plt.plot(X, Y, Z, '.k') 

        transformed_obstacles = obstacles
        time_mink = 0
        # t_init = time.time()
        # transformed_obstacles = minkowski_transform_2D(obstacles) if len(obstacles) else []
        # time_mink = time.time()-t_init

        PLANES_2D[tuple(normal_vector)] = transformed_obstacles 

        # for oi in transformed_obstacles:
        #     X, Y, Z  = zip(*oi)
        #     plt.plot(X, Y, Z, '.g') 

        # plt.plot([ground_point[0], target[0], target[0]], [ground_point[1], target[1], target[1]], [ground_point[2], target[2], 0], 'or')
        # plt.show()

    result = dp3D(ground_point, target, CABLE_LENGTH, UAV_RADIO, UGV_RADIO, UGV_HEIGHT, transformed_obstacles, k_length, k_collision), time_mink    
    return result


def minkowski_transform_2D(obstacles):

    p1 = np.round(obstacles[0][0], 5)
    p2 = []
    for obs in obstacles:
        
        found = False
        for p in obs:
            p = np.round(p, 5)
            if p1[0] != p[0] or p1[1] != p[1]:
                p2 = p
                found = True
                break
        
        if found:
            break

    # assert len(p2) != 0, f"Not all points in obstacles[0] should be equal, obtained {obstacles[0]}"
    # assert p1[0] != p2[0] or p1[1] != p2[1] , f"p1 and p2 should not be equal, obtained p1 {p1}, p2 {p2}"
    # print("**************")
    # print(p1)
    # print(p2)
    # print(p1[0] != p2[0], p1[1] != p2[1], p1[0] != p2[0] or p1[1] != p2[1])
    # print(p1[0], p2[0], p1[1], p2[1])
    # px = np.round(p1, 5)
    # py = np.round(p2, 5)
    # print("--px--py--")
    # print(px)
    # print(py)
    # print(px[0] != py[0], px[1] != py[1], px[0] != py[0] or px[1] != py[1])
    # print(px[0], py[0], px[1], py[1])
    # print()
    # print()

    origin = p1[:2]
    rad = angle_to(origin, p2[:2])
    y_value = rotate(origin, p2[:2], rad)[1]
    # counter_rad = angle_to(origin, obstacles[0][0], clockwise=True)
    new_obstacles = []
    for obs in obstacles:    
        new_obs = []
        for point in obs:
            # Only the X coord is keept
            new_point = (rotate(origin, point[:2], rad)[0], point[-1])
            new_obs.append(new_point)

        new_obstacles.append(new_obs)

    mink_obstacles = []
    for obs in new_obstacles:
        mink_obstacles.append(minkowski_distance(obs, UAV_RADIO, MINKOWSKI_CIRCLE_POINTS))  


    # assert list(np.round(obstacles[-1][-1][:2],3)) == list(np.round(rotate(origin, [rotate(origin, obstacles[-1][-1][:2], rad)[0], y_value], -rad), 3)), f'''
    #     Double Rotation should be equivalent, obtained: {obstacles[-1][-1][:2]} and {rotate(origin, [rotate(origin, obstacles[-1][-1][:2], rad)[0], y_value], -rad)}, 
    #     angle {rad}, origin {origin}, y_value {y_value}, p1 {p1}, p2 {p2}'''

    rollback_transformation = []
    for obs in mink_obstacles:    
        rollback_obs = []
        ## The problem is the rotation angle
        for p in obs:
            rot_point = (*rotate(origin, [p[0], y_value], -rad), p[-1])
            rollback_obs.append(rot_point)
        rollback_transformation.append(rollback_obs)

    # fig = plt.figure()
    # ax1 = fig.add_subplot(111, projection='3d')
    # for o in obstacles:
    #     X,Y,Z = zip(*o)
    #     plt.plot(X,Y,Z)

    # for o in rollback_transformation:
    #     X,Y,Z = zip(*o)
    #     plt.plot(X,Y,Z,'--r')

    # plt.show()

    return rollback_transformation


def obstacles_within_circle(T, ground_obs, aerial_obs):
    
    Cx, Cy = T[:2]
    radio = math.sqrt(CABLE_LENGTH**2 - (T[2]-UGV_HEIGHT)**2) + UGV_RADIO
    
    # Add vertices in range 
    ground_obs_inside_circle = {}
    for k, v in ground_obs.items():
        h = v[-1]
        for vertex in h.points[h.vertices]:
            if euclidian_distance(vertex, [Cx, Cy]) <= radio:
                ground_obs_inside_circle[k] = v
                break
    
    aerial_obs_inside_circle = {}
    for k, v in aerial_obs.items():
        h = v[-1]
        for vertex in h.points[h.vertices]:
            if euclidian_distance(vertex[:2], [Cx, Cy]) <= radio + UAV_RADIO:
                aerial_obs_inside_circle[k] = v
                break

    return ground_obs_inside_circle, aerial_obs_inside_circle


if __name__ == "__main__":
        
    # with open("data/scenarios_v1.pkl", "rb") as f:
    with open("data/scenarios.pkl", "rb") as f:
        scenarios = pkl.load(f)


    # plane_numbers = [100, 60, 30, 20, 10]
    # point_numbers = [100, 60, 30, 20, 10]
    plane_numbers = [30]
    point_numbers = [30]
    # plane_numbers = [120]
    # point_numbers = [250]
    k_lengths = [5]
    graph_pruning_percent = [10,20,30,40,50,60,70,80,90]

    # plane_numbers = [50]
    # point_numbers = [50]
    results = {}
    result_json = {}
    for i, s in enumerate(scenarios[:20]):
        print("scenario", i+1)


        for j in range(len(s["endpoints"])):
            
            PLANES_2D = {}

            print("--> Experiment", j+1)
            endpoints = s["endpoints"][j]
            a = list(endpoints["A"])
            b = list(endpoints["B"])
            a_obs, g_obs = s["obstacles"]["aerial_obs"], s["obstacles"]["ground_obs"] 

            # print(s["visibility_graph"]['vertices'])
            # TODO: 
            # draw the resulting scenario 3D: el escenario debe ser mostrado de forma especial 
            # porque es muy costoso pintarlo tal y como es, ademas de que no se ve muy bien

            for plane_number in plane_numbers:
                for point_number in point_numbers:
                    for k_length in k_lengths:
                        for gpp in graph_pruning_percent:

                            with open("data/scenarios.pkl", "rb") as f:
                                aux_ex = pkl.load(f)
                                v_graph = aux_ex[i]["visibility_graph"]


                            # graph = v_graph["graph"]
                            # points = graph.visgraph.get_points()
                            # total_edges = sum([len(graph.visgraph[p]) for p in points])
                            # edges2prune = int(total_edges * gpp/100) 
                            # for _ in range(edges2prune):
                            #     poped = True
                            #     while poped:
                            #         random.shuffle(points)
                            #         pi = points[0]
                            #         try:
                            #             graph.visgraph[pi].pop()
                            #             poped = False
                            #         except:
                            #             pass
                                
                            # v_graph["graph"] = graph
                            # print(total_edges, sum([len(v_graph["graph"].visgraph[p]) for p in points]))


                            t_first_step, dij_time, dij_len, astar_time, astar_len = visgraph_approach(
                                a, np.array(b), a_obs, g_obs, v_graph, 
                                planes_number=plane_number, 
                                gpoints_number=point_number, 
                                plotting=False, 
                                k_length=k_length, 
                                k_collision=1000
                            )

                            try:
                                results[(plane_number, point_number, k_length, gpp)]['first_step_times'].append(t_first_step)
                                results[(plane_number, point_number, k_length, gpp)]['dijkstra_times'].append(dij_time)
                                results[(plane_number, point_number, k_length, gpp)]['dijkstra_lengths'].append(dij_len)
                                results[(plane_number, point_number, k_length, gpp)]['astar_times'].append(astar_time)
                                results[(plane_number, point_number, k_length, gpp)]['astar_lengths'].append(astar_len)
                            except:
                                results[(plane_number, point_number, k_length, gpp)] = {
                                    'first_step_times': [t_first_step],
                                    'dijkstra_times': [dij_time],
                                    'dijkstra_lengths': [dij_len],
                                    'astar_times': [astar_time],
                                    'astar_lengths': [astar_len]
                                }
                            
                            print("plane_number:", plane_number, "point_number:", point_number, "k_length:", k_length, "gpp:", gpp, "t_first_step:", t_first_step, "dij_time:", dij_time, "dij_len:", dij_len)
                            # print("k_length:", k_length, "t_first_step:", t_first_step, "dij_time:", dij_time, "dij_len:", dij_len)
                        
                            print("... Saving JSON")    
                            try:
                                with open("myresults.json", "r") as f:
                                    result_json = json.loads(f.read())
                            
                                idx = str((i, j, plane_number, point_number, k_length, gpp))
                                result_json[idx] = {"plane_number": plane_number, "point_number": point_number, "k_length": k_length, "gpp": gpp}
                                
                                result_json[idx]['first_step_times'] = [t_first_step]
                                result_json[idx]['dijkstra_times'] = [dij_time]
                                result_json[idx]['dijkstra_lengths'] = [dij_len]
                                result_json[idx]['astar_times'] = [astar_time]
                                result_json[idx]['astar_lengths'] = [astar_len]

                                
                                with open("myresults.json", "w") as f:
                                    f.write(json.dumps(result_json))  
                            except:
                                result_json = {}
                                idx = str((i, j, plane_number, point_number, k_length, gpp))
                                result_json[idx] = {"plane_number": plane_number, "point_number": point_number, "k_length": k_length, "gpp": gpp}
                                
                                with open("myresults.json", "w") as f:
                                    f.write(json.dumps(result_json))

        print()

    with open("results_experiments.txt", "w+") as f:
        
        print("RESULTS")
        for plane_number in plane_numbers:
            for point_number in point_numbers:
                    for k_length in k_lengths:
                        r = results[(plane_number, point_number, k_length)]
                        print(f"--> Planes Number: {plane_number}   Points Number: {point_number}  K Length: {k_length}")
                        print("first_step_times", "mean:", np.mean(r['first_step_times']), "std:", np.std(r['first_step_times']))
                        print("dijkstra_times", "mean:", np.mean(r['dijkstra_times']), "std:", np.std(r['dijkstra_times']))
                        print("dijkstra_lengths", "mean:", np.mean(r['dijkstra_lengths']), "std:", np.std(r['dijkstra_lengths']))
                        print("astar_times", "mean:", np.mean(r['astar_times']), "std:", np.std(r['astar_times']))
                        print("astar_lengths", "mean:", np.mean(r['astar_lengths']), "std:", np.std(r['astar_lengths']))
                        print()

                        f.write(f"--> Planes Number: {plane_number}   Points Number: {point_number}  K Length: {k_length}\n")
                        f.write(f"first_step_times -> mean: {np.mean(r['first_step_times'])}, std: {np.std(r['first_step_times'])}\n")
                        f.write(f"dijkstra_times -> mean: {np.mean(r['dijkstra_times'])}, std: {np.std(r['dijkstra_times'])}\n")
                        f.write(f"dijkstra_lengths -> mean: {np.mean(r['dijkstra_lengths'])}, std: {np.std(r['dijkstra_lengths'])}\n")
                        f.write(f"astar_times -> mean: {np.mean(r['astar_times'])}, std: {np.std(r['astar_times'])}\n")
                        f.write(f"astar_lengths -> mean: {np.mean(r['astar_lengths'])}, std: {np.std(r['astar_lengths'])}\n")
                        f.write("\n")

