import numpy as np

from drawing import *
from tools import *
from planners import *


def get_cvisible_points(T, g_obs, a_obs, p, q, k_length, k_collision):
    
    T_proj = np.array([T[0], T[1], HTOP])
    cradius = get_top_circ_radious(T)

    vplanes = get_vertical_planes(T, T_proj, cradius, p, g_obs, a_obs)

    tops3D = []
    for vp in vplanes:
        tops = get_take_off_points(cradius, vp, q)
        cvis_tops = get_cvisible_tops(vp, tops, T)
        


        # CHECKPOINT #  plot_vertical_plane(vp,T,tops) 

        # ... rotate/project plane
        # ... find nonpvisible intervals (left, center, right) 
        # ... find pvisible tops 
        # ... check cvisibility of feasible tops (find L)
        # ... rotate/project feasible tops
        # ... get ground points !!!
        # ... append to tops3D the ground point the cvisible top and tether length (Pg, topPg, L)

    return tops3D

    # T_proj = np.array(list(T[:2])+[0])
    # v1 = T-ground_point
    # v2 = T_projection-ground_point
    # normal_vector = normalize(np.cross(v1, v2))

    # if tuple(normal_vector) in PLANES_2D:
    #     transformed_obstacles = PLANES_2D[tuple(normal_vector)]
    #     time_mink = 0
    # else:
    #     # Intervals can affect the computation of the intersections
    #     # intervals = plane_intervals(target, T_projection, ground_point) # Get the interval to know if a point is inside the vertical plane
    #     obstacles = []
    #     # for _,hull,_,_ in ground_obs.values():
    #     for _,hull,_ in ground_obs.values():
            
    #         edges = []    
    #         for simplex in hull.simplices:
    #             s = hull.points[simplex]
    #             edges += [(s[i], s[i+1]) for i in range(len(s)-1)] + [(s[-1], s[0])]
            
    #         intersections = plane_edges_collision_points_normal(ground_point, normal_vector, edges) # find intersections between obstacles and the vertical plane

    #         if len(intersections):
    #             obstacles.append(intersections)
            
    #     for _, hull in aerial_obs.values():
            
    #         edges = []
    #         for simplex in hull.simplices:
    #             s = hull.points[simplex]
    #             edges += [(s[i], s[i+1]) for i in range(len(s)-1)] + [(s[-1], s[0])]
            
    #         intersections = plane_edges_collision_points_normal(ground_point, normal_vector, edges) # find intersections between obstacles and the vertical plane

    #         if len(intersections):
    #             obstacles.append(intersections)

    #     # fig = plt.figure()
    #     # ax = fig.add_subplot(111, projection='3d')
    #     # for oi in obstacles:
    #     #     X, Y, Z  = zip(*oi)
    #     #     plt.plot(X, Y, Z, '.k') 

    #     transformed_obstacles = obstacles
    #     time_mink = 0
    #     # t_init = time.time()
    #     # transformed_obstacles = minkowski_transform_2D(obstacles) if len(obstacles) else []
    #     # time_mink = time.time()-t_init

    #     PLANES_2D[tuple(normal_vector)] = transformed_obstacles 

    #     # for oi in transformed_obstacles:
    #     #     X, Y, Z  = zip(*oi)
    #     #     plt.plot(X, Y, Z, '.g') 

    #     # plt.plot([ground_point[0], target[0], target[0]], [ground_point[1], target[1], target[1]], [ground_point[2], target[2], 0], 'or')
    #     # plt.show()

    # result = dp3D(ground_point, target, CABLE_LENGTH, UAV_RADIO, UGV_RADIO, UGV_HEIGHT, transformed_obstacles, k_length, k_collision), time_mink    
    # return result


def get_vertical_planes(T, T_proj, cradius, p, ground_obstacles, aerial_obstacles):

    Cx, Cy, _ = T_proj
    planes = []
    
    for i in range(p):
        border_point = np.array([math.cos(math.pi/p*i) + Cx, math.sin(math.pi/p*i) + Cy, HTOP])
        v = normalize(border_point - T_proj)
        Q = T_proj - v*cradius
        coords = get_plane_coords(Q, T) # find the best coordinates to represent the vertical plane (avoiding null coordinates problems)

        vplane_gobs = intersect_obstacles_and_vertical_plane(border_point, T, T_proj, ground_obstacles)
        vplane_aobs = intersect_obstacles_and_vertical_plane(border_point, T, T_proj, aerial_obstacles)
        planes.append({
            "Q": Q,
            "top_vector": v,
            "angle": math.pi/p*i,
            "coords": coords,
            "ground_obstacles": vplane_gobs,
            "aerial_obstacles": vplane_aobs
        })
    
    return planes


def get_plane_coords(X, T):
    return [0,2] if abs(X[0]-T[0]) > abs(X[1]-T[1]) else [1,2] 


def get_take_off_points(cradius, vertical_plane, q):

    Q = vertical_plane["Q"]
    v = vertical_plane["top_vector"]
    step = 2*cradius/(q-1)

    return [Q + step*i*v for i in range(q)]
        

def get_cvisible_tops(vplane, tops, T):
    
    c0, c1 = vplane["coords"]
    T_proj = (T[c0], T[c1])
    tops_proj = [[np.array((top[c0], top[c1]))] for top in tops]
    
    obs_proj = []
    for obs in vplane["ground_obstacles"]+vplane["aerial_obstacles"]:
        obs = [np.array((vertex[c0], vertex[c1])) for vertex in obs]
        gch = ConvexHull(obs)
        obs_proj.append(gch.points[gch.vertices])

    vertices_lists = [[T_proj]] + tops_proj + obs_proj
    visgraph = make_visibility_graph(vertices_lists)

    # CHECKPOINT # plot_visibility_graph(visgraph, obs_proj)

    weights, previous = pvisibility_2D(visgraph, T_proj, TETHER_LENGTH, obs_proj)

    # CHECKPOINT # plot_polygonal_paths(weights, previous, [top[0] for top in tops_proj], T_proj, obs_proj)


