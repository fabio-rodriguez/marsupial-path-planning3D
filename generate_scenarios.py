import numpy as np
import pickle as pkl
import random

from constants import *
from drawing import *
from tools import *


def generate_S1(path):
    
    h = MARSUPIAL_HEIGHT
    x_col1 = 30
    y_col1 = 40
    col_thick = 2

    # First obstacles
    colg1 = np.array([
        [x_col1, y_col1, 0], [x_col1, y_col1+col_thick, 0], [x_col1+col_thick, y_col1, 0], [x_col1+col_thick, y_col1+col_thick, 0],
        [x_col1, y_col1, h], [x_col1, y_col1+col_thick, h], [x_col1+col_thick, y_col1, h], [x_col1+col_thick, y_col1+col_thick, h],
    ])
    cola1 = colg1 + np.array([0,0,h+EPSILON]) 
    colg2 = colg1 + np.array([0,20,0]) 
    cola2 = colg2 + np.array([0,0,h+EPSILON]) 

    bar1 = np.array([
        [x_col1, y_col1+col_thick+EPSILON, 1.5*h], [x_col1, y_col1+20-EPSILON, 1.5*h],
        [x_col1, y_col1+col_thick+EPSILON, 2*h+EPSILON], [x_col1, y_col1+20-EPSILON, 2*h+EPSILON], 
        [x_col1+col_thick, y_col1+col_thick+EPSILON, 1.5*h], [x_col1+col_thick, y_col1+20-EPSILON, 1.5*h],
        [x_col1+col_thick, y_col1+col_thick+EPSILON, 2*h+EPSILON], [x_col1+col_thick, y_col1+20-EPSILON, 2*h+EPSILON], 
    ])

    # Second obstacles
    colg3 = colg1 + np.array([5,-5, 0])
    cola3 = np.array([
        [x_col1+5, y_col1-5, h+EPSILON], [x_col1+5+col_thick, y_col1-5, h+EPSILON], 
        [x_col1+5, y_col1-5+col_thick, h+EPSILON], [x_col1+5+col_thick, y_col1-5+col_thick, h+EPSILON], 
        [x_col1+5, y_col1-5, 5*h], [x_col1+5+col_thick, y_col1-5, 5*h], 
        [x_col1+5, y_col1-5+col_thick, 5*h], [x_col1+5+col_thick, y_col1-5+col_thick, 5*h], 
    ])

    colg4 = colg3 + np.array([0,30, 0])
    cola4 = colg4 + np.array([0,0,h+EPSILON]) 
    cola5 = cola4 + np.array([0,0,h+EPSILON]) 

    bar2 = np.array([        
        [x_col1+5, y_col1-5+col_thick+EPSILON, 2.5*h], [x_col1+5, y_col1+25-EPSILON, 2.5*h], 
        [x_col1+5, y_col1-5+col_thick+EPSILON, 3*h+2*EPSILON], [x_col1+5, y_col1+25-EPSILON, 3*h+2*EPSILON], 
        [x_col1+5+col_thick, y_col1-5+col_thick+EPSILON, 2.5*h], [x_col1+5+col_thick, y_col1+25-EPSILON, 2.5*h], 
        [x_col1+5+col_thick, y_col1-5+col_thick+EPSILON, 3*h+2*EPSILON], [x_col1+5+col_thick, y_col1+25-EPSILON, 3*h+2*EPSILON], 
    ])

    bar3 = np.array([
        [x_col1+5, y_col1-5+col_thick+EPSILON, 4.8*h], [x_col1+5, y_col1+15, 4.8*h], 
        [x_col1+5, y_col1-5+col_thick+EPSILON, 5*h], [x_col1+5, y_col1+15, 5*h], 
        [x_col1+col_thick+5, y_col1-5+col_thick+EPSILON, 4.8*h], [x_col1+col_thick+5, y_col1+15, 4.8*h], 
        [x_col1+col_thick+5, y_col1-5+col_thick+EPSILON, 5*h], [x_col1+col_thick+5, y_col1+15, 5*h] 
    ])

    gobs = [colg1, colg2, colg3, colg4]
    aobs = [cola1, cola2, cola3, cola4, cola5, bar1, bar2, bar3]

    S = (20,65,0)
    T = (41,40,13)
    visgraph = make_visibility_graph(gobs)
    plot_visibility_graph(visgraph, gobs)

    scenario = {
        "S": S,
        "T": T,
        "ground_obstacles": gobs,
        "aerial_obstacles": aobs,
        "ground_vis_graph": visgraph,
    }

    with open(path, "wb") as f:
        f.write(pkl.dumps(scenario))

    with open(path, "rb") as f:
        s = pkl.loads(f.read())

    plot_scenario(s, "images/S1.png")


def generate_S2_atttemp1(path):
    
    h = MARSUPIAL_HEIGHT
    wall_thick = 2

    x_wall1 = 20
    y_wall1 = 50
    wall11 = np.array([
        [x_wall1, y_wall1, 0], [x_wall1+70, y_wall1, 0], [x_wall1, y_wall1+wall_thick, 0], [x_wall1+70, y_wall1+wall_thick, 0],
        [x_wall1, y_wall1, h], [x_wall1+70, y_wall1, h], [x_wall1, y_wall1+wall_thick, h], [x_wall1+70, y_wall1+wall_thick, h],
    ])
    wall12 = wall11 + np.array([0, 0, h+EPSILON])
    wall21 = wall11 + np.array([-10, 10, 0])
    wall22 = wall21 + np.array([0, 0, h+EPSILON])
    
    x_roof1 = 10
    roof1 = np.array([
        [x_roof1, y_wall1, 2*h+2*EPSILON], [x_roof1+35, y_wall1, 2*h+2*EPSILON], 
        [x_roof1, y_wall1+10+wall_thick, 2*h+2*EPSILON], [x_roof1+35, y_wall1+10+wall_thick, 2*h+2*EPSILON],
        [x_roof1, y_wall1, 2*h+wall_thick], [x_roof1+35, y_wall1, 2*h+wall_thick], 
        [x_roof1, y_wall1+10+wall_thick, 2*h+wall_thick], [x_roof1+35, y_wall1+10+wall_thick, 2*h+wall_thick],
    ])
    roof2 = roof1 + np.array([45,0,0])

    x_wall3 = 10 - wall_thick - EPSILON
    y_wall3 = 40
    wall31 = np.array([
        [x_wall3, y_wall3, 0], [x_wall3 + wall_thick, y_wall3, 0], 
        [x_wall3, y_wall3+20+wall_thick, 0], [x_wall3 + wall_thick, y_wall3+20+wall_thick, 0], 
        [x_wall3, y_wall3, h], [x_wall3 + wall_thick, y_wall3, h], 
        [x_wall3, y_wall3+20+wall_thick, h], [x_wall3 + wall_thick, y_wall3+20+wall_thick, h],         
    ])
    wall32 = np.array([
        [x_wall3, y_wall3, h+EPSILON], [x_wall3 + wall_thick, y_wall3, h+EPSILON], 
        [x_wall3, y_wall3+20+wall_thick, h+EPSILON], [x_wall3 + wall_thick, y_wall3+20+wall_thick, h+EPSILON], 
        [x_wall3, y_wall3, 2*h+wall_thick], [x_wall3 + wall_thick, y_wall3, 2*h+wall_thick], 
        [x_wall3, y_wall3+20+wall_thick, 2*h+wall_thick], [x_wall3 + wall_thick, y_wall3+20+wall_thick, 2*h+wall_thick],         
    ])
    wall41 = wall31 + np.array([80+2*EPSILON+wall_thick,10,0])
    wall42 = wall32 + np.array([80+2*EPSILON+wall_thick,10,0])
        
    


    # bar1 = np.array([
    #     [x_col1, y_col1+col_thick+EPSILON, 1.5*h], [x_col1, y_col1+20-EPSILON, 1.5*h],
    #     [x_col1, y_col1+col_thick+EPSILON, 2*h+EPSILON], [x_col1, y_col1+20-EPSILON, 2*h+EPSILON], 
    #     [x_col1+col_thick, y_col1+col_thick+EPSILON, 1.5*h], [x_col1+col_thick, y_col1+20-EPSILON, 1.5*h],
    #     [x_col1+col_thick, y_col1+col_thick+EPSILON, 2*h+EPSILON], [x_col1+col_thick, y_col1+20-EPSILON, 2*h+EPSILON], 
    # ])

    # # Second obstacles
    # colg3 = colg1 + np.array([5,-5, 0])
    # cola3 = np.array([
    #     [x_col1+5, y_col1-5, h+EPSILON], [x_col1+5+col_thick, y_col1-5, h+EPSILON], 
    #     [x_col1+5, y_col1-5+col_thick, h+EPSILON], [x_col1+5+col_thick, y_col1-5+col_thick, h+EPSILON], 
    #     [x_col1+5, y_col1-5, 5*h], [x_col1+5+col_thick, y_col1-5, 5*h], 
    #     [x_col1+5, y_col1-5+col_thick, 5*h], [x_col1+5+col_thick, y_col1-5+col_thick, 5*h], 
    # ])

    # colg4 = colg3 + np.array([0,30, 0])
    # cola4 = colg4 + np.array([0,0,h+EPSILON]) 
    # cola5 = cola4 + np.array([0,0,h+EPSILON]) 

    # bar2 = np.array([        
    #     [x_col1+5, y_col1-5+col_thick+EPSILON, 2.5*h], [x_col1+5, y_col1+25-EPSILON, 2.5*h], 
    #     [x_col1+5, y_col1-5+col_thick+EPSILON, 3*h+2*EPSILON], [x_col1+5, y_col1+25-EPSILON, 3*h+2*EPSILON], 
    #     [x_col1+5+col_thick, y_col1-5+col_thick+EPSILON, 2.5*h], [x_col1+5+col_thick, y_col1+25-EPSILON, 2.5*h], 
    #     [x_col1+5+col_thick, y_col1-5+col_thick+EPSILON, 3*h+2*EPSILON], [x_col1+5+col_thick, y_col1+25-EPSILON, 3*h+2*EPSILON], 
    # ])

    # bar3 = np.array([
    #     [x_col1+5, y_col1-5+col_thick+EPSILON, 4.8*h], [x_col1+5, y_col1+15, 4.8*h], 
    #     [x_col1+5, y_col1-5+col_thick+EPSILON, 5*h], [x_col1+5, y_col1+15, 5*h], 
    #     [x_col1+col_thick+5, y_col1-5+col_thick+EPSILON, 4.8*h], [x_col1+col_thick+5, y_col1+15, 4.8*h], 
    #     [x_col1+col_thick+5, y_col1-5+col_thick+EPSILON, 5*h], [x_col1+col_thick+5, y_col1+15, 5*h] 
    # ])

    gobs = [wall11, wall12, wall21, wall22, wall31, wall32, wall41, wall42]
    aobs = [roof1, roof2]

    S = (20,65,0)
    T = (41,40,13)
    visgraph = make_visibility_graph(gobs)
    plot_visibility_graph(visgraph, gobs)

    scenario = {
        "S": S,
        "T": T,
        "ground_obstacles": gobs,
        "aerial_obstacles": aobs,
        "ground_vis_graph": visgraph,
    }

    with open(path, "wb") as f:
        f.write(pkl.dumps(scenario))

    with open(path, "rb") as f:
        s = pkl.loads(f.read())

    plot_scenario(s, "images/S2.png")


def generate_S2_atttemp2(path):
    
    h = MARSUPIAL_HEIGHT
    wall_thick = 2

    x_wall1 = 10
    y_wall1 = 40
    wall11 = np.array([
        [x_wall1, y_wall1, 0], [x_wall1, y_wall1+wall_thick, 0], 
        [x_wall1+80, y_wall1, 0], [x_wall1+80, y_wall1+wall_thick, 0], 
        [x_wall1, y_wall1, h], [x_wall1, y_wall1+wall_thick, h], 
        [x_wall1+80, y_wall1, h], [x_wall1+80, y_wall1+wall_thick, h]  
    ])
    wall12 = wall11 + np.array([0,0,h+EPSILON])
    wall21 = wall11 + np.array([0,10,0])
    wall22 = wall12 + np.array([0,10,0])
    

    gobs = [wall11, wall12, wall21, wall22]
    aobs = []

    S = (20,65,0)
    T = (41,40,13)
    visgraph = make_visibility_graph(gobs)
    plot_visibility_graph(visgraph, gobs)

    scenario = {
        "S": S,
        "T": T,
        "ground_obstacles": gobs,
        "aerial_obstacles": aobs,
        "ground_vis_graph": visgraph,
    }

    with open(path, "wb") as f:
        f.write(pkl.dumps(scenario))

    with open(path, "rb") as f:
        s = pkl.loads(f.read())

    plot_scenario(s, "images/S2.png")





def get_random_instances(n_instances, ground_n, aerial_n, block_thick, board_size):
    
    h = MARSUPIAL_HEIGHT
    xmax, ymax, zmax = board_size
    side = np.array([0,0,block_thick])
    
    ## Ground Obstacles
    is_disjoint_vertex_ground = lambda p, vlist: not any(map(
        lambda v: 
            v[0] <= p[0] <= v[0]+block_thick and 
            v[1] <= p[1] <= v[1]+block_thick, vlist))     
    
    g_main_vertices = []
    iter = 0
    while iter<MAX_ITERS and len(g_main_vertices)<ground_n+n_instances:
        x = random.randint(0, xmax-block_thick)
        y = random.randint(0, ymax-block_thick)

        if is_disjoint_vertex_ground((x,y), g_main_vertices):
            g_main_vertices.append(np.array((x,y,0)))
    
    ## Aerial Obstacles
    is_disjoint_vertex_air = lambda p, vlist: not any(map(
        lambda v: 
            v[0] <= p[0] <= v[0]+block_thick and 
            v[1] <= p[1] <= v[1]+block_thick and
            v[2] <= p[2] <= v[2]+block_thick , vlist))     
    
    a_main_vertices = []
    iter = 0
    while iter<MAX_ITERS and len(a_main_vertices)<aerial_n+n_instances:
        x = random.randint(0, xmax-block_thick)
        y = random.randint(0, ymax-block_thick)
        z = random.randint(h, zmax-block_thick)

        if is_disjoint_vertex_air((x,y,z), g_main_vertices):
            a_main_vertices.append(np.array((x,y,z)))

    random.shuffle(g_main_vertices)
    random.shuffle(a_main_vertices)

    starting_points = g_main_vertices[:n_instances]
    target_points = a_main_vertices[:n_instances]

    # Get Obstacles
    get_obs = lambda vlist: [np.array([
        v, v + side, 
        v+np.array([0,block_thick,0]), v+np.array([0,block_thick,0]) + side,
        v+np.array([block_thick,0,0]), v+np.array([block_thick,0,0]) + side, 
        v+np.array([block_thick,block_thick,0]), v+np.array([block_thick,block_thick,0]) + side 
        ]) for v in vlist]

    gobs = get_obs(g_main_vertices[n_instances:])
    aobs = get_obs(a_main_vertices[n_instances:])

    # gobs = [np.array([
    #     v, v + side, 
    #     v+np.array([0,block_thick,0]), v+np.array(0,block_thick,0) + side,
    #     v+np.array([block_thick,0,0]), v+np.array([block_thick,0,0]) + side, 
    #     v+np.array(block_thick,block_thick,0), v+np.array(block_thick,block_thick,0) + side 
    #     ]) for v in g_main_vertices[n_instances:]]

    # aobs = [np.array([
    #     v, v + side, 
    #     v+np.array(0,block_thick,0), v+np.array(0,block_thick,0) + side,
    #     v+np.array(block_thick,0,0), v+np.array(block_thick,0,0) + side, 
    #     v+np.array(block_thick,block_thick,0), v+np.array(block_thick,block_thick,0) + side 
    #     ]) for v in a_main_vertices[n_instances:]]

    return starting_points, target_points, gobs, aobs    


def get_random_scenarios(n_scenarios, ground_n, aerial_n, n_instances, block_thick, board_size, path, plot=None):

    rscenarios = []
    for i in range(n_scenarios):

        spoints, tpoints, gobs, aobs = get_random_instances(n_instances, ground_n, aerial_n, block_thick, board_size)
        # visgraph = make_visibility_graph(gobs)
        for j in range(n_instances):
            s = {
                "S": spoints[j],
                "T": tpoints[j],
                "ground_obstacles": gobs,
                "aerial_obstacles": aobs
                # "ground_vis_graph": visgraph,
            }
            rscenarios.append(s)

    with open(path, "wb") as f:
        f.write(pkl.dumps(rscenarios))

    if plot:
    
        with open(path, "rb") as f:
            s = pkl.loads(f.read())
    
        print("Number of Scenarios:", len(s))
        for i in range(len(s)):
            plot3D([
                {
                    "point": s[i]["S"], 
                    "label":"S", "color":"k"
                }, 
                {
                    "point": s[i]["T"],
                    "label":"T", "color":"r"
                }], 
                s[i]["ground_obstacles"]+s[i]["aerial_obstacles"])


if __name__ == "__main__":
    
    # generate_S1("scenarios/S1.pkl")
    generate_S2_atttemp2("scenarios/S2.pkl")

    # n = 1000
    # instances_n = 1
    # ground_n = 40 
    # aerial_n = 40
    # block_thick = 5 
    # board_size = (100,100,50)
    # path = "scenarios/random_scenarios.pkl"
    
    # get_random_scenarios(n, ground_n, aerial_n, instances_n, block_thick, board_size, path, plot=True)