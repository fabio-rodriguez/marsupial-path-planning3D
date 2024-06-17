import numpy as np
import pickle as pkl

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


if __name__ == "__main__":
    
    generate_S1("scenarios/S1.pkl")