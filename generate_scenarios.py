import numpy as np
import pickle as pkl

from constants import *
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
    cola1 = colg1 + np.array([0,0,h]) 
    colg2 = colg1 + np.array([0,20,0]) 
    cola2 = colg2 + np.array([0,0,h]) 

    bar1 = np.array([
        [x_col1, y_col1, 1.5*h], [x_col1, y_col1+20, 1.5*h], 
        [x_col1+col_thick, y_col1, 1.5*h], [x_col1+col_thick, y_col1+20, 1.5*h], 
        [x_col1, y_col1, 2*h], [x_col1, y_col1+20, 2*h], 
        [x_col1+col_thick, y_col1, 2*h], [x_col1+col_thick, y_col1+20, 2*h],          
    ])

    # Second obstacles
    colg3 = colg1 + np.array([5,-5, 0])
    cola3 = np.array([
        [x_col1+5, y_col1-5, h], [x_col1+5+col_thick, y_col1-5, h], 
        [x_col1+5, y_col1-5+col_thick, h], [x_col1+5+col_thick, y_col1-5+col_thick, h], 
        [x_col1+5, y_col1-5, 5*h], [x_col1+5+col_thick, y_col1-5, 5*h], 
        [x_col1+5, y_col1-5+col_thick, 5*h], [x_col1+5+col_thick, y_col1-5+col_thick, 5*h], 
    ])

    colg4 = colg3 + np.array([0,30, 0])
    cola4 = colg4 + np.array([0,0,h]) 
    cola5 = cola4 + np.array([0,0,h]) 

    bar2 = np.array([
        [x_col1+5, y_col1-5, 2.5*h], [x_col1+5+col_thick, y_col1-5, 2.5*h], 
        [x_col1+5, y_col1-5, 3*h], [x_col1+5+col_thick, y_col1-5, 3*h], 
        [x_col1+5, y_col1+25, 2.5*h], [x_col1+5+col_thick, y_col1+25, 2.5*h], 
        [x_col1+5, y_col1+25, 3*h], [x_col1+5+col_thick, y_col1+25, 3*h],         
    ])

    bar3 = np.array([
        [x_col1+5, y_col1-5, 4.8*h], [x_col1+5+col_thick, y_col1-5, 4.8*h], 
        [x_col1+5, y_col1-5, 5*h], [x_col1+5+col_thick, y_col1-5, 5*h], 
        [x_col1+5, y_col1+15, 4.8*h], [x_col1+5+col_thick, y_col1+15, 4.8*h], 
        [x_col1+5, y_col1+15, 5*h], [x_col1+5+col_thick, y_col1+15, 5*h],         
    ])

    gobs = [colg1, colg2, colg3, colg4]
    aobs = [cola1, cola2, cola3, cola4, cola5, bar1, bar2, bar3]

    S = (20,65,0)
    T = (42,40,8)

    scenario = {
        "S": S,
        "T": T,
        "ground_obstacles": gobs,
        "aerial_obstacles": aobs,
        "ground_vis_graph": make_visibility_graph(gobs),
    }

    with open(path, "wb") as f:
        f.write(pkl.dumps(scenario))

    with open(path, "rb") as f:
        s = pkl.loads(f.read())

    plot_scenario(s, "images/S1.png")





if __name__ == "__main__":
    
    generate_S1("scenarios/S1.pkl")