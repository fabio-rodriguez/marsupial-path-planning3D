import math
import matplotlib.pyplot as plt
import numpy as np
import pyvisgraph as vg

from constants import *
from mpl_toolkits.mplot3d.art3d import Poly3DCollection   
from scipy.spatial import ConvexHull


def euclidian_distance(v1, v2):
    return math.sqrt(sum([(x1 - x2)**2 for x1, x2 in zip(v1, v2)]))


def make_visibility_graph(ground_obstacles):

    vertices = [[vg.Point(*vertex) for vertex in go] for go in ground_obstacles]    
    g = vg.VisGraph()

    return g.build(vertices)


def plot_scenario(scenario, path=None):
    
    plot3D([
        {
            "point":scenario["S"], 
            "label":"S", "color":"k"
        }, 
        {
            "point":scenario["T"], 
            "label":"T", "color":"r"
        }], 
        scenario["ground_obstacles"]+scenario["aerial_obstacles"], path)


def plot3D(points, obstacles, path_to_output=None):


    # 8 points defining the cube corners
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")


    for pts in obstacles:

        hull = ConvexHull(pts)

        # Plot defining corner points
        ax.plot(pts.T[0], pts.T[1], pts.T[2], "k")

        # 12 = 2 * 6 faces are the simplices (2 simplices per square face)
        for s in hull.simplices:
            s = np.append(s, s[0])  # Here we cycle back to the first coordinate
            # ax.plot(pts[s, 0], pts[s, 1], pts[s, 2], "r-")
            verts = [list(zip(pts[s, 0],pts[s, 1],pts[s, 2]))]
            ax.add_collection3d(Poly3DCollection(verts))

        # Make axis label
        for i in ["x", "y", "z"]:
            eval("ax.set_{:s}label('{:s}')".format(i, i))

    for d in points:
        p,l,c =d["point"], d["label"], d["color"]
        ax.plot(p[0], p[1], p[2], "o", color=c, label=l)

    plt.legend()

    if path_to_output:
        plt.savefig(path_to_output)

    plt.show()


# TODO fix
# def obstacles_within_circle(T, ground_obs, aerial_obs):
    
#     l, h, r = TETHER_LENGTH, MARSUPIAL_HEIGHT, UAV_RADIOUS
#     l = 10
#     Cx, Cy = T[:2]
#     radio = math.sqrt(l**2 - (T[2]-h+r)**2)
    
#     # Add vertices in range 
#     result_gobs = [go  for go in ground_obs if any(map(lambda v: euclidian_distance(v[:2], [Cx, Cy]) <= radio, go))] 
#     result_aobs = [ao for ao in aerial_obs if any(map(lambda v: euclidian_distance(v[:2], [Cx, Cy]) <= radio, ao)) and any(map(lambda v: v[-1] < T[-1], ao))] 
        
#     return result_gobs, result_aobs

