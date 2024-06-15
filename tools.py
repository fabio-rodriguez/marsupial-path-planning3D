import math
import matplotlib.pyplot as plt
import numpy as np
import pyvisgraph as vg


from constants import *
from mpl_toolkits.mplot3d.art3d import Poly3DCollection   
from scipy.spatial import ConvexHull
from trimesh import intersections as tint


def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0: 
       return v
    return v / norm


def euclidian_distance(x1, x2):
    return np.linalg.norm(x1-x2)


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


def get_top_circ_radious(T,):
    l, h, r = TETHER_LENGTH, MARSUPIAL_HEIGHT, UAV_RADIUS
    return math.sqrt(l**2 - (T[-1]-h+r)**2)
    

def intersect_obstacles_and_vertical_plane(p1, p2, p3, obstacles):

    v1 = p2-p1
    v2 = p3-p1
    normal_vector = normalize(np.cross(v1, v2))

    vertical_obstacles = []
    for oi in obstacles: 
        hull = hull = ConvexHull(oi)
        edges = []    
        for simplex in hull.simplices:
            s = hull.points[simplex]
            edges += [(s[i], s[i+1]) for i in range(len(s)-1)] + [(s[-1], s[0])]
        
        intersections = plane_edges_collision_points_normal(p1, normal_vector, edges)

        if len(intersections):
            vertical_obstacles.append(intersections)
    
    return vertical_obstacles


def plane_edges_collision_points_normal(plane_point, normal, edges):

    intersections = []
    for edge in edges:
        inters, valid = tint.plane_lines(plane_point, normal, edge, line_segments=True)
        if len(inters):
            intersections.append(inters[0])
            
    return intersections


def plot_vertical_plane(plane, T, tops):
    
    title = round(math.degrees(plane["angle"]),2)
    coords = plane["coords"]
    gobs = plane["ground_obstacles"]
    aobs = plane["aerial_obstacles"]

    plot2D(str(title), [{"point":T, "label":"T", "color":"r"}], gobs+aobs, tops, coords)



def plot2D(title, points, obstacles, tops, coords=None, path_to_output=None):


    fig = plt.figure()
    ax = fig.add_subplot()
    
    ax.set_xlim([-20, 100])
    ax.set_ylim([0, 14])

    for pts in obstacles:

        pts = np.array(pts)
        ptsx = pts[:,coords[0]]  
        ptsz = pts[:,coords[1]]  
        
        hull = ConvexHull(list(zip(ptsx,ptsz)))

        # Plot defining corner points
        plt.plot(ptsx, ptsz, "k")

        # 12 = 2 * 6 faces are the simplices (2 simplices per square face)
        for s in hull.simplices:
            s = np.append(s, s[0])  # Here we cycle back to the first coordinate
            # ax.plot(pts[s, 0], pts[s, 1], pts[s, 2], "r-")
            verts = [list(zip(pts[s, 0],pts[s, 1]))]
            ax.fill(pts[s, 0], pts[s, 1])

    for d in points:
        p,l,c =d["point"], d["label"], d["color"]
        p = (p[coords[0]], p[coords[1]])
        ax.plot(p[0], p[1], "o", color=c, label=l)

    X_tops = [top[coords[0]] for top in tops]
    Z_tops = [top[coords[1]] for top in tops]
    ax.plot(X_tops, Z_tops, "o", color="b", label="tops")


    plt.legend()
    plt.title(title)

    if path_to_output:
        plt.savefig(path_to_output)

    plt.show()
    plt.close()
