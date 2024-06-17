import math
import matplotlib.pyplot as plt
import numpy as np
import pyvisgraph as vg

from mpl_toolkits.mplot3d.art3d import Poly3DCollection   
from scipy.spatial import ConvexHull


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

        for s in hull.simplices:
            s = np.append(s, s[0])  # Here we cycle back to the first coordinate
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


def plot_visibility_graph(vsgraph, obstacles, path_to_image=None):
     
    for vertex in  vsgraph.graph.get_points():
        for visible_point in vsgraph.find_visible(vertex):
            X, Y = [vertex.x, visible_point.x], [vertex.y, visible_point.y]
            plt.plot(X, Y, "-r")

    for o in obstacles:
        for v1 in o:
            for v2 in o:
                X, Y = [v1[0], v2[0]], [v1[1], v2[1]]
                plt.plot(X, Y, "-k")
    
    
    if path_to_image != None:
        plt.savefig(path_to_image)
    
    plt.show()
    plt.close()


def plot_polygonal_paths(weights, previous, tops, T, obstacles):
    
    T = vg.Point(*T)
    plt.plot([p[0] for p in tops], [p[1] for p in tops], "or")
    plt.plot([T.x], [T.y], "or")

    for top in tops:
        vtop = vg.Point(*top)
        if vtop in weights:
            polygonal = [vtop]
            while True:
                prev = previous[polygonal[-1]]
                polygonal.append(prev)
                if prev == T:
                    break
            
            plt.plot([p.x for p in polygonal], [p.y for p in polygonal], "-b")

    
    for o in obstacles:
        for v1 in o:
            for v2 in o:
                X, Y = [v1[0], v2[0]], [v1[1], v2[1]]
                plt.plot(X, Y, "-k")
    

    plt.show()



