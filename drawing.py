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
        scenario["ground_obstacles"]+scenario["aerial_obstacles"], path_to_output=path)


def plot_scenario_multitarget(scenario, path=None):
    
    targets = [
        {
            "point":ti, 
            "label":f"T{i+1}", "color":"r"
        }  for i, ti in enumerate(scenario["T"])] 

    plot3D([
        {
            "point":scenario["S"], 
            "label":"S", "color":"k"
        }] + targets, 
        scenario["ground_obstacles"]+scenario["aerial_obstacles"], path_to_output=path)


def plot3D(points, obstacles, path_to_output=None, tops=None, ground_paths=None):

    # 8 points defining the cube corners
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    for pts in obstacles:

        print(pts)
        print(len(pts), pts.shape)
        print()
        
        hull = ConvexHull(pts)

        # Plot defining corner points
        ax.plot(pts.T[0], pts.T[1], pts.T[2], ".k", markersize=2)

        # 12 = 2 * 6 faces are the simplices (2 simplices per square face)
        for s in hull.simplices:
            s = np.append(s, s[0])  # Here we cycle back to the first coordinate
            # ax.plot(pts[s, 0], pts[s, 1], pts[s, 2], "r-")
            verts = [list(zip(pts[s, 0],pts[s, 1],pts[s, 2]))]
            ax.add_collection3d(Poly3DCollection(verts, alpha=0.3))

        # Make axis label
        for i in ["x", "y", "z"]:
            eval("ax.set_{:s}label('{:s}')".format(i, i))

    for p in points:
        ax.plot([p["point"][0]], [p["point"][1]], [p["point"][2]], "o", color=p["color"], label=p["label"])  


    if tops:
        for top, values in tops.items():
            ax.plot([top[0]], [top[1]], [top[2]], "og", label="top")
            tether = values["tether"]
            ax.plot(tether[:,0], tether[:,1], tether[:,2], "-r", label="aerial path")

    if ground_paths:
        for gp in ground_paths:
            for i in range(len(gp)-1):
                ax.plot([gp[i][0], gp[i+1][0]], [gp[i][1], gp[i+1][1]], [0,0], "-g", label="ground_path")

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
     
    for vertex in  vsgraph.visgraph.get_points():
        for visible_point in vsgraph.find_visible(vertex):
            X, Y = [vertex.x, visible_point.x], [vertex.y, visible_point.y]
            plt.plot(X, Y, "-r")

    plot_obstacles(obstacles)
    
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

    plot_obstacles(obstacles)

    plt.show()


def plot_3Dtether(top, T, tether, obstacles, path_to_output=None):
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(T[0], T[1], T[2], "or")
    ax.plot(top[0], top[1], top[2], "or")
    
    for obs in obstacles:
        for v1 in obs:
            for v2 in obs:
                ax.plot([v1[0], v2[0]], [v1[1], v2[1]], [v1[2], v2[2]], "-k")
    
    for i in range(len(tether)-1):
        ax.plot(
            [tether[i][0], tether[i+1][0]], 
            [tether[i][1], tether[i+1][1]],
            [tether[i][2], tether[i+1][2]], "-b")

    if path_to_output:
        plt.savefig(path_to_output)

    plt.show()
    plt.close()


def plot_obstacles(obstacles):
    
    for o in obstacles:
        for v1 in o:
            for v2 in o:
                X, Y = [v1[0], v2[0]], [v1[1], v2[1]]
                plt.plot(X, Y, "-k")
    

def plot_dijkstra_graph(S, previous, obstacles):
    
    plot_obstacles(obstacles)

    plt.plot([S[0]], [S[1]], "ro")

    visited = [S]
    while visited:
        current = visited.pop(0)
        for k, v in previous.items():
            if v!=None and v == current:
                plt.plot([current[0], k[0]], [current[1], k[1]], "r-")
                visited.append(k)

    plt.show()


def plot_optimal_solution(S, T, min_ctop, ground_path, ground_obs, aerial_obs):
    
    plot3D([
        {
            "point":S, 
            "label":"S", "color":"k"
        }, 
        {
            "point":T, 
            "label":"T", "color":"r"
        }], 
        ground_obs + aerial_obs, 
        tops=min_ctop,
        ground_paths=[ground_path])
