import math
import numpy as np
import pyvisgraph as vg
import sys

from constants import *
from numpy.linalg import lstsq
from scipy.spatial import ConvexHull
from trimesh import intersections as tint


def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
       return v
    return v / norm


def euclidian_distance(x1, x2):
    return np.linalg.norm(x1-x2)


def vpoint_euclidian_distance(v1, v2):
    return np.linalg.norm(np.array([v1.x, v1.y])-np.array([v2.x, v2.y]))


def make_visibility_graph(vertices_lists):

    vertices = [[vg.Point(*vertex[:2]) for vertex in vlist] for vlist in vertices_lists]
    g = vg.VisGraph()
    g.build(vertices)

    return g


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
        intersections = set([tuple(i) for i in intersections])

        if len(intersections):
            vertical_obstacles.append(list(intersections))

    return vertical_obstacles


def plane_edges_collision_points_normal(plane_point, normal, edges):

    intersections = []
    for edge in edges:
        inters, valid = tint.plane_lines(plane_point, normal, edge, line_segments=True)
        if len(inters):
            intersections.append(inters[0])

    return intersections


def lineq(p1, p2):

    points = [p1, p2]
    x_coords, y_coords = zip(*points)
    A = np.vstack([x_coords, np.ones(len(x_coords))]).T
    m, c = lstsq(A, y_coords)[0]
    return lambda x: m*x + c


# Inspired on:
# https://www.udacity.com/blog/2021/10/implementing-dijkstras-algorithm-in-python.html
def upd_dijkstra_algorithm(start_node, goals, vertices, obstacles):

    unvisited_nodes = [start_node] + vertices + list(goals.keys())

    # We'll use this dict to save the cost of visiting each node and update it as we move along the graph   
    shortest_path = {}
 
    # We'll use this dict to save the shortest known path to a node found so far
    previous_nodes = {}
 
    # We'll use max_value to initialize the "infinity" value of the unvisited nodes   
    max_value = sys.maxsize

    for node in unvisited_nodes:
        shortest_path[node] = max_value
    
    # However, we initialize the starting node's value with 0   
    shortest_path[start_node] = 0
    previous_nodes[start_node] = None
    
    # The algorithm executes until we visit all nodes
    while unvisited_nodes:
        # The code block below finds the node with the lowest score
        current = unvisited_nodes[0]
        for i in range(1, len(unvisited_nodes)): # Iterate over the nodes
            if shortest_path[unvisited_nodes[i]] < shortest_path[current]:
                current = unvisited_nodes[i]
        unvisited_nodes.remove(current)

        # if (current.x, current.y, current.z) in goals:
        #     return current, previous_nodes, shortest_path
                
        # The code block below retrieves the current node's neighbors and updates their distances
        # neighbors = graph.get_outgoing_edges(current_min_node)
        for node in unvisited_nodes:
            if is_visible(current, node, obstacles):

                shortest_path[node] = euclidian_distance(current, node)
                previous_nodes[node] = current

                ### This is for optimizing ground path length + fligth path length                
                if node in goals:
                    shortest_path[node] += node["length"]

    return shortest_path, previous_nodes


# Given the numerical problems of pyvisgraph we must implement our own visibility
def is_visible(P1, P2, obstacles):
    for obs in obstacles:
        hull  = ConvexHull(obs)

        for simplex in hull.simplices:
            if do_intersect(obs[simplex, 0], obs[simplex, 1], P1,P2):
                return False

    return True


def orientation(p, q, r):
    """Return the orientation of the triplet (p, q, r).
    0 -> p, q and r are collinear
    1 -> Clockwise
    2 -> Counterclockwise
    """
    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
    if val == 0:
        return 0
    elif val > 0:
        return 1
    else:
        return 2

def on_segment(p, q, r):
    """Check if point q lies on segment pr."""
    if min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and min(p[1], r[1]) <= q[1] <= max(p[1], r[1]):
        return True
    return False


def do_intersect(p1, q1, p2, q2):
    """Return True if line segments 'p1q1' and 'p2q2' intersect."""
    # Find the four orientations needed for the general and special cases
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    # General case
    if o1 != o2 and o3 != o4:
        return True

    # Special cases
    # p1, q1 and p2 are collinear and p2 lies on segment p1q1
    if o1 == 0 and on_segment(p1, p2, q1):
        return True

    # p1, q1 and q2 are collinear and q2 lies on segment p1q1
    if o2 == 0 and on_segment(p1, q2, q1):
        return True

    # p2, q2 and p1 are collinear and p1 lies on segment p2q2
    if o3 == 0 and on_segment(p2, p1, q2):
        return True

    # p2, q2 and q1 are collinear and q1 lies on segment p2q2
    if o4 == 0 and on_segment(p2, q1, q2):
        return True

    return False
