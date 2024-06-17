import math
import numpy as np
import pyvisgraph as vg

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


def get_min_catenary(top, obstacles, L):
    pass


def catenary3D(catenary, vplane):
    pass
            