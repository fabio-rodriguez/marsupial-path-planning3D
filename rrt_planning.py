'''
MIT License
Copyright (c) 2019 Fanjin Zeng
This work is licensed under the terms of the MIT license, see <https://opensource.org/licenses/MIT>.  
'''

import matplotlib.pyplot as plt
import numpy as np
import pickle as pkl
import time

from collections import deque
from matplotlib import collections  as mc
from random import random
from scipy import linalg

from cat2 import *
from constants import *
from tools import euclidian_distance


BOARD_SHAPE = (100, 70, 40)

class Graph:
    ''' Define graph '''
    def __init__(self, startpos, endpos):
        self.startpos = startpos
        self.endpos = endpos

        self.vertices = [startpos]
        self.edges = []
        self.success = False

        self.vex2idx = {tuple(startpos):0}
        self.neighbors = {0:[]}
        self.distances = {0:0.}

        self.sx = endpos[0] - startpos[0]
        self.sy = endpos[1] - startpos[1]

        self.opt_cat = []

    def get_id(self, node):
        return self.vex2idx[tuple(node)]

    def add_vex(self, pos):
        try:
            idx = self.vex2idx[tuple(pos)]
        except:
            idx = len(self.vertices)
            self.vertices.append(pos)
            self.vex2idx[tuple(pos)] = idx
            self.neighbors[idx] = []
        return idx

    def add_edge(self, idx1, idx2, cost):
        self.edges.append((idx1, idx2))
        self.neighbors[idx1].append((idx2, cost))
        self.neighbors[idx2].append((idx1, cost))

    def randomPosition(self, board_shape, obstacles, radius):

        deltaX, deltaY = board_shape         
        while True:
            random_point = np.random.rand(2)-0.5
            random_point[0] *= 2*deltaX 
            random_point[1] *= deltaY
        
            distances = []
            new_edges = []
            for node in self.vertices:
                d = euclidian_distance(node, random_point)
                if d <= radius:  
                    distances.append(d)
                    new_edges.append((node, random_point))

            items = sorted(zip(distances, new_edges))            
            for d, new_edge in items:
                if is_visible(*new_edge, obstacles):
                
                    node_parent, new_node = new_edge 
                    parentidx = self.get_id(node_parent)
                    newidx = self.add_vex(new_node)

                    return d, newidx, parentidx
                

def RRT_star(startpos, T, ground_obs, aerial_obs, radius, k_length, n_iter=2*10**3, time_for_ending=200):
    ''' RRT star algorithm '''
    endpos = T[:2]
    G = Graph(startpos, endpos)

    gobs_4planning = [[v[:2] for v in oi if v[-1]==0] for oi in ground_obs]

    feasible_dist = math.sqrt(TETHER_LENGTH**2 - (endpos[-1]-MARSUPIAL_HEIGHT)**2) 
    init_t = time.time()
    iter = 0
    while iter < n_iter and time.time() - init_t < time_for_ending:

        d, newidx, parentidx = G.randomPosition(BOARD_SHAPE[:2], gobs_4planning, radius)
        
        G.add_edge(newidx, parentidx, d)
        G.distances[newidx] = G.distances[parentidx] + d
        newvex = G.vertices[newidx]

        ####
        if euclidian_distance(newvex, G.endpos) <= feasible_dist:
            
            top = [*newvex, MARSUPIAL_HEIGHT - UAV_RADIUS]
            cat, length, t = get_min_catenary_rectangles(
                np.array(top), T,
                ground_obs + aerial_obs, 
                euclidian_distance_lists(top, G.endpos), 
                TETHER_LENGTH, k_length, col2=True)
        
            if length>0:

                endidx = G.add_vex(G.endpos)
                G.success = True
                G.add_edge(newidx, endidx, length)

                if len(G.opt_cat) == 0 or G.distances[newidx]+length < G.distances[endidx]:
                    G.distances[endidx] = G.distances[newidx]+length
                    G.opt_cat = cat

        # dist = euclidian_distance(newvex, G.endpos)
        # if dist < 2 * radius:
        #     endidx = G.add_vex(G.endpos)
        #     G.add_edge(newidx, endidx, dist)
        #     try:
        #         G.distances[endidx] = min(G.distances[endidx], G.distances[newidx]+dist)
        #     except:
        #         G.distances[endidx] = G.distances[newidx]+dist

            
        iter += 1

    return G


def dijkstra(G, endnode, g_obs, a_obs):
    '''
    Dijkstra algorithm for finding shortest path from start position to end.
    '''
    srcIdx = G.vex2idx[tuple(G.startpos)]
    dist, endnode, dstIdx = get_dest_node(G, endnode, g_obs, a_obs)

    # build dijkstra
    nodes = list(G.neighbors.keys())
    dist = {node: float('inf') for node in nodes}
    prev = {node: None for node in nodes}
    dist[srcIdx] = 0

    while nodes:
        curNode = min(nodes, key=lambda node: dist[node])
        nodes.remove(curNode)
        if dist[curNode] == float('inf'):
            break

        for neighbor, cost in G.neighbors[curNode]:
            newCost = dist[curNode] + cost
            if newCost < dist[neighbor]:
                dist[neighbor] = newCost
                prev[neighbor] = curNode

    # retrieve path
    path = deque()
    curNode = dstIdx
    while prev[curNode] is not None:
        path.appendleft(G.vertices[curNode])
        curNode = prev[curNode]
    path.appendleft(G.vertices[curNode])
    return list(path)


def get_dest_node(G, endpoint, ground_obs, aerial_obs, k_length=26):
    ids = []
    distances = []
    end_nodes = []

    for v in G.vertices:
        dp_function = lambda x: get_min_catenary_rectangles(np.array(x), endpoint, ground_obs + aerial_obs, euclidian_distance_lists(x, endpoint), TETHER_LENGTH, k_length)   
        _, length, _ = dp_function(list(v)+[0])
        if length > 0:
            id = G.vex2idx[tuple(v)]
            ids.append(id)
            end_nodes.append(tuple(v))
            distances.append(G.distances[id])

    distances, end_nodes, ids = zip(*sorted(zip(distances, end_nodes, ids)))

    return distances[0], end_nodes[0], ids[0]


def plot(G, obstacles, radius, path=None):
    '''
    Plot RRT, obstacles and shortest path
    '''

    px = [x for x, y in G.vertices]
    py = [y for x, y in G.vertices]
    fig, ax = plt.subplots()

    ax.scatter(px, py, c='cyan')
    ax.scatter(G.startpos[0], G.startpos[1], c='black')
    ax.scatter(G.endpos[0], G.endpos[1], c='black')

    if obstacles:
        from scipy.spatial import ConvexHull
        for o in obstacles:
            points = np.array([np.array(p[:2]) for p in o if p[-1]==0])
            h = ConvexHull(points)
            for simplex in h.simplices:
                ax.plot(points[simplex, 0], points[simplex, 1], 'k-')


    lines = [(G.vertices[edge[0]], G.vertices[edge[1]]) for edge in G.edges]
    lc = mc.LineCollection(lines, colors='green', linewidths=2)
    ax.add_collection(lc)

    if path is not None:
        paths = [(path[i], path[i+1]) for i in range(len(path)-1)]
        lc2 = mc.LineCollection(paths, colors='blue', linewidths=3)
        ax.add_collection(lc2)

    ax.autoscale()
    ax.margins(0.1)

    plt.show()


def pathSearch(startpos, endpos, obstacles, n_iter, radius, stepSize):
    G = RRT_star(startpos, endpos, obstacles, n_iter, radius, stepSize)
    if G.success:
        path = dijkstra(G)
        # plot(G, obstacles, radius, path)
        return path


if __name__ == '__main__':


    with open("scenarios/S2.pkl", "rb") as f:
        scenario = pkl.load(f)

    S = scenario["S"]
    T = scenario["T"]
    a_obs, g_obs = scenario["aerial_obstacles"], scenario["ground_obstacles"] 
    v_graph = scenario["ground_vis_graph"]
    radius = 10 # RRT parameter
    k_length = 26

    tt = time.time()
    G = RRT_star(S[:2], T, g_obs, a_obs, radius, k_length)
    print("time for graph", tt-time.time())

    if G.success:
        path = dijkstra(G, T, g_obs, a_obs)
        print(path)
        plot(G, g_obs, radius, path)
    else:
        plot(G, g_obs, radius)


                
            
            
            
            

            