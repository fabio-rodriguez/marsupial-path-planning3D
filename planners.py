import sys
from tools import * 

def pvisibility_2D(graph, T):

    current_nodes = [T]
    weights = {T:0}
    previous = {T:None}
    while current_nodes[0][-1] > 0:
        current = current_nodes.pop(0)
        edges = graph[T]
        for e in edges:
            v = e.get_adjacent(current)
            if current == T or (current[-1] < T[-1] and is_icpc(v, current, previous[current])):
                current_nodes.append(v)
                weights[v] = euclidian_distance(v, current) + weights[current]
                previous[v] = current

        current_nodes.sort(key=lambda x: x[-1], reverse=True)
        
    return weights, previous         


def is_icpc(v1,v2,v3):
    r = lineq(v1, v3)
    return (v1[0] <= v2[0] <= v3[0] or v1[0] >= v2[0] >= v3[0]) and r(v2[0]) >= v2[1]




# https://www.udacity.com/blog/2021/10/implementing-dijkstras-algorithm-in-python.html
def dijkstra_algorithm(graph, start_node, goals):

    unvisited_nodes = list(graph.visgraph.get_points())

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
    
    # The algorithm executes until we visit all nodes
    while unvisited_nodes:
        # The code block below finds the node with the lowest score
        current_min_node = unvisited_nodes[0]
        for i in range(1, len(unvisited_nodes)): # Iterate over the nodes
            if shortest_path[unvisited_nodes[i]] < shortest_path[current_min_node]:
                current_min_node = unvisited_nodes[i]

        if (current_min_node.x, current_min_node.y) in goals:
            return current_min_node, previous_nodes, shortest_path
                
        # The code block below retrieves the current node's neighbors and updates their distances
        # neighbors = graph.get_outgoing_edges(current_min_node)
        neighbors = [edge.p2 if edge.p1==current_min_node else edge.p1 for edge in graph.visgraph.get_edges() if edge.__contains__(current_min_node)]
        for neighbor in neighbors:
            tentative_value = shortest_path[current_min_node] + euclidian_distance_points(current_min_node, neighbor)
            
            ### This is for optimizing ground path length + fligth path length
            if (neighbor.x, neighbor.y) in goals:
                tentative_value += goals[(neighbor.x, neighbor.y)]
            
            if tentative_value < shortest_path[neighbor]:
                shortest_path[neighbor] = tentative_value
                # We also update the best path to the current node
                previous_nodes[neighbor] = current_min_node
 
        # After visiting its neighbors, we mark the node as "visited"
        unvisited_nodes.remove(current_min_node)
    
    return None, None, None


