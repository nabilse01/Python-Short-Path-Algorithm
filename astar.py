import math
import heapq
import math
import time
import sys
import os
'''
This code is a Python program that finds the shortest path between two nodes in a graph using the A* algorithm.

The program starts by defining a graph using a dictionary data structure, where each key is a node and its value is a dictionary of its neighbors and the distance/cost to reach them. It also defines a coordinates dictionary that represents the (x,y) coordinates for each node.

It then adds reverse edges to the graph to make it undirected.

Next, it defines a function straight_line_distance that calculates the straight-line distance heuristic between two given nodes based on their coordinates.

The main function astar takes the input graph, start and target nodes. It initializes variables such as the heap priority queue, predecessor dictionary came_from, and two score dictionaries that are used to keep track of the path's cost. The A* algorithm is implemented using a while loop that continues until all the nodes have been visited or the target node is reached. It selects the node with the lowest f_score from the heap and expands it to check its neighbors. It updates each neighbor's g_score if the tentative_g_score is lower than the current g_score. If the target node is reached, the function returns the shortest path and its total cost. Otherwise, it returns infinity and an empty list indicating no path exists between start and target.

Finally, the code sets up the start and target nodes, runs the astar function, and prints the elapsed time, the shortest_path_cost, and the shortest_path.

'''
graph = {
    'A': {'B': 2, 'C2': 2},
    'C2': {'C': 2},
    'B': {'D': 4, 'E': 5},
    'C': {'E': 1},
    'D': {},
    'E': {'G': 8},
    'G': {'F': 2},
    'F': {}
}

coordinates = {
    'A': (0, 0), 'B': (2, 0), 'C2': (0, 1), 'C': (0, 2), 'D': (4, 0), 'E': (2, 2), 'G': (3, 2), 'F': (4, 2)
}

# This code adds reverse edges to the graph.
for node in graph:
    for neighbor, distance in graph[node].items():
        # Add reverse edge
        if neighbor not in graph:
            graph[neighbor] = {}
        graph[neighbor][node] = distance

# This function defines the straight-line distance heuristic function.


def straight_line_distance(node, goal):
    x1, y1 = coordinates[node]
    x2, y2 = coordinates[goal]
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

# This function implements the A* algorithm.


def astar(graph, start, target):
    # Here we use a heap which is used as a priority queue to store nodes based on cost/distance.
    heap = [(straight_line_distance(start, target), start)]
    came_from = {}  # variable to store the predecessor of each node
    # dictionary which will hold gScore of all nodes initialized to infinity
    g_score = {node: float('inf') for node in graph}
    g_score[start] = 0  # The starting nodes gScore is set to zero
    # dictionary which will hold fScore of all nodes initialized to infinity
    f_score = {node: float('inf') for node in graph}
    # The fScore of start node is calculated
    f_score[start] = straight_line_distance(start, target)
    while heap:
        _, current = heapq.heappop(heap)
        # If the current node is the target node, then we have found the shortest path.
        if current == target:
            path = []
            while current != start:
                path.insert(0, current)
                current = came_from[current]
            path.insert(0, start)
            return (g_score[target], path)
        # For each neighbor of the current node, we calculate the tentative g score.
        for neighbor, cost in graph[current].items():
            tentative_g_score = g_score[current] + cost
            # If the tentative g score is lower than the current g score of the neighbor node, then we update the variables accordingly.
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + \
                    straight_line_distance(neighbor, target)
                heapq.heappush(heap, (f_score[neighbor], neighbor))
    # If there is no path found to the target i.e heap is empty we return infinity as the distance and an empty list as the path
    return (float('inf'), [])


# Get the start and target nodes from the environment variables.
Start = os.environ.get('Start', 'D')
Target = os.environ.get('Target', 'F')


# Start the timer.
start_time = time.time()

# Find the shortest path from the start node to the target node.
shortest_path_cost, shortest_path = astar(graph, Start, Target)

# Stop the timer.
end_time = time.time()

# Calculate the elapsed time.
elapsed_time = end_time - start_time

# Print the elapsed time, shortest path cost, and shortest path.
print(f"Elapsed time: {elapsed_time} seconds")
print("Shortest Path Cost:", shortest_path_cost)
print("Shortest Path:", shortest_path)
