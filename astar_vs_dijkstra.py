import math
import csv
import heapq
import time
import sys
import os
import random

graph = {}

# Open CSV file containing edges data
with open('tbledges_202305111432.csv', newline='') as csvfile:
    # Create a csv reader object with specified fieldnames for each column in the csv file
    reader = csv.DictReader(csvfile, fieldnames=[
                            'id', 'first_node_id', 'second_node_id', 'length_in_meters'])
    # Skip first row of csv file (header)
    next(reader)
    # Iterate through each row in csv file after header
    for row in reader:
        # Get node ids for both end points of edge and distance between them as a float value from the row just read
        node1 = row['first_node_id']
        node2 = row['second_node_id']
        distance = float(row['length_in_meters'])

        # Add nodes to graph if not already present
        if node1 not in graph:
            graph[node1] = {}
        if node2 not in graph:
            graph[node2] = {}

        # Add the edge between nodes with given distance in both directions since it is an undirected graph
        graph[node1][node2] = distance
        graph[node2][node1] = distance

coordinates = {}  # create an empty dictionary to store id as key and coordinates as value
with open('tblnodes_202305111211.csv', newline='') as csvfile:
    id_list = []  # create empty list to store ids
    # read the CSV file using DictReader function with specified fields
    reader = csv.DictReader(csvfile, fieldnames=['id', 'x', 'y'])
    reader_id_list = csv.reader(csvfile)
    next(reader)  # skip header row in the csv file

    for row in reader:
        id_ = row['id']  # retrieve id from 'id' column of the current row
        # retrieve x coordinate from 'x' column of the current row and convert it into a float datatype
        x = float(row['x'])
        # retrieve y coordinate from 'y' column of the current row and convert it into a float datatype
        y = float(row['y'])
        # add the tuple of x and y coordinates as value to the 'id' key in the dictionary
        coordinates[id_] = (x, y)
        # append the 'id' field value into the id_list


# Open the CSV file
with open('tblnodes_202305111211.csv', 'r') as file:
    # Create a CSV reader object
    reader = csv.reader(file)

    # Skip the header row (if it exists)
    next(reader)

    # Create an empty list to store the column values
    id_list = []

    # Iterate over the rows in the CSV file
    for row in reader:
        # Append the value in the "id" column to the list
        id_list.append(row[0])


for node in graph:  # Loop through nodes in the graph.
    # Loop through neighbors of current node.
    for neighbor, distance in graph[node].items():
        # Add reverse edge
        if neighbor not in graph:  # If neighbor is not already a key in the graph dictionary:
            # Create a new empty dictionary for that neighbor.
            graph[neighbor] = {}
        # Add a new key-value pair to neighbor's dictionary mapping node to distance.
        graph[neighbor][node] = distance


# define the straight-line distance heuristic function


def straight_line_distance(node, goal):
    # This function calculates the straight line distance between two nodes using their coordinates.
    # It takes two arguments, node and goal, which are the starting and ending points respectively.
    # The coordinates of each point are stored in the dictionary named 'coordinates'.
    # The x and y values of both points are extracted from this dictionary using their respective keys.
    # These values are then used to calculate the Euclidean distance between the two points and return it as output.

    x1, y1 = coordinates[node]  # retrieve coordinates of start node
    x2, y2 = coordinates[goal]  # retrieve coordinates of goal node
    # Calculate Euclidean distance between two nodes
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


# Function to find the straight line distance between two points
def straight_line_distance(node, goal):
    # get the coordinates (x and y) of both points from the dictionary 'coordinates'
    x1, y1 = coordinates[node]
    x2, y2 = coordinates[goal]
    # Return the calculated distance using the formula
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

# A* search algorithm implementation


def astar(graph, start, target):
    # Here we use a heap which is used as a priority queue to store  nodes based on cost/distance
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
        if current == target:
            path = []
            while current != start:
                path.insert(0, current)
                current = came_from[current]
            path.insert(0, start)
            return (g_score[target], path)
        for neighbor, cost in graph[current].items():
            # the tentative_g_score represents the combination of current node's g score and the cost of the edge between current and neighbor node
            tentative_g_score = g_score[current] + cost
            # if the tentative g score is lower than the current g score of the neighbor node, then update the variables accordingly.
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + \
                    straight_line_distance(neighbor, target)
                heapq.heappush(heap, (f_score[neighbor], neighbor))
    # If there is no path found to the target i.e heap is empty we return infinity as the distance and an empty list as the path
    return (float('inf'), [])


def dijkstra(graph, start, target):
    # This function implements Dijkstra's algorithm to find the shortest path from start node to target node.
    # The input parameters:
    # graph - dictionary object where keys are nodes and values are dictionaries that consist of edges (keys) and their associated costs (values)
    # start - starting node in the graph
    # target - target node in the graph
    # Create a heap to store visited nodes
    heap = [(0, start)]

    # Initialize visited set, predecessors dictionary, and distances dictionary
    visited = set()
    predecessors = {}
    distances = {node: float('inf') for node in graph}
    distances[start] = 0

    # While heap is not empty
    while heap:
        # Pop smallest cost node from heap
        (cost, node) = heapq.heappop(heap)

        # If node is not in visited set
        if node not in visited:
            visited.add(node)

            # If node equals target node, return cost and path from start to target
            if node == target:
                path = []
                while node != start:
                    path.insert(0, node)
                    node = predecessors[node]
                path.insert(0, start)
                return (cost, path)

            # For each adjacent node and cost
            for adj_node, adj_cost in graph[node].items():
                # If neighbor has not been visited
                if adj_node not in visited:
                    new_cost = distances[node] + adj_cost

                    # Updates distances if there is a shorter path to the adjacent node
                    if new_cost < distances[adj_node]:
                        distances[adj_node] = new_cost
                        predecessors[adj_node] = node

                        # Pushes the new path's cost & node into heap
                        heapq.heappush(heap, (new_cost, adj_node))

    # If target is not found, return infinity and empty path list
    return (float('inf'), [])

# This function implements Depth First Search(DFS) in a graph
# with a given start node and end node. It uses the concept of
# recursion to traverse through each node in the graph.
# we use this function to chech if the path exist or not
# This loop iterates through every node in the graph dictionary

for node in graph:
    # This nested loop iterates through all of this node's neighbors and the distance to each neighbor
    for neighbor, distance in graph[node].items():
        # If the neighbor is not present in the graph dictionary, it adds an empty dictionary to that node.
        if neighbor not in graph:
            graph[neighbor] = {}
        # It then adds a reverse edge from the neighbor to the current node with the same distance.
        graph[neighbor][node] = distance


# This code uses environment variables as inputs for Start and Target nodes
# If no value is provided then it will use default start node: '1224' and target node: '134'
Start = os.environ.get('Start') or '344'
Target = os.environ.get('Target') or '134'

# We'll record the current time before starting the algorithm.


def print_shortest_path(alg_name, shortest_path_cost, shortest_path, start_time):
    end_time = time.time()
    # Calculate the elapsed time.
    elapsed_time = end_time - start_time
    print(f"Measuring the shortest path between {Start} and {Target}...")
    print(f"{alg_name} algorithm")
    # Print out the amount of time it took for the algorithm to run.
    print(f"Elapsed time: {elapsed_time} seconds")
    # Print out the shortest path cost and path
    print("Shortest Path Cost:", shortest_path_cost)
    print("Shortest Path:", shortest_path)


start_time = time.time()

shortest_path_cost, shortest_path = astar(graph, Start, Target)
print_shortest_path('Astar', shortest_path_cost, shortest_path, start_time)

start_time = time.time()

shortest_path_cost, shortest_path = dijkstra(graph, Start, Target)
print_shortest_path('Dijkstra', shortest_path_cost, shortest_path, start_time)
