import heapq
import time
import os
'''
This is a Python script that implements Dijkstra's algorithm for finding the shortest path between two nodes in a graph.

The script starts by defining the graph as a dictionary of nodes, where each node is itself a dictionary of its neighbors and the distance to each neighbor. It then adds reverse edges to the graph by iterating over each node and each neighbor, adding an edge from the neighbor to the node with the same distance.

Next, it defines a straight-line distance heuristic function, which calculates the Euclidean distance between two nodes using their coordinates.

The main part of the script is the implementation of Dijkstra's algorithm in the dijkstra() function. This function takes the graph, start node, and target node as input, and returns the shortest path and its cost as a tuple.

The function starts by initializing a heap with the start node, and setting the distance to all other nodes as infinity. As each node is visited, its distance is updated, and its neighbors are added to the heap if they have not already been explored. The algorithm returns when it reaches the target node or when all reachable nodes have been explored.

Finally, the os module is used to get the start and target nodes as environment variables, and the time module is used to time the execution of the algorithm. The shortest path and its cost are printed to the console, along with the elapsed time.

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
        # If the neighbor is not already in the graph, add it.
        if neighbor not in graph:
            graph[neighbor] = {}
        # Add an edge from the neighbor to the node, with the same distance.
        graph[neighbor][node] = distance

# This function defines a straight-line distance heuristic.



# This function implements Dijkstra's algorithm.

def dijkstra(graph, start, target):
    # Create a heap to store the nodes to be visited.
    heap = [(0, start)]
    # Create a set to track the nodes that have already been visited.
    visited = set()
    # Create a dictionary to store the predecessors of each node.
    predecessors = {}
    # Create a dictionary to store the distances to each node.
    distances = {node: float('inf') for node in graph}
    # Set the distance to the start node to 0.
    distances[start] = 0

    # While there are still nodes to be visited:
    while heap:
        # Get the node with the lowest distance from the heap.
        (cost, node) = heapq.heappop(heap)

        # If the node has already been visited, skip it.
        if node in visited:
            continue

        # Mark the node as visited.
        visited.add(node)

        # If the node is the target, return the path to the target.
        if node == target:
            path = []
            while node != start:
                path.insert(0, node)
                node = predecessors[node]
            path.insert(0, start)
            return (cost, path)

        # For each neighbor of the node:
        for adj_node, adj_cost in graph[node].items():
            # If the neighbor has not already been visited:
            if adj_node not in visited:
                # Calculate the new cost to reach the neighbor.
                new_cost = distances[node] + adj_cost

                # If the new cost is less than the current cost to reach the neighbor:
                if new_cost < distances[adj_node]:
                    # Update the distance to the neighbor.
                    distances[adj_node] = new_cost

                    # Update the predecessor of the neighbor.
                    predecessors[adj_node] = node

                    # Add the neighbor to the heap.
                    heapq.heappush(heap, (new_cost, adj_node))

    # If the target node is not reachable, return (float('inf'), []).
    return (float('inf'), [])


# Get the start and target nodes from the environment.
Start = os.environ.get('Start', 'C')
Target = os.environ.get('Target', 'F')


# Start the timer.
start_time = time.time()

# Find the shortest path from the start node to the target node.
shortest_path_cost, shortest_path = dijkstra(graph, Start, Target)

# Stop the timer.
end_time = time.time()

# Calculate the elapsed time.
elapsed_time = end_time - start_time

# Print the elapsed time.
print(f"Elapsed time: {elapsed_time} seconds")

# Print the shortest path cost.
print("Shortest Path Cost:", shortest_path_cost)  

# Print the shortest path.
print("Shortest Path:", shortest_path)
