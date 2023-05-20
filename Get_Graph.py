import csv
'''
The given code imports the csv module to read data from a CSV file. It initializes an empty dictionary called "graph". The CSV file is opened and read using DictReader from the csv module.

The next() function is used to skip the header row as it is not required in our graph data. After that, for each row in the CSV, the code extracts values of first_node_id, second_node_id, and length_in_meters.

Then, it checks if node1 and node2 are present in the graph dictionary keys. If they are not, then they are added to the dictionary with a blank value initialization.

After that, undirected adjacency list representation is created using node IDs as keys and their respective distances as values. The final graph object is stored in the graph dictionary.

The output prints the graph in the form of nested dictionaries representing an adjacency list with nodes as keys and their adjacent nodes with distances as values.
'''
graph = {}

with open('tbledges_202305111432.csv', newline='') as csvfile:
    reader = csv.DictReader(csvfile, fieldnames=[
                            'id', 'first_node_id', 'second_node_id', 'length_in_meters'])

    next(reader)  # It is used to skip header row

    for row in reader:  # Iterating each row in csv file starting from second row

        # The first node ID for current edge
        node1 = row['first_node_id']
        # The second node ID for current edge
        node2 = row['second_node_id']
        # Edge distance between given node1 id and node2 id
        distance = float(row['length_in_meters'])

        # To handle directed graph storage, if nodes not present in graph dictionary, add them as keys (<-if needed->)
        if node1 not in graph:
            graph[node1] = {}

        if node2 not in graph:
            graph[node2] = {}

        # Creating undirected adjacency list representation using node IDs and their respective distances
        graph[node1][node2] = distance
        graph[node2][node1] = distance


print(graph)
