# short-path-python-algorithms
Shortest Path Python Algorithm: A* vs Dijkstra
When trying to find the shortest path between two points in a graph or map, two popular algorithms that can be used are the A* and Dijkstra algorithms. In this project, we compare and analyze the performance of these two algorithms in Python.

Installation
Make sure you have Python 3 installed. Then, clone or download this repository.
git clone https://github.com/your-username/shortest-path.git

Algorithms
A* Algorithm
A* algorithm is a heuristic search algorithm that uses heuristics to evaluate which paths to explore during search. It expands nodes in the order of their heuristic evaluation function f(n) = g(n) + h(n) where:

g(n) stands for the cost (so far) to reach the node n
h(n) stands for the estimated cost to get from the node n to the target.
f(n) stands for total estimated cost of path through node n
The bigger the f(n), the less likely the node will be expanded. The lower the f(n), the more likely it will be expanded.

Dijkstra Algorithm
Dijkstra algorithm is a shortest-path algorithm that starts at the source node and explores all neighboring nodes, essentially discovering the entire graph. During exploration, it keeps track of the minimum distance thus far to each node explored.

Conclusion
Both algorithms have their advantages and disadvantages. A* algorithm is more efficient when it comes to complicated graphs while Dijkstra's algorithm is a simpler implementation and easier to understand and program. So, the best approach depends on the context and the needs of the problem at hand.

In this project, we have shown how to implement both algorithms in Python and compared their execution times using the same data set.
