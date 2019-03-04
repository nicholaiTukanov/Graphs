from collections import defaultdict as dd


class Graph:

    # Constructor
    def __init__(self):
        self.graph = dd(dict)
        self.search_tree = dd(dict)
        self.visited = dd(bool)

    # Print tree
    def print_tree(self):
        print("Tree produce by graph search")
        for v in self.search_tree:
            if (self.search_tree[v]):
                print("{} : {}".format(v, self.search_tree[v]))
        print("")

    # Print graph
    def print_graph(self):
        print("Graph as adj list")
        for v in self.graph:
            print("{} : {}".format(v, self.graph[v]))
        print("")

    # Add graph edge
    def add_graph_edge(self, v, u, length):
        self.graph[v].update({u: length})

    # Add graph edges where l is an adj list for vertex v
    def add_graph_edges(self, v, l):
        if v not in self.graph:
            self.graph[v] = {}
        for u in l:
            self.graph[v].update({u: l[u]})

    # Add tree edge
    def add_tree_edge(self, v, u, length):
        self.search_tree[v].update({u: length})

    # Add vertex to graph
    def add_graph_vertex(self, v):
        self.graph[v] = {}

    # Add vertex to tree
    def add_tree_vertex(self, v):
        self.search_tree[v] = {}

    # Initialize visited dict where key is vertex and value is flag
    def init_visited(self):
        for v in self.graph:
            self.visited[v] = False

    # insert new vertex in sorted order
    def insertsorted(self, queue, u, delta):
        inserted = False
        for i in range(0, len(queue)):
            if (delta[queue[i]] > delta[u]):
                inserted = True
                queue.insert(i, u)
        if not inserted:
            queue.append(u)
        return queue

    # Breadth first search
    def BFS(self, s):

        # Init queue
        queue = [s]

        # Set start vertex to true
        self.init_visited()
        self.visited[s] = True

        # Loop until queue is empty
        while queue:

            # Remove first vertex in queue
            v = queue.pop(0)

            # Add vertex to tree
            self.add_tree_vertex(v)

            # For each edge(v,u), if u is not visited, add to queue, set
            # visited to true, and add the edge to the tree.
            for u in self.graph[v]:
                if (self.visited[u] == False):
                    queue.append(u)
                    self.visited[u] = True
                    self.add_tree_edge(v, u, 1)

        self.print_tree()

    # Breadth first search
    def DFS(self, s):

        # Set start vertex to true
        self.visited[s] = True

        for u in self.graph[s]:
            if (self.visited[u] == False):
                self.visited[u] = True
                self.add_tree_edge(s, u, 1)
                self.DFS(u)

    # Bellman ford
    def BellmanFord(self, s):

        # set lengths to every vertex in graph as infinity
        delta = {}
        for v in self.graph:
            delta[v] = 9999
        # set starting vertex length to 0
        delta[s] = 0

        i = 1
        while (i <= len(self.graph) - 1):

            for v in self.graph:
                for u in self.graph[v]:
                    delta[u] = min(delta[u], delta[v] + self.graph[v][u])

            i += 1

        print("Output array of Bellman ford\n{}".format(delta))

    # Dijkstra using priority queue
    def Dijkstra(self, s):

        # Init queue
        queue = [s]
        delta = {}
        for v in self.graph:
            delta[v] = 9999
        delta[s] = 0

        self.init_visited()
        self.visited[s] = True

        # Loop until queue is empty
        while queue:

            # Remove first vertex in queue
            v = queue.pop(0)

            # Add vertex to tree
            self.add_tree_vertex(v)

            # For each edge(v,u), if u is not visited, add to queue, set
            # visited to true, and add the edge to the tree.
            for u in self.graph[v]:
                # set delta[u] to be the minimum possible path length from
                # s to u
                delta[u] = min(delta[u], delta[v] + self.graph[v][u])
                # If the vertex has never been visited set the flag to
                # true, add the newly visited vertex in sorted order to the
                # queue
                if (self.visited[u] == False):
                    queue = self.insertsorted(queue, u, delta)
                    self.visited[u] = True

        print("Output array of Dijkstra\n{}".format(delta))