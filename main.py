# ***********************************************************************************************************
# Graph Search - Shortest Path - Single Source Shortest Path
# (Reference: http://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=GRL_1_A)
# ***********************************************************************************************************
# Solution: This python program is a solution,
# written by YM Li, in Fall 2018.
# ***********************************************************************************************************

import heapq, sys


class PriorityQueue:
    """
      Applies the priority queue data structure.
      Items inserted is in the order of values related to them.
    """
    def  __init__(self):
        self.heap = []

    def push(self, item, priority):
        entry = (priority, item)
        heapq.heappush(self.heap, entry)

    def pop(self):
        priority, item = heapq.heappop(self.heap)
        return (item, priority)

    def isEmpty(self):
        return len(self.heap) == 0


class Graph:
    """
    Creates a graph object.
    """
    def __init__(self, vertex_num, edge_num, root):
        """
        Initializes the graph.
        :param (int) vertex_num - number of vertices
        :param (int) edge_num - number of edges
        :param (int) root - the number representing root node
        """
        self.vertex_num = vertex_num
        self.root = root

        # creates an array of tuples s.t. at vertices[vertex], all [another vertex attached, edge cost] are include.
        self.vertices = []
        # creates an array of ints including the shortest path
        self.path = []
        for i in range(0, vertex_num):
            self.vertices.append([])
            #print(self.vertices)
            self.path.append(100000) # inf = 100000 for this specific question
            #print(self.path)

    def addEdge(self, s, t, d):
        """
        Adds edge reading from user input into the array vertices.
        :param (int) s - source vertices of i-th edge
        :param (int) t - target vertices of i-th edge
        :param (int) d - the cost of the i-th edge
        """
        self.vertices[s].append((t, d))

    def dijkstra(self):
        """
        Applies and modifies Dijkstra algorithm to find the shortest path.
        """
        # initializes current queue and path
        q = PriorityQueue()
        q.push(self.root, 0)
        self.path[self.root] = 0

        # updates queues and path
        while not q.isEmpty():
            u, u_cost = q.pop()
            #print(u)
            #print(u_cost)
            for info in self.vertices[u]:
                t = info[0] # target vertex attached to vertex u
                d = info[1] # cost of edge of (u,t)
                if self.path[t] > u_cost + d:
                    self.path[t] = u_cost + d
                    q.push(t, self.path[t])


if __name__ == "__main__":
    # gets the 1st line of user input and initializes the graph
    print("Enter your inputs here:")
    input_graph = sys.stdin.readline()
    input_graph = input_graph.split(" ")
    V = int(input_graph[0])
    E = int(input_graph[1])
    r = int(input_graph[2])
    problem = Graph(V, E, r)

    # get lines of input and initializes the edges of graph
    for i in range(0, E):
        input_edges = sys.stdin.readline()
        input_edges = input_edges.split(" ")
        s = int(input_edges[0])
        t = int(input_edges[1])
        d = int(input_edges[2])
        problem.addEdge(s, t, d)

    # solves the problem
    problem.dijkstra()

    # print out the result
    for v in problem.path:
        if v == 100000:
            print("INF")
        else:
            print(v)
