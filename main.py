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

    def __init__(self, vertex_num, edge_num, root):
        self.vertex_num = vertex_num
        self.root = root
        self.vertices = []
        self.dist = []
        for i in range(0, vertex_num):
            self.vertices.append([])
            #print(self.vertices)
            self.dist.append(100000)
            #print(self.dist)

    def addEdge(self, s, t, d):
        self.vertices[s].append((t, d))

    def dijkstra(self):

        q = PriorityQueue()
        q.push(self.root, 0)
        self.dist[self.root] = 0

        while not q.isEmpty():
            u, u_cost = q.pop()
            #print(u)
            #print(u_cost)
            for edge in self.vertices[u]:
                t = edge[0]
                #print(t)
                d = edge[1]
                if self.dist[t] > u_cost + d:
                    self.dist[t] = u_cost + d
                    q.push(t, self.dist[t])


if __name__ == "__main__":
    print("Enter your inputs here:")
    input_graph = sys.stdin.readline()
    input_graph = input_graph.split(" ")
    V = int(input_graph[0])
    E = int(input_graph[1])
    r = int(input_graph[2])
    problem = Graph(V, E, r)

    for i in range(0, E):
        input_edges = sys.stdin.readline()
        input_edges = input_edges.split(" ")
        s = int(input_edges[0])
        t = int(input_edges[1])
        d = int(input_edges[2])
        problem.addEdge(s, t, d)

    problem.dijkstra()

    for v in problem.dist:
        if v == 100000:
            print("INF")
        else:
            print(v)
