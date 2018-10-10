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
        self.count = 0

    def push(self, item, priority):
        entry = (priority, self.count, item)
        heapq.heappush(self.heap, entry)
        self.count += 1

    def pop(self):
        (priority, _, item) = heapq.heappop(self.heap)
        return (item, priority)

    def isEmpty(self):
        return len(self.heap) == 0


class Graph:

    class Edge:
        def __init__(self, left, right, cost):
            self.left = left
            self.right = right
            self.cost = cost

    def __init__(self, vertex_num, edge_num, root):
        self.vertex_num = vertex_num
        self.edge_num = edge_num
        self.root = root
        self.vertices = []
        self.dist = []
        for i in range(0, vertex_num):
            self.vertices.append([])
            self.dist.append(100000)

    def addEdge(self, left, right, cost):
        self.vertices[left].append(self.Edge(left, right, cost))

    def dijkstra(self):

        q = PriorityQueue()
        q.push(self.root, 0)
        self.dist[self.root] = 0

        while q:
            u_cost, u = q.pop()
            if self.dist[u] < u_cost:
                continue
            for e in self.vertices[u]:
                if self.dist[e.right] > u_cost + e.cost:
                    self.dist[e.right] = u_cost + e.cost
                    q.push(e.right, self.dist[e.right])


if __name__ == "__main__":
    print("Enter your inputs here:")
    input = sys.stdin.readline()
    V = int(input[0])
    E = int(input[2])
    r = int(input[4])
    problem = Graph(V, E, r)

    for i in range(0, E):
        input2 = sys.stdin.readline()
        left = int(input2[0])
        right = int(input2[2])
        cost = int(input2[4])
        problem.addEdge(left, right, cost)

    problem.dijkstra()

    for v in problem.dist:
        if v == 999:
            print("INF")
        else:
            print(v)
