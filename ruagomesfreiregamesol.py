import math
import pickle
import time


class SearchProblem:

    def __init__(self, goal, model, auxheur=[]):
        self.goal = goal
        self.model = model
        self.coords = auxheur

    def cost(self, node, succ):
        return math.sqrt((self.coords[node][0] - self.coords[succ][0])**2 + (self.coords[node][1] - self.coords[succ][1])**2)

    def h(self, start_node, goal_index=0):
        """ Heuristica: distancia no mapa """
        return self.cost(start_node, self.goal[goal_index])

    def search(self, init, limitexp=2000, limitdepth=10, tickets=[math.inf, math.inf, math.inf]):
        bound = self.h(init[0])
        # user init como path
        while True:
            t = self.find(init, 0, bound)
            if t == 'FOUND':
                return init
            elif t == math.inf:
                return []
            bound = t

    def find(self, path, g, bound):
        node = path[-1]
        f = g + self.h(node)
        if f > bound:
            return f
        if node in self.goal:
            return 'FOUND'
        min = math.inf
        for succ in self.model[node]:
            if succ not in path:
                path.append(succ[1])
                t = self.find(path, f + self.cost(node, succ[1]), bound)
                if t == 'FOUND':
                    return 'FOUND'
                if t < min:
                    min = t
                path.pop()
        return min
