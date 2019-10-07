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
        path = [[[], init]]         # path = [[[transport], [node]], [...]]
        n_detectives = len(init)
        if (n_detectives == 1 and tickets[0] == math.inf):
            while True:
                t = self.find_1(path, 0, bound)
                if t == 'FOUND':
                    return path
                elif t == math.inf:
                    return []
                bound = t

        elif (n_detectives == 1 and tickets[0] != math.inf):
            while True:
                t = self.find_2(path, 0, bound, tickets)
                if t == 'FOUND':
                    return path
                elif t == math.inf:
                    return []
                bound = t


    def find_1(self, path, g, bound):
        node = path[-1][-1][0]
        f = g + self.h(node)
        if f > bound:
            return f
        if node in self.goal:
            return 'FOUND'
        min = math.inf
        for succ in self.model[node]:
            if succ not in path:
                proper_succ = [[succ[0]], [succ[1]]]
                path.append(proper_succ)
                t = self.find_1(path, f + self.cost(node, succ[1]), bound)
                if t == 'FOUND':
                    return 'FOUND'
                if t < min:
                    min = t
                path.pop()
                
        return min


    def find_2(self, path, g, bound, tickets):
        node = path[-1][-1][0]
        f = g + self.h(node)
        if f > bound:
            return f
        if node in self.goal:
            return 'FOUND'
        min = math.inf
        for succ in self.model[node]:
            if succ not in path and tickets[succ[0]] > 0:
                proper_succ = [[succ[0]], [succ[1]]]
                path.append(proper_succ)
                tickets[succ[0]] -= 1
                t = self.find_2(path, f + self.cost(node, succ[1]), bound, tickets)
                if t == 'FOUND':
                    return 'FOUND'
                if t < min:
                    min = t
                path.pop()
                tickets[succ[0]] += 1

        return min
