import math
import pickle
import time
from pprint import pprint as p

class SearchProblem:

    def __init__(self, goal, model, auxheur=[]):
        self.goal = goal
        self.model = model
        self.coords = auxheur
        self.max_dist = math.sqrt((self.coords[31][0] - self.coords[112][0])**2 + (self.coords[31][1] - self.coords[112][1])**2)

    def cost(self, node, succ):
        dist = math.sqrt((self.coords[node][0] - self.coords[succ][0])**2 + (self.coords[node][1] - self.coords[succ][1])**2)
        return dist / self.max_dist

    def h(self, start_node, goal_index=0):
        """ Heuristica: distancia no mapa / max dist entre 2 pontos """
        return self.cost(start_node, self.goal[goal_index])      

    def search(self, init, limitexp=2000, limitdepth=10, tickets=[math.inf, math.inf, math.inf]):
        n_detectives = range(len(self.goal))
        shortest_paths = [0] * len(n_detectives)
        for i in n_detectives:
            self.current_goal = self.goal[i]
            print(self.current_goal)
            shortest_paths[i] = self.IDA([init[i]], limitexp, limitdepth, tickets)
        p(shortest_paths)

    def IDA(self, init, limitexp, limitdepth, tickets):
        bound = self.h(init[0]) #initial limit
        path = [[[], init]]         # path = [[[transport], [node]], [...]]
        while True:
            t = self.find(path, 1, bound, tickets)
            if t == 'FOUND':
                return path
            elif t == math.inf:
                return []
            bound = t
        return path

    def find(self, path, g, bound, tickets = [math.inf, math.inf, math.inf]):
        node = path[-1][-1][0]
        f = g + self.h(node)
        if f > bound:
            return f
        if node == self.current_goal:
            return 'FOUND'
            
        min = math.inf
        for succ in self.model[node]:
            if succ not in path and tickets[succ[0]] > 0:
                proper_succ = [[succ[0]], [succ[1]]]
                path.append(proper_succ)
                tickets[succ[0]] -= 1
                t = self.find(path, f + self.cost(node, succ[1]), bound, tickets)
                if t == 'FOUND':
                    return 'FOUND'
                if t < min:
                    min = t
                path.pop()
                tickets[succ[0]] += 1

        return min

