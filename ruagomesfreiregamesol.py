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
        self.busy_times = {}

    def cost(self, node, succ):
        dist = math.sqrt((self.coords[node][0] - self.coords[succ][0])**2 + (self.coords[node][1] - self.coords[succ][1])**2)
        return dist / self.max_dist

    def h(self, start_node, goal_index=0):
        """ Heuristica: distancia no mapa / max dist entre 2 pontos """
        return self.cost(start_node, self.goal[goal_index])      

    def search(self, init, limitexp=2000, limitdepth=10, tickets=[math.inf, math.inf, math.inf]):
        def add_positions(list):
            for i in range(len(list)):
                if i not in self.busy_times:
                    self.busy_times[i] = [list[i][1][0]]
                else:
                    self.busy_times[i].append(list[i][1][0])

        n_detectives = range(len(self.goal))
        shortest_paths = [0] * len(n_detectives)
        for i in n_detectives:
            self.current_goal = self.goal[i]
            shortest_paths[i] = self.IDA([init[i]], limitexp, limitdepth, tickets)
        
        #find the limiting path, the one that is the longest between the shortest paths
        longest_index = 0
        for i, path in enumerate(shortest_paths[1:]):
            if len(path) > len(shortest_paths[longest_index]):
                longest_index = i

        #longest_index holds the index of the agent with the longest path
        add_positions(shortest_paths[longest_index])
        for i in range(len(shortest_paths)):
            if (i == longest_index):
                continue
            self.current_goal = self.goal[i]
            print('Current busy positions: ' + str(self.busy_times))
            shortest_paths[i] = self.IDFS(shortest_paths[longest_index], init[i], limitexp, limitdepth, tickets)
            add_positions(shortest_paths[i])

        print('Found paths:')
        p(shortest_paths)

        res = []

        for i in range(len(shortest_paths[0])):
            l = [[],[]]
            for j in range(len(shortest_paths)):
                l[0] += shortest_paths[j][i][0]
                l[1] += shortest_paths[j][i][1]
            res.append(l)
        print('Finished parsing:')
        p(res)
        return res

        

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

    def IDFS(self, longest_path, start_node, limitexp=2000, limitdepth=10, tickets=[math.inf, math.inf, math.inf]):
        def DLS(depth, node):
            if depth == 0:
                if node == self.current_goal:
                    return [], True
                else:
                    return None, True 
            elif depth > 0:
                any_remaining = False
                for child in self.model[node]:
                    found, remaining = DLS(depth - 1, child[1])
                    if found != None:
                        proper_succ = [[child[0]], [child[1]]]
                        return [proper_succ] + found, True
                    if remaining:
                        any_remaining = True
                return None, any_remaining
    
        return [[[], [start_node]]] + DLS(len(longest_path) - 1, start_node)[0]

