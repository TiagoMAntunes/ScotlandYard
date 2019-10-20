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
        self.collisions = [{}, {}, {}]	# [{t0: [[dx, dy], node], t1: ...}, {...}, {...}]

    def cost(self, node, succ):
        dist = math.sqrt((self.coords[node-1][0] - self.coords[succ-1][0])**2 + (self.coords[node-1][1] - self.coords[succ-1][1])**2)
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

        def get_collisions(n_detectives, paths):
            for i in range(len(n_detectives) -1):
                for j in range(path_lengths[i]):
                    node = shortest_paths[i][j][1][0]
                    if (j <= path_lengths[i+1] and node == shortest_paths[i+1][j][1][0]):
                        if j not in self.collisions[i]:
                            self.collisions[i+1][j] = [[i], node]
                        else:
                            self.collisions[i+1][j][0].append(i)

        n_detectives = range(len(self.goal))
        shortest_paths = [0] * len(n_detectives)
        for i in n_detectives:
            self.current_goal = self.goal[i]
            shortest_paths[i] = self.IDA([init[i]], limitexp, limitdepth, tickets, i)
            print("shortest", i, " ", shortest_paths[i])

        # calculate each path's length
        path_lengths = []
        for path in shortest_paths:
        	path_lengths.append(len(path))

        # get collisions for each path
        get_collisions(n_detectives, shortest_paths)
        print("collisions = ", self.collisions)

        # recalculate paths with collisions, avoiding them
        uncolliding_paths = [0] * len(n_detectives)
        for i in n_detectives:
            if self.collisions[i]:
                uncolliding_paths[i] = self.IDA([init[i]], limitexp, limitdepth, tickets, i)

            else:
                uncolliding_paths[i] = shortest_paths[i]

            add_positions(uncolliding_paths[i])

        print("not colliding = ", uncolliding_paths)

        #find the limiting path, the one that is the longest between the shortest paths
        longest_index = 0
        for i, path in enumerate(shortest_paths[1:]):
            if len(path) > len(shortest_paths[longest_index]):
                longest_index = i

        #longest_index holds the index of the agent with the longest path
        # add_positions(shortest_paths[longest_index])
        for i in range(len(shortest_paths)):
            if (i == longest_index):
                continue
            self.current_goal = self.goal[i]
            print("current goal = ", self.current_goal)
            print('Current busy positions: ' + str(self.busy_times))
            shortest_paths[i] = self.IDFS(shortest_paths[longest_index], init[i], limitexp, limitdepth, tickets)
            add_positions(shortest_paths[i])


        res = []

        for i in range(len(shortest_paths[0])):
            l = [[],[]]
            for j in range(len(shortest_paths)):
                l[0] += shortest_paths[j][i][0]
                l[1] += shortest_paths[j][i][1]
            res.append(l)
        return res

        

    def IDA(self, init, limitexp, limitdepth, tickets, detective):
        bound = self.h(init[0]) 	#initial limit
        path = [[[], init]]         # path = [[[transport], [node]], [...]]
        while True:
            t = self.find(path, 1, bound, 0, detective, tickets)
            if t == 'FOUND':
                return path
            elif t == math.inf:
                return []
            bound = t
        return path

    def find(self, path, g, bound, time, detective, tickets = [math.inf, math.inf, math.inf]):
        node = path[-1][-1][0]
        f = g + self.h(node)
        if f > bound:
            return f
        if node == self.current_goal:
            return 'FOUND'
            
        min = math.inf

        for succ in self.model[node]:
            # detective cant go to unreachable places, nor use transport without tickets
            # detective cant go to places where it has collisions
            # detective cant create new collisions
            if succ not in path and tickets[succ[0]] > 0 and \
            (not self.collisions[detective] or \
            time+1 not in self.collisions[detective] or \
            self.collisions[detective][time+1][1] != succ[1]) and \
            (time+1 not in self.busy_times or \
            succ[1] not in self.busy_times[time+1]):

                proper_succ = [[succ[0]], [succ[1]]]
                path.append(proper_succ)
                tickets[succ[0]] -= 1
                t = self.find(path, f + self.cost(node, succ[1]), bound, time+1, detective, tickets)
                if t == 'FOUND':
                    return 'FOUND'
                if t < min:
                    min = t
                path.pop()
                tickets[succ[0]] += 1

        return min

    def IDFS(self, longest_path, start_node, limitexp=2000, limitdepth=10, tickets=[math.inf, math.inf, math.inf]):
        def DLS(depth, node, time):
            if depth == 0:
                if node == self.current_goal:
                    return [], True
                else:
                    return None, True 
            elif depth > 0:
                any_remaining = False
                for child in self.model[node]:
                    if child[1] in self.busy_times[time+1]: #busy position at the time
                        continue
                    found, remaining = DLS(depth - 1, child[1], time + 1)
                    if found != None:
                        proper_succ = [[child[0]], [child[1]]]
                        return [proper_succ] + found, True
                    if remaining:
                        any_remaining = True
                return None, any_remaining

        return [[[], [start_node]]] + DLS(len(longest_path)-1, start_node, 0)[0]

