import math
import pickle
import time
from pprint import pprint as p
import queue

class SearchProblem:

    def __init__(self, goal, model, auxheur=[]):
        self.goal = goal
        self.model = model
        self.coords = auxheur

    def format(self, parent, child):
        
        return [[[self.model[parent][child][0]], [self.model[parent][child][1]]]]

    def search(self, init, limitexp=2000, limitdepth=10, tickets=[math.inf, math.inf, math.inf]):
        def backtrack(node):
            path = []
            while self.parents[node][0] > 0:
                path = self.format(self.parents[node][0], self.parents[node][1]) + path
                node = self.parents[node][0]
            path = [[[], [node]]] + path
            return path

        #BFS on all agents
        paths = []
        for i in range(len(self.goal)):
            self.BFS(init[i], self.goal[i])
            paths.append(backtrack(self.goal[i]))
        #Validate longest path

        p(paths)

        #SDFS for all paths with specific size adapting to longer sizes


    def BFS(self, node, goal):
        self.parents = [[-math.inf, 0] * x for x in range(len(self.model) + 1)] #create parents array
        self.distances = [-math.inf] * (len(self.model)+1) #create distances array
        self.visited = [False] * (len(self.model)+1)
        Q = queue.Queue()
        Q.put(node)
        self.distances[node] = 0
        self.visited[node] = True
        while not Q.empty():
            u = Q.get()
            i = 0
            for transition in self.model[u]: #check possible paths
                child = transition[1]
                if not self.visited[child]: #not visited
                    self.visited[child] = True
                    self.distances[child] = self.distances[u] + 1
                    self.parents[child] = [u, i]
                    if child == goal:
                        return child
                    Q.put(child)
                i += 1


