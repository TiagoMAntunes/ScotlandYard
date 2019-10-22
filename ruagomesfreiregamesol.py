import math
import pickle
import time
from pprint import pprint as p
from queue import Queue

expansions = 0

class SearchProblem:

    def __init__(self, goal, model, auxheur=[]):
        self.goal = goal
        self.model = model
        self.coords = auxheur
        self.expansions = 0
        self.heur = [[] for _ in range(len(goal)) ]

        #pre-process all goals

        for i in range(len(goal)):
            self.heur[i] = self.min_distances(i)
        
        print('Distances calculated')
        print(self.heur)

    def search(self, init, limitexp=2000, limitdepth=10, tickets=[math.inf, math.inf, math.inf]):
        #minimum distance to perform
        minimum_distance = max([self.cost(init[x], x) for x, _ in enumerate(init)])
        

    def min_distances(self, goal_index):
        #BFS all nodes
        origin = self.goal[goal_index]
        distances = [-1] * len(self.model)
        parents = [-1] * len(self.model)
        q = Queue()
        q.put(origin)
        
        distances[origin] = 0
        while not q.empty():
            visiting = q.get()
            for path in self.model[visiting]:
                child = path[1]
                if distances[child] != -1:
                    continue
                distances[child] = distances[visiting] + 1
                
                q.put(child)
        
        return distances

    def cost(self, origin, destiny_index):
        distances = self.heur[destiny_index] # get current objectives distances
        return distances[origin]

    