import math
import pickle
import time
from pprint import pprint as p
from queue import Queue
from itertools import product

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

    def search(self, init, limitexp=2000, limitdepth=10, tickets=[math.inf, math.inf, math.inf]):
        #minimum distance to perform
        minimum_distance = max([self.cost(init[x], x) for x, _ in enumerate(init)])
        self.ida_star(minimum_distance, init, tickets)
        print('Number of expansions {}'.format(self.expansions))
        

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

    def ida_star(self, start_size, init, tickets):
        bound = start_size
        path = [[[-1, init[i]]] for i in range(len(self.goal))] 
        print(path)
        while True:
            t = self.find(path, 0, bound, tickets)
            print('out with t {}'.format(t))
            if t == 'FOUND':
                break
            bound +=1
            print('Bound increased')
        print('Path is: {} with a bound of {}'.format(path, bound))
        return path

    def find(self, path, current_cost, bound, tickets):
        #print('Path is {}'.format(path))
        node = [path[x][-1][1] for x in range(len(path))]
        #estimates to get to all points
        f = [(self.cost(node[i], i) + current_cost) for i in range(len(path))]
        
        #have we reached the goal?
        if all( [(lambda x, i: x == self.goal[i])(node[i], i) for i in range(len(node))]):
            return 'FOUND'

        #check if bound has been exceeded by any of them
        if any( [(lambda x: x > bound)(f[i]) for i in range(len(path))]):
            return f

        #Count expansions
        self.expansions += len(node)
        
        #possible combinations to try
        combinations = list(product( *[map(tuple,self.model[node[i]]) for i in range(len(path))] )) 
        
        #sort by minimal f
        combinations.sort(key = lambda x: sum([self.cost(x[i][1], i) for i in range(len(x))]))
        

        for poss_path in combinations:
            
            if len(set(poss_path)) != len(poss_path):
                #collision handling!
                continue
            new_tickets = [] + tickets

            for a in poss_path:
                new_tickets[a[0]] -= 1

            if (any(map((lambda x: x < 0), new_tickets))):
                continue

            for i in range(len(path)): #adds path to try again
                path[i].append([poss_path[i][0],poss_path[i][1]])
            

            t = self.find(path, current_cost + 1, bound, new_tickets)
            if t == 'FOUND':
                return 'FOUND'
           
            for i in range(len(path)):
                path[i].pop()
            pass
        return 0
