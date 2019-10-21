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
            self.tickets = tickets
            self.BFS(init[i], self.goal[i])
            paths.append(backtrack(self.goal[i]))
            
        #Validate longest path
        longest = 0
        for path in paths:
            if longest < len(path):
                longest = len(path)
        
        #SDFS for all paths with specific size adapting to longer sizes
        valid_paths = []
        for i in range(len(paths)):
            self.current_goal = self.goal[i]
            valid_paths.append(self.SDFS(longest, init[i], limitexp, limitdepth, tickets))


        print("----------------------")
        p(valid_paths)
        print("----------------------")

        return paths


    def BFS(self, node, goal):
        self.parents = [[-math.inf, 0, [0,0,0]] for x in range(len(self.model) + 1)] #create parents array
        self.distances = [-math.inf] * (len(self.model)+1) #create distances array
        self.visited = [False] * (len(self.model)+1)
        Q = queue.Queue()
        Q.put(node)
        self.distances[node] = 0
        self.visited[node] = True
        self.parents[node][2] = self.tickets
        
        while not Q.empty():
            u = Q.get()
            i = 0
            for transition in self.model[u]: #check possible paths
                #Validate child
                child = transition[1]
                if self.parents[u][2][transition[0]] <= 0:
                    continue
        
                if not self.visited[child]: #not visited
                    self.visited[child] = True
                    self.distances[child] = self.distances[u] + 1
                    
                    #update tickets
                    tickets = list(self.parents[u][2])
                    tickets[transition[0]] -= 1
                    self.parents[child] = [u, i, tickets]
                    
                    #stop earlier
                    if child == goal:
                        return child
                    Q.put(child)
                i += 1



    def SDFS(self, longest_path, start_node, limitexp=2000, limitdepth=10, tickets=[math.inf, math.inf, math.inf]):
        def DLS(depth, node, time):
            if depth == 0:
                if node == self.current_goal:
                    print('Goal found! ' + str(node))
                    return []
                else:
                    return None
            elif depth > 0:
                new_node = RecursiveNode(node)
                for child in self.model[node]:
                    found = DLS(depth - 1, child[1], time + 1)
                    if found != None:
                        new_node.add(found)
                return new_node

        #return [[[], [start_node]]] + DLS(longest_path, start_node, 0)[0]
        return DLS(longest_path, start_node, 0)



class RecursiveNode:
    def __init__(self, value):
        self.children = []
        self.value = value

    def add(self, child):
        self.children.append(child)

    def __repr__(self):
        return str(self.value) + ' [ ' + str(self.children) + ' ]'