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

        for path in paths:
            print("path:", path)


        #SBFS for all paths with specific size adapting to longer sizes
        n = len(self.goal)
        valid_paths = []
        if n > 1:
            aux_depth = longest
            not_done = True
            while not_done:
                for i in range(n):
                    self.current_goal = self.goal[i]
                    all_possible = []
                    # all_possible = self.LBFS(aux_depth, [[],init[i]], limitexp, limitdepth, tickets)
                    self.recBFS(aux_depth, [0,init[i]], 1, [], all_possible)
                    valid_paths.append(all_possible)
                    not_done = False
                    if len(all_possible) <= 1:
                        not_done = True
                        aux_depth += 1
                        break

            print("----------------------")
            print(valid_paths)
            print("----------------------")
                

        return paths


    def BFS(self, node, goal):
        self.parents = [[-math.inf, 0] for x in range(len(self.model) + 1)] #create parents array
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
                #Validate child
                child = transition[1]
                if not self.visited[child]: #not visited
                    self.visited[child] = True
                    self.distances[child] = self.distances[u] + 1
                    self.parents[child] = [u, i]
                    
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


    def recBFS(self, depth_limit, transition, current_depth, path, paths, limitexp=2000, limitdepth=10):
    #    print("======")
    #    print("depth: ", current_depth)
    #    print("node: ", transition[1])
    #    print("goal: ", self.current_goal)
    #    print("transition: ", transition)
    #    print("path before: ", path)
    #    print("======")

        if (current_depth == depth_limit and transition[1] == self.current_goal):
            path2 = path + [[[transition[0]], [transition[1]]]]
            paths.append(path2)

        if current_depth == 1:
            path = [[[],[transition[1]]]]
            for child in self.model[transition[1]]:
                self.recBFS(depth_limit, child, current_depth+1, path, paths, limitexp, limitdepth)

        elif current_depth < depth_limit:
            path = path[:current_depth-1]
            path += [[[transition[0]], [transition[1]]]]
            for child in self.model[transition[1]]:
                self.recBFS(depth_limit, child, current_depth+1, path, paths, limitexp, limitdepth)





class RecursiveNode:
    def __init__(self, value):
        self.children = []
        self.value = value

    def add(self, child):
        self.children.append(child)

    def __repr__(self):
        return str(self.value) + ' [ ' + str(self.children) + ' ]'