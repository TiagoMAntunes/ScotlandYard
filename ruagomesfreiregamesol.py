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

        def clean_paths(dirty_paths, depth_limit, init):
            paths = []
            
            final = [[[], [self.current_goal]]]
            node = self.current_goal
            i = depth_limit
            while True:
                # one path found, prepare to look for more
                print("node: ", node)
                print("dirty_node: ", dirty_paths[node])
                if i == 0:
                    if node == init:
                        final = [[[], [node]]] + final
                        paths.append(final)
 
                    else:
                        node = self.current_goal
                        i = depth_limit
                        final = [[0, [self.current_goal]]]

                # no more paths terminating in goal
                if dirty_paths[self.current_goal][depth_limit] == []:
                    break;

                else:
                    connection = dirty_paths[node][i].pop()
                    print("depth = ", i, "connection: ", connection)
                    print("from ", node, " to ", connection[0])
                    node = connection[0]
                    transport = self.model[node][connection[1]][0]
                    final[0][0].append(transport)
                    final = [[[], [node]]] + final
                    print("final = ", final)
                    i -= 1

            print("proper paths: ", paths)
            return paths


       
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
            print(path)

        print("len of longest: ", longest)

        #SBFS for all paths with specific size adapting to longer sizes
        valid_paths = []
        n = len(self.goal)
        if n > 1:
            aux_depth = longest
            while True:
                print("aux_depth = ", aux_depth)
                for i in range(n):
                    self.current_goal = self.goal[i]
                    all_possible = self.LBFS(aux_depth, [[],init[i]], limitexp, limitdepth, tickets)
                    
                    # if path with len n not found, search for paths with len n+1
                    if all_possible == None:
                        aux_depth += 1
                        break

                    clean_paths(all_possible, aux_depth, init[i])

                    break
                print("----------------------")
                print(valid_paths)
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


    def LBFS(self, depth_limit, start_transition, limitexp=2000, limitdepth=10, tickets=[math.inf, math.inf, math.inf]):
        self.parents = [{} for x in range(len(self.model) + 1)] #create parents array
        self.distances = [-math.inf] * (len(self.model)+1) #create distances array
        node = start_transition[1]
        Q = queue.Queue()
        Q.put(node)
        self.distances[node] = 0        #{depth: [(pai, index), (...)]}
        self.parents[node][0] = 0
        while not Q.empty():
            u = Q.get()
            i = 0
            for transition in self.model[u]: #check possible paths
                #Validate child
                child = transition[1]
                if self.distances[u] == depth_limit:
                    continue
        
                else: 
                    self.distances[child] = self.distances[u] + 1

                    if self.distances[child] not in self.parents[child]:
                        self.parents[child][self.distances[child]] = [(u, i)]

                    else:
                        self.parents[child][self.distances[child]].append((u, i))
                    
                    Q.put(child)
                i += 1  

        # verify that path to goal with len = depth_limit exists
        if depth_limit in self.parents[self.current_goal]:
            print("goal: ", self.current_goal)
            print("limit: ", self.parents[self.current_goal])
            return self.parents

        return None




class RecursiveNode:
    def __init__(self, value):
        self.children = []
        self.value = value

    def add(self, child):
        self.children.append(child)

    def __repr__(self):
        return str(self.value) + ' [ ' + str(self.children) + ' ]'