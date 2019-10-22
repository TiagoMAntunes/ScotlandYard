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
            
        
        def validate_tickets(paths):
            print("paths to validate: ", paths)
            print("tickets: ", self.tickets)
            aux_tickets = [] + self.tickets
            path_size = len(paths[0])
            for path in paths:
                for i in range(1, path_size):
                    if aux_tickets[path[i][0][0]] == 0:
                        return False

                    aux_tickets[path[i][0][0]] -= 1

                return True
            return False


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


        # recBFS for all paths with specific size adapting to longer sizes
        n = len(self.goal)        
        aux_depth = longest
        not_done = True
        has_tickets = False
        while not_done or not has_tickets:
            sameLen_paths = []
            for i in range(n):
                self.current_goal = self.goal[i]

                all_possible = []
                self.recBFS(aux_depth, [0,init[i]], 1, [], all_possible)

                # if path with len n doesnt exist
                not_done = False
                has_tickets = False
                if len(all_possible) <= 1:
                    not_done = True
                
                # if path with len n exists, verify if enough tickets
                if not not_done and self.tickets[0] != math.inf:
                    for path in all_possible:
                        if validate_tickets([path]):
                            has_tickets = True
                            if n == 1:
                                return path

                if not_done or (self.tickets[0] != math.inf and not has_tickets):
                    aux_depth += 1
                    break
            #    print("----------------------")
            #    print(all_possible)
            #    print("----------------------")

                # if it exists, add it
                sameLen_paths += [all_possible]

        #print("----------------------")
        #print(sameLen_paths)
        #print("----------------------")


        # Choose paths with no collisions and enough tickets
        valid_paths = []
        print("-----")
        for path1 in sameLen_paths[0]:
            for path2 in sameLen_paths[1]:
                for path3 in sameLen_paths[2]:

                    # if they collide, dont proceed with search
                    collide1 = collide2 = False
                    for i in range(longest):
                        if path1[i][1] == path2[i][1]:
                            collide1 = True
                            break

                        elif path1[i][1] == path3[i][1] or path2[i][1] == path3[i][1]:
                            collide2 = True
                            break

                        if collide1 or collide2:
                            break

                    if collide1:
                        break

                    if collide2:
                        continue

                    # if no limit of tickets, return first path found
                    if (tickets[0] == math.inf):
                        return [path1, path2, path3]

                    else:
                        if validate_tickets([path1, path2, path3]):
                            valid_paths += [path1, path2, path3]


        print("valid: ", valid_paths)
        print("-----")






                

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
        #print("======")
        #print("depth: ", current_depth)
        #print("node: ", transition[1])
        #print("goal: ", self.current_goal)
        #print("transition: ", transition)
        #print("path before: ", path)
        #print("======")

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