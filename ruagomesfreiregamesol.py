import math
import pickle
import time
from pprint import pprint as p
from queue import Queue
import itertools

class SearchProblem:

    def __init__(self, goal, model, auxheur=[]):
        self.goal = goal
        self.model = model
        self.coords = auxheur
        self.heur = [[] for _ in range(len(goal)) ]

        #pre-process all goals
        for i in range(len(goal)):
            self.heur[i] = self.min_distances(i)


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
        return distances[origin] + 1


    def format(self, parent, child):
        return [[[self.model[parent][child][0]], [self.model[parent][child][1]]]]


    def search(self, init, limitexp=2000, limitdepth=10, tickets=[math.inf, math.inf, math.inf], anyorder=False):
        def backtrack(node):
            path = []
            while self.parents[node][0] > 0:
                path = self.format(self.parents[node][0], self.parents[node][1]) + path
                node = self.parents[node][0]
            path = [[[], [node]]] + path
            return path            
            
        
        def validate_tickets(paths):
            """
            Given a set of paths, verifies if there are enough tickets
            available to persue it.
            """
            aux_tickets = [] + self.tickets
            path_size = len(paths[0])
            for path in paths:
                for i in range(1, path_size):
                    if aux_tickets[path[i][0][0]] == 0:
                        return False

                    aux_tickets[path[i][0][0]] -= 1

            return True


        def format_paths(paths):
            len_paths = len(paths[0])
            formated = [[[], [paths[0][0][1][0], paths[1][0][1][0], paths[2][0][1][0]]]]
            for i in range(1, len_paths):
                t0 = paths[0][i][0][0]
                t1 = paths[1][i][0][0]
                t2 = paths[2][i][0][0]
                n0 = paths[0][i][1][0]
                n1 = paths[1][i][1][0]
                n2 = paths[2][i][1][0]
                formated += [[[t0, t1, t2], [n0, n1, n2]]]

            return formated

        
        def goal_score(init, goal):
            """
            Given a set of goals and initial positions, scores the former
            according to the maximum length of the shortest path of any 
            initial node to the corresponding goal.
            """
            max = -math.inf
            for i, pos in enumerate(init):
                if (self.cost(pos, i) > max):
                    max = self.cost(pos, i)
            return max



        if (anyorder):
            self.goal_perms = {}
            for comb in itertools.permutations(self.goal, len(self.goal)):
                self.goal_perms[comb] = goal_score(init, comb);

            least_scored = min(self.goal_perms, key=self.goal_perms.get)
            self.goal = least_scored

            # get the minimum distance that they ALL need to do
            longest = min(
                max([self.cost(init[x], y) for x in range(len(init))] for y in range(len(init))))

        else:
            longest = max([self.cost(init[x], x)
                                    for x in range(len(init))])


        self.tickets = tickets
        self.anyorder = anyorder


        # recBFS for all paths with specific size adapting to longer sizes
        n = len(self.goal)   
        aux_depth = longest
        done = False
        has_tickets = False
        total_exp = 0;
        while not done or (self.tickets[0] != math.inf and not has_tickets):
            sameLen_paths = []
            for i in range(n):
                self.current_goal = self.goal[i]

                all_possible = []
                exp = [1]
                self.recBFS(aux_depth, exp, [0,init[i]], 1, [], all_possible, i)
                total_exp += exp[0]

                # if path with len n doesnt exist
                done = True
                has_tickets = False
                if len(all_possible) <= 1:
                    done = False
                
                # if path with len n exists, verify if enough tickets
                if n == 1:
                    if done and self.tickets[0] != math.inf:
                        for path in all_possible:
                            if validate_tickets([path]):
                                print("exp0 = ", total_exp)
                                # if only one agent, return found path
                                return path

                    # if path doesnt exist or not enough tickets, look for a new path
                    if not done or (self.tickets[0] != math.inf and not has_tickets):
                        aux_depth += 1
                        break

                    print("exp1 = ", total_exp)
                    return all_possible[0]
                    

                # path exists, 3 detectives
                elif n == 3:
                    if not done:
                        aux_depth += 1
                        # need to update goal's score and persue new minimal goal
                        if self.anyorder:
                            #print("----")
                            #print("prev goal: ", self.goal)
                            self.goal_perms[self.goal] += 1
                            self.goal = min(self.goal_perms, key=self.goal_perms.get)
                            aux_depth = self.goal_perms[self.goal]
                            #print("updated perms: ", self.goal_perms)
                            #print("new goal: ", self.goal)
                            #print("----")
                        break

                    sameLen_paths += [all_possible]

            # Choose paths with no collisions and enough tickets
            if (n == 3 and done):
                #print("====")
                #print("sameLen_paths: ", sameLen_paths)
                #print("====")
                longest = len(sameLen_paths[0][0])
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

                            # if paths 1 and 2 collide, go to next path2
                            if collide1:
                                break

                            # if path 3 collides with 1 and/or 2, go to next path3
                            if collide2:
                                continue

                            # no collisions found
                            # if no limit of tickets, return first path found
                            if (tickets[0] == math.inf):
                                print("exp2 = ", total_exp)
                                return format_paths([path1, path2, path3])

                            # if tickets are limited, make sure to have enough
                            else:
                                if validate_tickets([path1, path2, path3]):
                                    print("exp3 = ", total_exp)
                                    return format_paths([path1, path2, path3])

                # couldnt find path without collisions and enough tickets
                done = False

                # need to update goal's score and persue new minimal goal
                if (self.anyorder):
                    #print("====")
                    #print("prev goal: ", self.goal)
                    self.goal_perms[self.goal] += 1
                    self.goal = min(self.goal_perms, key=self.goal_perms.get)
                    aux_depth = self.goal_perms[self.goal]
                    #print("updated perms: ", self.goal_perms)
                    #print("new goal: ", self.goal)
                    #print("====")



    def BFS(self, node, goal):
        self.parents = [[-math.inf, 0] for x in range(len(self.model) + 1)] #create parents array
        self.distances = [-math.inf] * (len(self.model)+1) #create distances array
        self.visited = [False] * (len(self.model)+1)
        Q = Queue()
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


    def recBFS(self, depth_limit, exp, transition, current_depth, path, paths, goal_index, limitexp=2000, limitdepth=10):
        #print("======")
        #print("limit: ", depth_limit)
        #print("depth: ", current_depth)
        #print("node: ", transition[1])
        #print("goal: ", self.current_goal)
        #print("transition: ", transition)
        #print("path before: ", path)
        #print("======")

        moves_left = depth_limit - current_depth

        if current_depth == depth_limit and transition[1] == self.current_goal:
            path2 = path + [[[transition[0]], [transition[1]]]]
            paths.append(path2)

        if current_depth == 1:
            path = [[[],[transition[1]]]]
            for child in self.model[transition[1]]:
                if self.heur[goal_index][child[1]] <= moves_left:
                    exp[0] += 1
                    self.recBFS(depth_limit, exp, child, current_depth+1, path, paths, goal_index, limitexp, limitdepth)

        elif current_depth < depth_limit:
            path = path[:current_depth-1]
            path += [[[transition[0]], [transition[1]]]]
            for child in self.model[transition[1]]:
                if self.heur[goal_index][child[1]] <= moves_left:
                    exp[0] += 1
                    self.recBFS(depth_limit, exp, child, current_depth+1, path, paths, goal_index, limitexp, limitdepth)