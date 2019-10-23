import math
import pickle
import time
from pprint import pprint as p
from queue import Queue
from itertools import product, permutations

expansions = 0


class SearchProblem:

    def __init__(self, goal, model, auxheur=[]):
        self.goal = goal
        self.model = model
        self.coords = auxheur
        self.expansions = 0
        self.heur = [[] for _ in range(len(goal))]

        # pre-process all goals

        for i in range(len(goal)):
            self.heur[i] = self.min_distances(i)

        print('Distances calculated')

    def search(self, init, limitexp=2000, limitdepth=10, tickets=[math.inf, math.inf, math.inf], anyorder=False):
        self.anyorder = anyorder
        # minimum distance to perform
        if anyorder:
            # get the minimum distance that they ALL need to do
            minimum_distance = min(
                max([self.cost(init[x], y) for x in range(len(init))] for y in range(len(init))))
        else:
            minimum_distance = max([self.cost(init[x], x)
                                    for x in range(len(init))])

        print('Minimum distance is: {}'.format(minimum_distance))
        final_path = self.ida_star(minimum_distance, init, tickets)
        print('Number of expansions {}'.format(self.expansions))

        return final_path

    def min_distances(self, goal_index):
        # BFS all nodes to find minimum distance for A* heuristic
        origin = self.goal[goal_index]
        distances = [-1] * len(self.model)
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
        # get current objectives distances
        distances = self.heur[destiny_index]
        return distances[origin]

    def format_path(self, paths, formated):
        length = len(paths[0])
        for path in paths:
            for i in range(length):
                if i > 0:
                    formated[i][0].append(path[i][0])

                formated[i][1].append(path[i][1])
        print(formated)
        return formated

    def ida_star(self, start_size, init, tickets):
        bound = start_size
        path = [[[-1, init[i]]] for i in range(len(self.goal))]

        while True:
            t = self.find(path, 0, bound, tickets)
            if t == 'FOUND':
                break
            bound += 1
            print('Bound increased')
        print('Path is: {} with a bound of {}'.format(path, bound))

        return self.format_path(path, [[[], []] for x in range(len(path[0]))]) 

    def find(self, path, current_cost, bound, tickets):
        node = [path[x][-1][1] for x in range(len(path))]

        # estimates to get to all points
        f = [(self.cost(node[i], i) + current_cost) for i in range(len(path))]

        # have we reached the goal?
        if self.anyorder and any(map(lambda x: x == node, permutations(self.goal))) or self.goal == node:
            return 'FOUND'

        # check if bound has been exceeded by any of them
        if any([(lambda x: x > bound)(f[i]) for i in range(len(path))]):
            return f

        # Count expansions
        self.expansions += len(node)

        # possible combinations to try
        combinations = list(
            product(*[map(tuple, self.model[node[i]]) for i in range(len(path))]))

        # sort by minimal f
        combinations.sort(key=lambda x: sum(
            [self.cost(x[i][1], i) for i in range(len(x))]))

        for poss_path in combinations:

            if len(set(poss_path)) != len(poss_path):
                # collision handling!
                continue

            new_tickets = [] + tickets

            for a in poss_path:
                new_tickets[a[0]] -= 1

            # Too many tickets used?
            if (any(map((lambda x: x < 0), new_tickets))):
                continue

            for i in range(len(path)):  # adds path to be tried
                path[i].append([poss_path[i][0], poss_path[i][1]])

            t = self.find(path, current_cost + 1, bound, new_tickets)

            if t == 'FOUND':
                return 'FOUND'

            for i in range(len(path)):  # path not found, remove from path
                path[i].pop()
            pass
        return 0
                


