import random

from game import Directions
from game import Agent
from game import Actions
from pacman import GameState
from search import AStarSearchNode


class RandomAgent(Agent):

    def getAction(self, state):
        "The agent receives a GameState (defined in pacman.py)."
        direction = random.choice([Directions.WEST,Directions.EAST,Directions.SOUTH,Directions.NORTH])
        while direction not in state.getLegalPacmanActions():
            direction = random.choice([Directions.WEST,Directions.EAST,Directions.SOUTH,Directions.NORTH])
        return direction


class RandomHeuristicAgent(Agent):

    def __init__(self):
        self.map = []
        pass

    def registerInitialState(self, state):
        """
        This is the first time that the agent sees the layout of the game
        board. Here, we choose a path to the goal. In this phase, the agent
        should compute the path to the goal and store it in a local variable.
        All of the work is done in this method!

        state: a GameState object (pacman.py)
        :type state: GameState
        """
        self.map = state.getWalls()
        self.map_height = self.map.height
        self.map_width = self.map.width
        self.food_heuristic = [[state.hasFood(j,i) for j in range(0, self.map_width)] for i in range(0, self.map_height)]
        self.ghost_heuristic = [[0 for j in range(0, self.map_width)] for i in range(0, self.map_height)]

    def compute_ghost_heuristic(self, state):
        """

        :type state: GameState
        """
        self.ghost_heuristic = [[0 for j in range(0, self.map_width)] for i in range(0, self.map_height)]
        for ghost_index in range(1,state.getNumAgents()):
            position = state.getGhostPosition(ghost_index)
            position = (int(position[0]), int(position[1]))
            ghost_state = state.getGhostState(ghost_index)
            heuristic_val = 1
            self.ghost_heuristic[int(position[1])][int(position[0])] += heuristic_val
            for x in range(position[0]-1, position[0]+1):
                for y in range(position[1]-1, position[1]+1):
                    self.ghost_heuristic[y][x] += heuristic_val
            for x in range(position[0]-2, position[0]+2):
                for y in range(position[1]-2, position[1]+2):
                    self.ghost_heuristic[y][x] += heuristic_val

    def getAction(self, state):
        "The agent receives a GameState (defined in pacman.py)."
        self.compute_ghost_heuristic(state)
        self.food_heuristic = [[int(state.hasFood(j,i)) for j in range(0, self.map_width)] for i in range(0, self.map_height)]
        self.heuristic = [[-5 * self.food_heuristic[y][x] + 5*self.ghost_heuristic[y][x] for x in range(0, self.map_width)] for y in range(0, self.map_height)]

        best = 999999999
        best_choice = Directions.STOP
        for move in self.getPossibleActions(state):
            coordinates = [state.getPacmanPosition(), Actions.directionToVector(move)]
            final = [sum([int(z) for z in x]) for x in zip(*coordinates)]
            if self.heuristic[final[1]][final[0]] < best:
                best = self.heuristic[final[1]][final[0]]
                best_choice = move
            elif self.heuristic[final[1]][final[0]] == best and random.randint(1,3) == 2:
                best_choice = move
        return best_choice

    def getPossibleActions(self, state):
        actions = state.getLegalPacmanActions()
        actions.remove(Directions.STOP)
        return actions


class StarSearch(Agent):

    def __init__(self):
        Agent.__init__(self)
        self.recompute_delay = 0
        self.visited = []
        self.actions = []
        self.map = []
        self.map_height = 0
        self.map_width = 0
        self.ghost_cost = []
        self.food_heuristic = []
        self.f = dict()
        self.current_target = (1, 1)

    def registerInitialState(self, state):
        """
        This is the first time that the agent sees the layout of the game
        board. Here, we choose a path to the goal. In this phase, the agent
        should compute the path to the goal and store it in a local variable.
        All of the work is done in this method!

        state: a GameState object (pacman.py)
        :type state: GameState
        """
        self.map = state.getWalls()
        self.map_height = self.map.height
        self.map_width = self.map.width
        self.visited = [[0 for j in range(0, self.map_width)] for i in range(0, self.map_height)]
        self.ghost_danger = []


    def compute_ghost_heuristic(self, state):
        """

        :type state: GameState
        """
        self.ghost_danger = []
        self.ghost_cost = [[0 for j in range(0, self.map_width)] for i in range(0, self.map_height)]
        for ghost_index in range(1,state.getNumAgents()):
            position = state.getGhostPosition(ghost_index)
            position = (int(position[0]), int(position[1]))
            ghost_state = state.getGhostState(ghost_index)
            heuristic_val = 1
            if ghost_state.scaredTimer is not 0:
                heuristic_val -= ghost_state.scaredTimer
            self.ghost_cost[int(position[1])][int(position[0])] += 4*heuristic_val
            near_set = [(int(position[0]),int(position[1]))]
            for x in range(position[0]-1, min([position[0]+2,self.map_width])):
                for y in range(position[1]-1, min([position[1] + 2, self.map_height])):
                    if (x,y) not in near_set:
                        near_set.append((x,y))
                        self.ghost_cost[y][x] += 3 * heuristic_val
            for x in range(max(0,position[0]-3), min([position[0]+4,self.map_width])):
                for y in range(max(0, position[1] - 3), min([position[1] + 4, self.map_height])):
                    if (x, y) not in near_set:
                        near_set.append((x, y))
                        if self.ghost_cost[y][x] == 0:
                            self.ghost_cost[y][x] += heuristic_val
            self.ghost_danger += near_set

    def getAction(self, state):

        """

        :type state: GameState
        """
        self.compute_ghost_heuristic(state)

        pos = state.getPacmanPosition()
        self.visited[int(pos[1])][int(pos[0])]+=1
        if len(self.actions)>0:
            pop = self.actions.pop(0)
            if self.recompute_delay < 250 and pop in state.getLegalPacmanActions():
                self.recompute_delay += 1
                if pos not in self.ghost_danger:
                    print pop
                    return pop
                else:
                    print 'Oh shit!'
            self.actions = []
        self.recompute_delay = 0
        self.food_heuristic = [[int(state.hasFood(j,i)) for j in range(0, self.map_width)] for i in range(0, self.map_height)]
        for x in range(0, self.map_width):
            for y in range(0, self.map_height):
                self.f[(x,y)] = -1000* self.food_heuristic[y][x] + 0.1*self.ghost_cost[y][x] + 100*self.manhattanHeuristic(pos, (x,y))

        if not state.hasFood(self.current_target[0], self.current_target[1]) or self.current_target in self.ghost_danger:
            best_value = min(self.f.itervalues())
            best_solutions = [k for k, v in self.f.iteritems() if v == best_value]
            distance_for_solutions = dict((k,self.manhattanHeuristic(pos,k)) for k in best_solutions)
            best_value = min(distance_for_solutions.itervalues())
            best_solutions = [k for k, v in distance_for_solutions.iteritems() if v == best_value]
            self.current_target = random.choice(best_solutions)

        self.actions = self.aStarSearch(state.getPacmanPosition(), self.current_target, state)
        if len(self.actions) == 0: return Directions.STOP
        pop=self.actions.pop(0)
        print pop
        coordinates = [state.getPacmanPosition(), Actions.directionToVector(pop)]
        final = [sum([int(z) for z in x]) for x in zip(*coordinates)]
        a = self.getPossibleActions(state)
        scared_of = 1
        while self.ghost_cost[final[1]][final[0]] > scared_of:
            scared_of = self.ghost_cost[final[1]][final[0]]
            self.actions = []
            a.remove(pop)
            if len(a) == 0:
                return Directions.STOP
            pop = random.choice(a)
            coordinates = [state.getPacmanPosition(), Actions.directionToVector(pop)]
            final = [sum([int(z) for z in x]) for x in zip(*coordinates)]

        return pop

    def manhattanHeuristic(self, start, goal):
        "The Manhattan distance heuristic for a PositionSearchProblem"
        xy1 = start
        xy2 = goal
        return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])

    def getPossibleActions(self, state):
        actions = state.getLegalPacmanActions()
        actions.remove(Directions.STOP)
        return actions

    def aStarSearch(self, start, goal, state, depth=100):
        """Search the node that has the lowest combined cost and heuristic first."""
        closed_set = []
        opened_set = [AStarSearchNode(start, 0, self.food_heuristic[start[1]][start[0]])]
        path_storage = dict()
        iteration = 0
        while len(opened_set) > 0:

            current = min(opened_set)
            opened_set.remove(current)

            if current == goal or iteration >= depth:
                position = current.position
                path = []
                while path_storage.has_key(position):
                    path.append(path_storage[position][1])
                    position = path_storage[position][0]
                final_path = []
                for action in reversed(path):
                    final_path.append(action)
                return final_path
            iteration += 1
            closed_set.append(current)
            for neighbor in self.get_successors(state, current.position):
                node = neighbor[0]
                in_closed_set = node in closed_set
                in_opened_set = node in opened_set
                if in_closed_set and closed_set[closed_set.index(node)].cost < current.cost:
                    continue
                elif in_opened_set and opened_set[opened_set.index(node)].cost < current.cost:
                    continue
                else:
                    v_cost = current.cost + neighbor[2]
                    v_heuristic = v_cost + -3*self.food_heuristic[node[1]][node[0]] + self.manhattanHeuristic(start, node)
                    if in_closed_set or in_opened_set:
                        if in_opened_set:
                            v = opened_set[opened_set.index(node)]
                        else:
                            v = closed_set[closed_set.index(node)]
                        v.cost = v_cost
                        v.heuristic = v_heuristic
                        v.move = neighbor[1]
                        if in_closed_set and v_cost < 10000:
                            closed_set.remove(v)
                            opened_set.append(v)
                    else:
                        v = AStarSearchNode(node, v_cost, v_heuristic, neighbor[1])
                        opened_set.append(v)
                    path_storage[v.position] = (current.position, v.move)
        return []

    def get_successors(self, state, position):
        """

        :param state: GameState
        :param position: int[]
        """
        successors = []
        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x, y = position
            dx, dy = Actions.directionToVector(action)
            nextx, nexty = int(x + dx), int(y + dy)
            if not state.hasWall(nextx, nexty):
                nextState = (nextx, nexty)
                if self.ghost_cost[nexty][nextx] > 0:
                    cost = 1 + 10 * self.ghost_cost[nexty][nextx]
                else:
                    cost = 1
                successors.append((nextState, action, cost))
        return successors
