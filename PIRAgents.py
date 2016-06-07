import random

from game import Directions
from game import Agent
from game import Actions
from pacman import GameState


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

    def getAction(self, state):
        "The agent receives a GameState (defined in pacman.py)."
        return random.choice(state.getLegalPacmanActions())
