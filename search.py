# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""
import copy

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    :type problem:
    """
    position = problem.getStartState()
    visited = util.Counter()
    visited[position] = 1
    for childNode in problem.getSuccessors(position):
        result = dfs_recursive(problem, visited, [], childNode)
        if result != -1:
            return result
    return []


def dfs_recursive(problem, visited, path, node):
    if visited[node[0]] == 0:
        path = copy.copy(path)
        visited[node[0]] = 1
        path.append(node[1])
        if problem.isGoalState(node[0]):
            return path
        else:
            for childNode in problem.getSuccessors(node[0]):
                result = dfs_recursive(problem, visited, path, childNode)
                if result != -1:
                    return result
            return -1
    return -1


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first.
    """
    queue = util.Queue()
    visited = [problem.getStartState()]

    queue.push((problem.getStartState(),[]))

    while not queue.isEmpty():
        (current, path) = queue.pop()
        for childNode in problem.getSuccessors(current):
            if childNode[0] not in visited:
                visited.append(childNode[0])
                if problem.isGoalState(childNode[0]):
                    return path + [childNode[1]]
                else:
                    queue.push((childNode[0], path+[childNode[1]]))

    return []


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    queue = util.PriorityQueue()
    visited = [problem.getStartState()]

    queue.push((problem.getStartState(),[]),0)

    while not queue.isEmpty():
        (current, path) = queue.pop()
        for childNode in problem.getSuccessors(current):
            if childNode[0] not in visited:
                visited.append(childNode[0])
                new_path = path + [childNode[1]]
                if problem.isGoalState(childNode[0]):
                    return new_path
                else:
                    queue.push((childNode[0], new_path), problem.getCostOfActions(new_path))

    return []

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def inversed_heuristic(heuristic, state, problem):
    h = heuristic(state, problem)
    if h==0:
        return float("inf")
    else:
        return 1/h

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    closedSet = []
    opened_set = util.PriorityQueueWithFunction(lambda s: inversed_heuristic(heuristic, s[0], problem))
    opened_set_states = []
    opened_set.push((problem.getStartState(),'',0))
    opened_set_states.append(problem.getStartState())
    cameFrom = dict()

    gScore = util.CounterWithInfinityDefault()
    gScore[problem.getStartState()] = 0

    while not opened_set.isEmpty():
        current = opened_set.pop()
        opened_set_states.remove(current[0])
        if problem.isGoalState(current[0]):
            path = [current[1]]
            while cameFrom.has_key(current[0]):
                current = cameFrom.get(current[0])
                path.append(current[1])
            return path.reverse()
        closedSet.append(current)
        for neighbor in problem.getSuccessors(current[0]):
            if neighbor in closedSet:
                continue
            tentative_gScore = gScore[current[0]] + neighbor[2]
            if neighbor[0] not in opened_set_states:
                opened_set.push(neighbor)
                opened_set_states.append(neighbor[0])
            elif tentative_gScore >= gScore[neighbor]:
                continue
            cameFrom[neighbor[0]] = current
            gScore[neighbor[0]] = tentative_gScore
    return []


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
