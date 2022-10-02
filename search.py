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

from fileinput import nextfile
from typing import Set
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

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    #Inicialización de estructuras

    from util import Stack #LIFO
    path = Stack() #Aca voy guardando el camino desde el comienzo hasta mi estado actual
    fringe = Stack() #Aca guardo los estados a expandir
    final_path = list() #Aca voy a guardar las direcciones finales para llegar
    visited_pos = list() #Aca guardo los nodos que ya visité para no entrar en un loop 
    
    #Comienzo desde el estado inicial
    next_pos = problem.getStartState()
    fringe.push((next_pos, final_path))
    while not fringe.isEmpty() and not problem.isGoalState(next_pos): #Loop hasta que no llegue al estado deseado y siga teniendo elementos
        next_pos, final_path = fringe.pop()           # Saco del fringe el proximo nodo que quiero visitar
        if next_pos not in visited_pos:       
            visited_pos.append(next_pos)     #Voy marcando los nodos visitados
            if problem.isGoalState(next_pos):
                return final_path
            else:
                next_pos = problem.getSuccessors(next_pos) #Voy recorriendo los sucesores
                for next_node, dir, cost in next_pos:
                    fringe.push((next_node,final_path + [dir]))        # Ubico en el fringe el nodo adyacente                            
                    path.push(final_path + [dir] ) # Guardo el camino + la nueva dirección
    return final_path
    #util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    from util import Queue #FIFO
    path = Queue() #Aca voy guardando el camino desde el comienzo hasta mi estado actual
    fringe = Queue() #Aca guardo los estados a expandir
    final_path = list() #Aca voy a guardar las direcciones finales para llegar
    visited_pos = list() #Aca guardo los nodos que ya visité para no entrar en un loop 
    
    #Comienzo desde el estado inicial
    next_pos = problem.getStartState()
    fringe.push((next_pos, final_path))
    while not fringe.isEmpty() and not problem.isGoalState(next_pos): #Loop hasta que no llegue al estado deseado y siga teniendo elementos
        next_pos, final_path = fringe.pop()           # Saco del fringe el proximo nodo que quiero visitar
        if next_pos not in visited_pos:       
            visited_pos.append(next_pos)     #Voy marcando los nodos visitados
            if problem.isGoalState(next_pos):
                return final_path
            else:
                next_pos = problem.getSuccessors(next_pos) #Voy recorriendo los sucesores
                for next_node, dir, cost in next_pos:
                    fringe.push((next_node,final_path + [dir]))        # Ubico en el fringe el nodo adyacente                            
                    path.push(final_path + [dir] ) # Guardo el camino + la nueva dirección
    return final_path
    #util.raiseNotDefined()


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    from util import PriorityQueue #FIFO
    path = PriorityQueue() #Aca voy guardando el camino desde el comienzo hasta mi estado actual
    fringe = PriorityQueue() #Aca guardo los estados a expandir
    final_path = list() #Aca voy a guardar las direcciones finales para llegar
    visited_pos = list() #Aca guardo los nodos que ya visité para no entrar en un loop 
    initial_cost = 0
    final_cost = initial_cost
    #Comienzo desde el estado inicial
    next_pos = problem.getStartState()
    print('problem.getStartState() = ', problem.getStartState())
    fringe.push((next_pos, final_path), initial_cost)
    print('fringe.pop()INICIAL',fringe.pop())
    while not fringe.isEmpty() and not problem.isGoalState(next_pos): #Loop hasta que no llegue al estado deseado y siga teniendo elementos
        print('fringe.pop() = ',fringe.pop())
        next_pos, final_path, actual_cost = fringe.pop()           # Saco del fringe el proximo nodo que quiero visitar
        if next_pos not in visited_pos:       
            visited_pos.append(next_pos)     #Voy marcando los nodos visitados
            if problem.isGoalState(next_pos):
                return final_path
            else:
                next_pos = problem.getSuccessors(next_pos) #Voy recorriendo los sucesores
                for next_node, dir, cost in next_pos:
                    print(cost)
                    final_cost = final_cost + cost
                    fringe.push((next_node,final_path + [dir]), cost)        # Ubico en el fringe el nodo adyacente                            
                    path.push((final_path + [dir]), cost ) # Guardo el camino + la nueva dirección
    return final_path
    #util.raiseNotDefined()


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch