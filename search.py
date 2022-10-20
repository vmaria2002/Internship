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
import random

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
        """python pacman.py -l bigMaze -z .5 -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic

          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def expand(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (child,
        action, stepCost), where 'child' is a child to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that child.
        """
        util.raiseNotDefined()

    def getActions(self, state):
        """
          state: Search state

        For a given state, this should return a list of possible actions.
        """
        util.raiseNotDefined()

    def getActionCost(self, state, action, next_state):
        """
          state: Search state
          action: action taken at state.
          next_state: next Search state after taking action.

        For a given state, this should return the cost of the (s, a, s') transition.
        """
        util.raiseNotDefined()

    def getNextState(self, state, action):
        """
          state: Search state
          action: action taken at state

        For a given state, this should return the next state after taking action from state.
        """
        util.raiseNotDefined()

    def getCostOfActionSequence(self, actions):
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

"""2.10"""
def randomSearchAgent(problem):
    """As a baseline, we will create an agent which searches
a solution randomly: it just picks one legal action at each step of search. Write a new search
function in search.py similar to tinyMazeSearch function. The function returns a list of
actions from the initial state to goal state, each action being randomly selected from the set of
legal actions."""

    solutie=[]
    startPoz = problem.getStartState()
    while( not (problem.isGoalState(startPoz)) ):
        succesorii = problem.expand(startPoz)
        nr = len(succesorii)
        randSucc = int(random.random()*nr)
        urm = succesorii[randSucc]
        startPoz=urm[0]
        solutie.append(urm[1])

    print("Solutia este: ", solutie)
    return solutie

def randomSearch ( problem ) :
        current = problem . getStartState ()
        solution =[]
        while ( not ( problem . isGoalState ( current ) ) ) :
            succ = problem . expand ( current )
            no_of_successors = len ( succ )
            random_succ_index = int ( random . random () * no_of_successors )
            next = succ [ random_succ_index ]
            current = next [0]
            solution . append ( next [1])
        print (" The solution is " , solution)
        return solution    
def depthFirstSearch(problem):
   

    start = problem.getStartState()
    "nodul de start de unde pleaca dfs"
    curent = start
    "nodul in care ma aflu"
    noduriExplorate = []
    "noduri pe care deja le am vizitat"
    stiva = util.Stack()
    stiva.push((start,[]))
    "pun in stiva nodul de start"
    "cat timp stiva nu e goala si nu am ajuns la punctul final (mancare)"
    while not stiva.isEmpty() and not problem.isGoalState(curent):
        nod, directii= stiva.pop()
        noduriExplorate.append(nod)
        "marchez ca si vizitat nodul pe care urmeaza sa il explorez"
        succesori = problem.expand(nod)
        "ma uit la succesoruii nodului curent"
        for i in succesori:
            coordonataCurenta = i[0]
            "coordonata nodului (x,y)"
            if not coordonataCurenta in noduriExplorate:
                "daca coordonata nu este in nodurile explorate"
                curent = i[0]
                directie = i[1]
                "directia pe unde trebuie sa o iau"
                stiva.push((curent,directii+[directie]))
                "pun i stiva coordonata curenta si directia "
    return directii+[directie]
    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    start = problem.getStartState()
    "nodul de start de unde pleaca bfs"
    noduriExplorate = []
    "noduri pe care deja le am vizitat"
    noduriExplorate.append(start)
    coada = util.Queue()
    coada.push((start, []))
    "pun in coada nodul de start"
    "cat timp coada nu e goala "

    while not coada.isEmpty():
        nod, directii = coada.pop()
        "daca am ajuns la final returnam directiile"
        if problem.isGoalState(nod):
            return directii
        succesori = problem.expand(nod)
        "ma uit la succesoruii nodului curent"
        for i in succesori:
            coordonataCurenta = i[0]
            "coordonata nodului (x,y)"
            if not coordonataCurenta in noduriExplorate:
                "daca coordonata nu este in nodurile explorate"
                directie = i[1]
                "directia pe unde trebuie sa o iau"
                noduriExplorate.append(coordonataCurenta)
                coada.push((coordonataCurenta, directii + [directie]))
                "pun i stiva coordonata curenta si directia "
    return []
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"   

    startState = problem.getStartState()
    coadaDePrioritati = util.PriorityQueue()
    noduriExplorate = []
    coadaDePrioritati.push((startState, [], 0), 0)

    while not coadaDePrioritati.isEmpty():

        nod, directii, cost = coadaDePrioritati.pop()
        currentNode = (nod, cost)
        noduriExplorate.append(currentNode)

        if problem.isGoalState(nod):
            return directii
        
        succesori = problem.expand(nod)

        for i in succesori:
            noulCost = problem.getCostOfActionSequence(directii + [i[1]])
            noulNod = (i[0], directii + [i[1]], noulCost)

            vizitat = False

            for j in noduriExplorate:
                jState, jCost = j

                if (i[0] == jState) and (noulCost >= jCost):
                    vizitat = True

            if not vizitat:
                coadaDePrioritati.push(noulNod, noulCost + heuristic(i[0], problem))
                noduriExplorate.append((i[0], noulCost))


    return []


    util.raiseNotDefined()



# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
rs=randomSearch