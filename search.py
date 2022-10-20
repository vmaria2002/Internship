#https://github.com/lyzfrank/AI-Pacman/blob/master/uninformed_search.py
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
    nodcrt= (problem.getStartState(), [])
    frontiera=util.Stack()
    teritoriu = []
    frontiera.push(nodcrt)

    while not frontiera.isEmpty():
        nodcrt= frontiera.pop()
        if problem.isGoalState(nodcrt[0]):
            return nodcrt[1]
        teritoriu.append(nodcrt[0])
        succesori=problem.expend(nodcrt[0])
        for succesor in succesori:
            (stare, mutare, cost)=succesor   
            if stare not in teritoriu and stare not in (nodcrt[0] for nod in frontiera.list):
                cale=nodcrt[1] + [mutare]
                frontiera.push((stare, cale))

    return []


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
   
    nodcrt = (problem.getStartState(), [])  #in aceasta frontiera este o pereche de ( starea, actiuni necesare pentru accesarea starii)
    frontiera= util.Queue()
    teritoriu = []

    #aici ce trebuie sa fie frontiera???

    frontiera.push(nodcrt)

    while not frontiera.isEmpty():
        nodcrt= frontiera.pop()
        if problem.isGoalState(nodcrt[0]):
            return nodcrt[1]
        teritoriu.append(nodcrt[0])
        succesori=problem.getSuccesors(nodcrt[0])
        for succesor in succesori:
            (stare, mutare, )=succesor   
            if stare not in teritoriu and stare not in (nodcrt[0] for nod in frontiera.list):
                cale=nodcrt[1] + [mutare]
                frontiera.push((stare, cale))

    return []

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    nodCrt = (problem.getStartState, []) # primul nod, care incepe de la starea initiala a problemei, stare initiala, lista de actiuni la inceput vida
    frontiera=util.PriorityQueue()  #frontiera trebuie sa fie coada cu prioritati
    teritoriu =[] #noduri expandate, la inceput nu avem niciun nod expandat
    
    #(nodCrt, 0): pentru fiecare nod se asociaza COSTUL
    #se pune initial 0, intrucat pt a te deplasa din stare initiala in ea insasi, costul ar fi 0
    frontiera.push(nodCrt, 0) #nodul initial se adauga la inceput in frontiera
    
    #cat timp frontiera nu este vida
    while not frontiera.isEmpty():
        nodCrt = frontiera.pop() #nod obtinut din frontiera, nod cu prioritate maxima
        #verificam daca starea asociata acestui nod este stare finala sau nu
         
        #accesez starea, e primul element: nodCrt[0], 
        if(problem.isGoalState(nodCrt[0])):
            #nodCrt[1] :reprezinta lista de actiuni care trebuie efectuate pentru a se ajunge in starea finala dorita
            return nodCrt[1]  #starea curenta == cu starea finala
        #nu ne aflam in starea finala, deci trebuie sa adaugam la acest teritoriu, multimea starilor
        #adaug din nodul curent doar starea asociata nodului
        teritoriu.append(nodCrt[0])
        #dupa ce adauf starea
        #vreau sa obtin succesorii acestei sati curente
        succesori=problem.getSuccesors(nodCrt[0]) #expandex cu starea care se gaseste in nodul curent
       
        #pt fiecare succeor gasit in lista de succesori
        #se returneaza lista succesorilor pentru fiecare din succesori, cunoscandu-se starea, mutarea si costul mutarii
        for succesor in succesori:
            (stare, mutare, cost)=succesor
            #daca starea nu se gaseste in teritoriu
            if stare not in teritoriu:
                #poate/nu poate sa fie in frontiera
                #dar noi avem deja coada de prioritati, si see verifica, facandu-se update daca este nevoie
                # a se vedea in util.py, functia: def update(self, item, priority)
                #explicatie:daca are o prioritate mai mica, face updat, pt prioritatea maxima, altfel, face direct adaugarea

                #se calculeaza noua cale
                # nodCrt[1]: este lista de actiuni pe care o voi concatena cu nutarea asociata  listei de actiuni/cale (este in lista numita "mutare")
                cale = nodCrt[1] + [mutare]
                #calculez valoarea asociata acestei noi stari
                q = problem.getCostOfActionSequence(cale)
                #partea de adaugare
                frontiera.update((stare, cale), q) #introduc un nou nod, format din noul succesor si calea sa, iar prioritatea este valoarea q

    return []  #esec 


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    
    frontiera=util.PriorityQueue()  
    teritoriu =[] 

    #doar aici se fac modificari, adaugandu-se partea de euristica
    h=heuristic(problem.getStartState(), problem) #par1: starea pentru care se calculeaza functia euristica= starea initiala
   
    g=0 #costul real din starea initiala, in starea finala, este=0
    f=g+h
    nodCrt = (problem.getStartState, [])  #nodul curent care se creaza in aceasta situatie
    frontiera.push(nodCrt, f) #se adauga nodul curent si costul sau (f)

    while not frontiera.isEmpty():
        nodCrt = frontiera.pop() #se extrage un nod
        if(problem.isGoalState(nodCrt[0])): #daca starea este stare finala
            return nodCrt[1]   #returnam lista de actiuni
        teritoriu.append(nodCrt[0]) #adaugam starea respectiva in teritoriu
        succesori=problem.getSuccesors(nodCrt[0])  #obtin succesorii nodului curent
        for succesor in succesori: #pt fiecare succesor, vom avea: 
            (stare, mutare, cost)=succesor #stare. mutare, cost aferent
        
            if stare not in teritoriu: #stare ce nu e in teritoriu
               #voi calcula 
                cale = nodCrt[1] + [mutare]
               
                q = problem.getCostOfActionSequence(cale)
                #cost pana in stare curenta
               
               
                #in euristica, se pune cale, nu problem.getStartState()-starea curenta
                h=heuristic(stare, problem) #par1: starea pentru care se calculeaza functia euristica= starea initiala
               
                #in frontiera se actualizeaza cu aceasta stare in calea specificata de g
                frontiera.update((stare, cale), q) 

    return []


#A* cautarea informata
#reprezinta toate actiunile/mutarile, de la starea initiala, la starea curenta
#la care se adauga h (): euristica=costul estimat de la starea curenta, la starea finala
#h: nu trebuie sa suraestimeze costul real
#Suma Menhetan: suma celor doua distante implementata in util.py


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch

#de pus obstacole in labirint:

#lista tuturor pozitiilor, care reprezinta puncte
#starea initiala: cand atinge un obiect, il scoate din starea curenta, din lista de pozitii cautate
# operatii posibile 
#stare finala, camd lista respectiva este vida