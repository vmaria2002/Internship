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

#in aceasta clasa, avem prezentate toate metodele pentru care se soreste cautarea(dfs, bfs, A*)

class SearchProblem:
    #aceste metode sunt implementate in searchAgent.py
    #starea curenta
    def getStartState(self):
        util.raiseNotDefined()
    #starea finala
    def isGoalState(self, state):
        util.raiseNotDefined()
    #lista de miscari posibile
    def getSuccessors(self, state):
        util.raiseNotDefined()
    #obtine mutarea facuta
    def getActions(self, state):
        util.raiseNotDefined()

    def getActionCost(self, state, action, next_state):
        util.raiseNotDefined()

    def getNextState(self, state, action):
        util.raiseNotDefined()

    def getCostOfActionSequence(self, actions):
        util.raiseNotDefined()


#se da o secventa de mutari, aceasta va fi solutia unui labirint, pentru care s-a generat lista de miscari
#obs: pentru orice alt labirint, nu se va gasi solutie-->pacman va ramane blocat intr-un colt
def tinyMazeSearch(problem):
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


#problema cautarii Random:
#cod sursa: laborator

def randomSearch(problem):
    solutie = []     #lista de solutii, initial goala
    startPoz = problem.getStartState()  #se porneste de pe pozitia in care se afla pacman
    while (not (problem.isGoalState(startPoz))): #cat timp nu ne aflam in pozitia finala, exploram spatiul starilor
        succesorii = problem.getSuccessors(startPoz) #obtin lista succesorilor pentru pozitia in care se afla

        #logica pentru a selecta un anumit succesor
        nr = len(succesorii)                #nr=nr maxim pe care il poate atinge, pt a nu iesi din domeiul specifiact
        randSucc = int(random.random() * nr) #numar al pozitiei: random
        urm = succesorii[randSucc]           #nodul explorat de pe pozitia random
        startPoz = urm[0] #dupa ce ne aflam pe pozitia de random, vom actualiza start: aceasta va fi prima valoare din lista de succesori gasiti
        solutie.append(urm[1]) #se ia primul din succesorii random, adaugandu-se la

    print("Solutia este: ", solutie)
    return solutie




def depthFirstSearch(problem):
    start = problem.getStartState()  #nodul curent informatii: starea nodului curent, lista de mutari
    curent = start #nodul in care ma aflu

    # Incep sa construiesc arborele de cautare
    # frontiera=stari pe care inca nu le-am explorat,dar pe care le putem explora
    # nodurile expandate, pornind din punctul curent
    # frontiera e o margine. scot de aici un nod, il expandez mai departe
    # frontiera= delimitatie intre ce am explorat si ce nu am explorat

    teritoriu = [] #teritoriu
    frontiera = util.Stack() #se foloseste o stiva pentru a explora tot teritoriul

    #cand incep sa exlorez, sunt in start, de aceea, nu am nicio directie pe care sa o memorez
    frontiera.push((start, [])) #in frotiera pun nodul cu care incep + lista de directii

    # la fiecare pas, trebuie sa ma hotarasc, ce nod din frontiera va fi explorat in continuare :)
    # frontiera goala: nu mai are unde cauta, am ramas fara alternative
    "si"
    # cazul in care se termina jocul
    # daca s-a ajuns in starea finala
    # se returneaza lista de deplasari/mutari efectuate pana sa ajunga in pozitia respectiva
    while not frontiera.isEmpty() and not problem.isGoalState(curent):
        #daca mai am ce explora, pot verific succesorii:

        #pas1: se extrag din frontiera ultimul nod introdus
        nod, directii = frontiera.pop()
        #se adauga acest nou in lista celor vizitate (teritoriu)
        # cazul in care putem continua sa expandam nodurile, pentru a  gasi nodurile pe care le va parcurge
        teritoriu.append(nod)
        # daca am deja o cale catre nod, nu il mai pun, caut alte cai ale celorlalte noduri
        succesori = problem.getSuccessors(nod) #expandez succesorii nodului curent

        for i in succesori:
            # parcurg succesorii
            # evita parcurgerea de cicluri
            coordonataCurenta = i[0] #iau primul nod gasit in lista de succesori, nodul care se exploreaza, nu voi lua doar nodul, nu si lista de directii
             # nu vreau 2 modalitati de a ajunge la nod
            # evit cazul in care pornesc cu cautarea dintr-un nod al frontierei, si il intalnesc iar

            # daca nu e nodul in teritoriu si nici nu e in frontiera,
            if not coordonataCurenta in teritoriu:
                # adaug la cale, mutarea nodului.
                curent = i[0]
                directie = i[1]
                # adaug in frontiera nodul, updatandu-se calea
                frontiera.push((curent, directii + [directie]))
                "pun i frontiera coordonata curenta si directia "
    #se returneaza lista de de directii pe care le va face acman pentru a ajunge la destinatie
    return directii + [directie]
    util.raiseNotDefined()


def breadthFirstSearch(problem):
    start = problem.getStartState()
    teritoriu = [] #noduri pe care deja le am vizitat"
    teritoriu.append(start) #cand ajung la un nod, il marchez ca vizitat
    frontiera = util.Queue() #folosesc o coada, pt a vizita primele noduri adaugate in frontiera
    frontiera.push((start, [])) #initial, in frontiear vom avea nodul cu care se incepe, dar nu a facut nicio mutare-> lista[]

    while not frontiera.isEmpty():
        nod, directii = frontiera.pop()  #iau cate un nod din frontiera

        if problem.isGoalState(nod): #am ajuns la final
            return directii #reutnam lista in care
        succesori = problem.getSuccessors(nod) #se expandeaza succesorii nodului
        for i in succesori:
            coordonataCurenta = i[0] #se ia nodul curent

            #se evita ciclurile:
            if not coordonataCurenta in teritoriu: #daca coordonata nu este in nodurile explorate
                directie = i[1] #se salveaza directia adociata expandat
                teritoriu.append(coordonataCurenta) #se adauga in teritoriu
                #updatare directie
                #pun in stiva coordonata curenta si directia
                frontiera.push((coordonataCurenta, directii + [directie]))
    return []
    util.raiseNotDefined()


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    nodCrt = (problem.getStartState,
              [])  # primul nod, care incepe de la starea initiala a problemei, stare initiala, lista de actiuni la inceput vida
    frontiera = util.PriorityQueue()  # frontiera trebuie sa fie coada cu prioritati
    teritoriu = []  # noduri expandate, la inceput nu avem niciun nod expandat

    # (nodCrt, 0): pentru fiecare nod se asociaza COSTUL
    # se pune initial 0, intrucat pt a te deplasa din stare initiala in ea insasi, costul ar fi 0
    frontiera.push(nodCrt, 0)  # nodul initial se adauga la inceput in frontiera

    # cat timp frontiera nu este vida
    while not frontiera.isEmpty():
        nodCrt = frontiera.pop()  # nod obtinut din frontiera, nod cu prioritate maxima
        # verificam daca starea asociata acestui nod este stare finala sau nu

        # accesez starea, e primul element: nodCrt[0],
        if (problem.isGoalState(nodCrt[0])):
            # nodCrt[1] :reprezinta lista de actiuni care trebuie efectuate pentru a se ajunge in starea finala dorita
            return nodCrt[1]  # starea curenta == cu starea finala
        # nu ne aflam in starea finala, deci trebuie sa adaugam la acest teritoriu, multimea starilor
        # adaug din nodul curent doar starea asociata nodului
        teritoriu.append(nodCrt[0])
        # dupa ce adauf starea
        # vreau sa obtin succesorii acestei sati curente
        succesori = problem.getSuccesors(nodCrt[0])  # expandex cu starea care se gaseste in nodul curent

        # pt fiecare succeor gasit in lista de succesori
        # se returneaza lista succesorilor pentru fiecare din succesori, cunoscandu-se starea, mutarea si costul mutarii
        for succesor in succesori:
            (stare, mutare, cost) = succesor
            # daca starea nu se gaseste in teritoriu
            if stare not in teritoriu:
                # poate/nu poate sa fie in frontiera
                # dar noi avem deja coada de prioritati, si see verifica, facandu-se update daca este nevoie
                # a se vedea in util.py, functia: def update(self, item, priority)
                # explicatie:daca are o prioritate mai mica, face updat, pt prioritatea maxima, altfel, face direct adaugarea

                # se calculeaza noua cale
                # nodCrt[1]: este lista de actiuni pe care o voi concatena cu nutarea asociata  listei de actiuni/cale (este in lista numita "mutare")
                cale = nodCrt[1] + [mutare]
                # calculez valoarea asociata acestei noi stari
                q = problem.getCostOfActionSequence(cale)
                # partea de adaugare
                frontiera.update((stare, cale),
                                 q)  # introduc un nou nod, format din noul succesor si calea sa, iar prioritatea este valoarea q

    return []  # esec

#A star, este bazata pe euristici, pentru a avea a indeplini scopul intr-un numar minim de pasi
#Se foloseste urmatoarea euristica: f=g+h, unde: f=costul de la start->nodul curent || g=costul de la nodul curent-> goal

def aStarSearch(problem, heuristic=nullHeuristic):
    startState = problem.getStartState() #nodul de inceput
    frontiera = util.PriorityQueue() #se va folosi o coada de prioritate
    #se gaseste imlementata in util.py
    #are la baza un heap, care se actualizeaza la fiecare pas(heapify), in varf fiind valoarea de cost minim
    teritoriu = [] #nodurile vizitate, sunt marcate
                   #in teritoriu pastrez doar starea si lista, fiindca asa am construit celelalte functii

    h = heuristic(problem.getStartState(), problem)  # par1: starea pentru care se calculeaza functia euristica= starea initiala
    g = 0  # costul real din starea initiala, in starea finala, este=0
    f = g + h
    #frontiera va avea 3 campuri: pentru fiecare nod, avem: stare, lista de mutari si cost
    frontiera.push((startState, [], 0), 0) #primul nod(start, nu are niciun cost atribuit si nici mutari anterioare

    while not frontiera.isEmpty(): #explorez nodurile avute in frontiera
        nod, directii, cost = frontiera.pop()  #e o tupla, si vreau a accesez fiecare camp
        currentNode = (nod, cost)
        teritoriu.append(currentNode) #marchez nodul ca vizitat

        #am ajuns in starea finala, nu mai avem ce vizita
        if problem.isGoalState(nod):
            return directii
        #obtin lista de sucesori
        succesori = problem.getSuccessors(nod)

        #pentru fiecare sccesor, calculez g: cu
        for i in succesori:

            q = problem.getCostOfActionSequence(directii + [i[1]])
            nouaCale=directii + [i[1]]
            noulNod = (i[0], nouaCale, q)

            vizitat = False

            #se verifica daca este vizitat si costul din momentul acesta seste mai mare decat ce s-a gasuit pana acum
            #daca nu e mai bine, spunem ca e vizitat, si nu updatam nimic
            for j in teritoriu:
                jState, jCost = j

                if (i[0] == jState) and (q >= jCost):
                    vizitat = True

            #daca nu e vizitat,
            #schimbam euristica
            #adaug nodul in frontiera, modificand euristica cu nol nod si costul
            #marchez vizitat

            if not vizitat:
                h = heuristic(i[0], problem)
                frontiera.push(noulNod, q + h)
                teritoriu.append((i[0], q))

    return []

    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
rs = randomSearch
ucs = uniformCostSearch
