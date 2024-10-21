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

def depthFirstSearch(problem, posititon=None):


    help = (problem.getStartState(), 1)
    nodes = [problem.getStartState()]
    nodeStack = []

    iteratedepth(problem, help, nodes, nodeStack)


    return nodeStack

def iteratedepth(problem , node : tuple , nodesDone : list , nodeStack ) :


    if not problem.isGoalState(node[0]) :


        ##if there are moves available
        if (problem.getSuccessors(node[0])) :

            ##it just takes a path until the end is found, so if it takes the wrong child
            ##it takes a long path
            new_lst = problem.getSuccessors(node[0])[::-1]
            ##check children
            for i in new_lst:

                if i[0] not in nodesDone :
                    nodesDone.append(i[0])
                    nodeStack.append(i[1])


                    if iteratedepth(problem , i , nodesDone, nodeStack) :
                         return True

                    else:
                        nodeStack.pop()

            return False
        # we are at the end
        else :
            return False
    ##we got the finish
    else :
        return True

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    pathQueue = util.Queue()
    listvisited = [problem.getStartState()]

    m = (problem.getStartState() ,() , 0)  # gets path from queue
    pathQueue.push(m)

    while not pathQueue.isEmpty()  :   # Creating loop to visit each node

        m = pathQueue.pop()
        for i in problem.getSuccessors(m[0]):

            if i[0] not in listvisited:


                act = m[1] + (i[1],)
                t=(i[0] , act , m[2] + i[2] )

                isgoal = problem.isGoalState(i[0])
                if (isgoal) :
                    return t[1]

                pathQueue.push(t )
                listvisited.append(i[0])



    return  ''


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    pathQueue = util.PriorityQueue()
    listvisited = [problem.getStartState()]

    m = ()  # gets path from queue
    k = (problem.getStartState(), "beginstate" , 0 )  # gets coordinates for the last node in the path
    goal = problem.isGoalState(k)


    while not problem.isGoalState(k[0]):  # Creating loop to visit each node

        for i in problem.getSuccessors(k[0]):
            if i[0] not in listvisited:
                t = m + (i,)
                pathQueue.update(t, k[2]+i[2])
                listvisited.append(i[0])

        m = pathQueue.pop()
        k = m[len(m) - 1]


    directions = []
    for i in m:
        directions.append(i[1])
    return directions



def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    ##endstate is (0,0)??

    flyingDistance = (((state[0])**2)+((state[0])**2))**(1/2)

    return flyingDistance

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    pathQueue = util.PriorityQueue()
    listvisited = [problem.getStartState()]

    m = (problem.getStartState(), (), 0)  # gets path from queue
    pathQueue.update(m , 0)

    while not pathQueue.isEmpty():  # Creating loop to visit each node

        m = pathQueue.pop()
        for i in problem.getSuccessors(m[0]):

            if i[0] not in listvisited:

                act = m[1] + (i[1],)
                t = (i[0], act, m[2] + i[2])

                isgoal = problem.isGoalState(i[0])
                if (isgoal):
                    return t[1]

                aStarCost = heuristic(i[0] , problem)
                pathQueue.update(t , aStarCost)
                listvisited.append(i[0])

    return ''

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
