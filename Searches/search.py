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
		"""
	# print("Start:", problem.getStartState())
	# print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
	# print("Start's successors:", problem.getSuccessors(problem.getStartState()))

	"Start of Your Code"
	visited = set()
	actions = []
	frontier = util.Stack()
	startState = problem.getStartState()
	startNode = (startState, [])
	frontier.push(startNode)
	while not frontier.isEmpty():
		currentNode = frontier.pop()
		currentState = currentNode[0]
		actions = currentNode[1]
		if problem.isGoalState(currentState):
			return actions
		if currentState not in visited:
			visited.add(currentState)
			for successor in problem.getSuccessors(currentState):
				successorNode = (successor[0], actions + [successor[1]])
				frontier.push(successorNode)
	return actions
	"End of Your Code"

# ________________________________________________________________

class _RecursiveDepthFirstSearch(object):
	'''
		=> Output of 'recursive' dfs should match that of 'iterative' dfs you implemented
		above. 
		Key Point: Remember in tutorial you were asked to expand the left-most child 
		first for dfs and bfs for consistency. If you expanded the right-most
		first, dfs/bfs would be correct in principle but may not return the same
		path. 

		=> Useful Hint: self.problem.getSuccessors(node) will return children of 
		a node in a certain "sequence", say (A->B->C), If your 'recursive' dfs traversal 
		is different from 'iterative' traversal, try reversing the sequence.  

	'''
	def __init__(self, problem):
		" Do not change this. " 
		# You'll save the actions that recursive dfs found in self.actions. 
		self.actions = [] 
		# Use self.explored to keep track of explored nodes.  
		self.explored = set()
		self.problem = problem

	def RecursiveDepthFirstSearchHelper(self, node):
		'''
		args: start node 
		outputs: bool => True if path found else Fasle.
		'''
		"Start of Your Code"
		if self.problem.isGoalState(node):
			self.actions.reverse()
			return True
		if node == self.problem.getStartState() and node not in self.explored:
			self.explored.add(node)
		for successor in reversed(self.problem.getSuccessors(node)):
			if successor[0] not in self.explored:
				self.explored.add(successor[0])
				self.actions.append(successor[1])
				check = self.RecursiveDepthFirstSearchHelper(successor[0])
				if not check:
					self.actions.pop()
				else:
					return True
		return False
		"End of Your Code"


def RecursiveDepthFirstSearch(problem):
	" You need not change this function. "
	# All your code should be in member function 'RecursiveDepthFirstSearchHelper' of 
	# class '_RecursiveDepthFirstSearch'."

	node = problem.getStartState() 
	rdfs = _RecursiveDepthFirstSearch(problem)
	path_found = rdfs.RecursiveDepthFirstSearchHelper(node)
	return list(reversed(rdfs.actions)) # Actions your recursive calls return are in opposite order.
# ________________________________________________________________


def depthLimitedSearch(problem, limit = 129):

	"""
	Search the deepest nodes in the search tree first as long as the
	nodes are not not deeper than 'limit'.

	For medium maze, pacman should find food for limit less than 130. 
	If your solution needs 'limit' more than 130, it's bogus.
	Specifically, for:
	'python pacman.py -l mediumMaze -p SearchAgent -a fn=dls', and limit=130
	pacman should work normally.  

	Your search algorithm needs to return a list of actions that reaches the
	goal. Make sure to implement a graph search algorithm.
	Autograder cannot test this function.  

	Hints: You may need to store additional information in your frontier(queue).

		"""

	"Start of Your Code"
	visited = {}
	actions = []
	frontier = util.Stack()
	startState = problem.getStartState()
	startNode = (startState, [], 0)
	frontier.push(startNode)
	while not frontier.isEmpty():
		currentNode = frontier.pop()
		currentState = currentNode[0]
		actions = currentNode[1]
		depth = currentNode[2]
		if depth > limit:
			continue
		if problem.isGoalState(currentState):
			return actions
		if currentState not in visited or visited[currentState] > depth:
			visited.update({currentState:depth})
			for successor in problem.getSuccessors(currentState):
				successorNode = (successor[0], actions + [successor[1]], depth + 1)
				frontier.push(successorNode)
	return actions
	"End of Your Code"

# ________________________________________________________________

class _RecursiveDepthLimitedSearch(object):
	'''
		=> Output of 'recursive' dfs should match that of 'iterative' dfs you implemented
		above. 
		Key Point: Remember in tutorial you were asked to expand the left-most child 
		first for dfs and bfs for consistency. If you expanded the right-most
		first, dfs/bfs would be correct in principle but may not return the same
		path. 

		=> Useful Hint: self.problem.getSuccessors(node) will return children of 
		a node in a certain "sequence", say (A->B->C), If your 'recursive' dfs traversal 
		is different from 'iterative' traversal, try reversing the sequence.  

	'''
	def __init__(self, problem):
		" Do not change this. " 
		# You'll save the actions that recursive dfs found in self.actions. 
		self.actions = [] 
		# Use self.explored to keep track of explored nodes.  
		self.explored = set()
		self.problem = problem
		self.current_depth = 0
		self.depth_limit = 129 # For medium maze, You should find solution for depth_limit not more than 204.

	def RecursiveDepthLimitedSearchHelper(self, node):
		'''
		args: start node 
		outputs: bool => True if path found else Fasle.
		'''
		"Start of Your Code"
		if self.current_depth > self.depth_limit:
			return False
		temp = False
		for item in self.explored:
			if item[0] == node:
				temp = True
				break
		if not temp:
			self.current_depth = self.current_depth + 1
			self.explored.add((node, self.current_depth))
		if self.problem.isGoalState(node) and self.current_depth <= self.depth_limit:
			return True
		for successor in reversed(self.problem.getSuccessors(node)):
			check = True
			delete = None
			depth = 0
			for item in self.explored:
				if item[0] == successor[0]:
					depth = item[1]
					break
			for item in self.explored:
				if item[0] == successor[0]:
					if item[1] > self.current_depth + 1:
						delete = item
						check = False
						break
			if (successor[0], depth) not in self.explored:
				if self.RecursiveDepthLimitedSearchHelper(successor[0]):
					self.actions.append(successor[1])
					return True
			if not check:
				self.explored.remove(delete)
				if self.RecursiveDepthLimitedSearchHelper(successor[0]):
					self.actions.append(successor[1])
					return True	
		self.current_depth = self.current_depth - 1
		return False
		"End of Your Code"


def RecursiveDepthLimitedSearch(problem):
	"You need not change this function. All your code in member function RecursiveDepthLimitedSearchHelper"
	node = problem.getStartState() 
	rdfs = _RecursiveDepthLimitedSearch(problem)
	path_found = rdfs.RecursiveDepthLimitedSearchHelper(node)
	return list(reversed(rdfs.actions)) # Actions your recursive calls return are in opposite order.
# ________________________________________________________________


def breadthFirstSearch(problem):
	"""Search the shallowest nodes in the search tree first."""

	"Start of Your Code"
	visited = set()
	actions = []
	frontier = util.Queue()
	startState = problem.getStartState()
	startNode = (startState, [])
	frontier.push(startNode)
	visited.add(startState)
	while not frontier.isEmpty():
		currentNode = frontier.pop()
		currentState = currentNode[0]
		actions = currentNode[1]
		if problem.isGoalState(currentState):
			return actions
		for successor in problem.getSuccessors(currentState):
			childNode = successor
			if childNode[0] not in visited:
				visited.add(childNode[0])
				frontier.push((childNode[0], actions + [childNode[1]]))
	return actions
	"End of Your Code"


def uniformCostSearch(problem):
	"""Search the node of least total cost first.
	   You may need to pay close attention to util.py.
	   Useful Reminder: Note that problem.getSuccessors(node) returns "step_cost". 

	   Key Point: If a node is already present in the queue with higher path cost 
	   (or higher priority), you'll update its cost (or priority) 
	   (Similar to pseudocode in figure 3.14 of your textbook.). 
	   Be careful, autograder cannot catch this bug.
	"""

	"Start of Your Code"
	visited = set()
	actions = []
	newAction = []
	frontier = util.PriorityQueue()
	startState = problem.getStartState()
	startNode = (startState, [])
	frontier.push(startNode, 0)
	while not frontier.isEmpty():
		temp = frontier.pop()
		currentNode = temp[0]
		currentState = currentNode[0]
		actions = currentNode[1]
		cost = temp[1]		
		if problem.isGoalState(currentState):
			return actions
		if currentState not in visited:
			visited.add(currentState)
			for successor in problem.getSuccessors(currentState):
				if successor[0] not in visited:
					newAction = actions.copy()
					newAction.append(successor[1])
					newcost = successor[2]+cost 
					frontier.push((successor[0], newAction), newcost)
				elif frontier.item_present_with_higher_priority(successor, cost) is not None:
					index, count = frontier.item_present_with_higher_priority(successor, cost)
					frontier.Update_priority(successor, cost, index, count)
	return actions
	"End of Your Code"

def nullHeuristic(state, problem=None):
	"""
	A heuristic function estimates the cost from the current state to the nearest
	goal in the provided SearchProblem.  This heuristic is trivial.
	"""
	return 0

def aStarSearch(problem, heuristic=nullHeuristic):
	'''
	Pay clos attention to util.py- specifically, args you pass to member functions. 

	Key Point: If a node is already present in the queue with higher path cost 
	(or higher priority), you'll update its cost (or priority) 
	(Similar to pseudocode in figure 3.14 of your textbook.). 
	Be careful, autograder cannot catch this bug.

	'''
	"Start of Your Code"
	visited = set()
	actions = []
	newAction = []
	frontier = util.PriorityQueue()
	startState = problem.getStartState()
	startNode = (startState, [])
	frontier.push(startNode, 0)
	while not frontier.isEmpty():
		temp = frontier.pop()
		currentNode = temp[0]
		currentState = currentNode[0]
		actions = currentNode[1]
		cost = temp[1] - heuristic(currentState, problem)		
		if problem.isGoalState(currentState):
			return actions
		if currentState not in visited:
			visited.add(currentState)
			for successor in problem.getSuccessors(currentState):
				newAction = actions.copy()
				newAction.append(successor[1])
				newcost = successor[2] + cost 
				if successor[0] not in visited:
					frontier.push((successor[0], newAction), newcost + heuristic(successor[0], problem))
	return []
	"End of Your Code"


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
rdfs = RecursiveDepthFirstSearch
dls = depthLimitedSearch
rdls = RecursiveDepthLimitedSearch
astar = aStarSearch
ucs = uniformCostSearch
