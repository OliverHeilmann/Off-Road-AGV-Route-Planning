"""
Description:
    -   ...

By Oliver Heilmann
Modified from ...
"""

import numpy as np
from priorityqueue import PriorityQueue

class Node:
    def __init__(self, state, parent=None):
        self.state = state
        self.parent = parent

    def __eq__(self, other):
        return self.state == other.state

    def __lt__(self, other):
        return self.state < other.state

    def __hash__(self):
        return hash(self.state)

    def __str__(self):
        return f"Node space {self.state}"


class Frontier:
    # Note the heuristic function is passed in as a parameter
    # Python borrows some nice features from functional programming
    def __init__(self, heuristic, start_node=None):
        self.heuristic = heuristic

        self.queue = PriorityQueue()
        self.states = set()

        if start_node is not None:
            self.push(start_node)
            
    def push(self, node):
        cost = self.heuristic(node)
        # get_priority returns math.inf if the task is not in the queue
        if cost < self.queue.get_priority(node):
            self.queue.push(node, priority=cost)
            self.states.add(node.state)
        
    def pop(self):
        node = self.queue.pop()
        self.states.remove(node.state)
        return node
        
    def contains(self, state):
        return state in self.states
    
    def length(self):
        return self.queue.length()


def valid_space(maze, space, maxslope):
    return 0 <= space[0] < len(maze) \
           and 0 <= space[1] < len(maze[0]) \
           and maze[space[0]][space[1]] < maxslope


def greedy_search(maze, maxslope, start=(0, 0), goal=None):
    if goal is None:
        goal = (len(maze) - 1, len(maze[0]) - 1)

    # here's our Manhattan distance heurstic, as a lambda expression
    heuristic = lambda node: abs(goal[0] - node.state[0]) + abs(goal[1] - node.state[1])
    frontier = Frontier(heuristic, Node(start))
    explored = set()

    current_node = frontier.pop()
    number_explored = 0
    
    while not current_node.state == goal:
        current_state = current_node.state

        number_explored += 1
        explored.add(current_state)
        
        # the four neigbouring locations
        right = (current_state[0], current_state[1] + 1)
        left = (current_state[0], current_state[1] - 1)
        down = (current_state[0] + 1, current_state[1])
        downright = (current_state[0] - 1, current_state[1] + 1)
        downleft = (current_state[0] - 1, current_state[1] - 1)
        up = (current_state[0] - 1, current_state[1])
        upright = (current_state[0] + 1, current_state[1] + 1)
        upleft = (current_state[0] + 1, current_state[1] - 1)
        
        for space in [right, left, down, downright, downleft, up, upright, upleft]:
            if valid_space(maze, space, maxslope) and space not in explored:
                node = Node(space, parent=current_node)
                frontier.push(node)

        if frontier.length() == 0:
            return None, number_explored

        current_node = frontier.pop()
    
    return current_node, number_explored


def greedyRoute( slopemap : np.array, maxslope=100, gridsize=(1,1) ):
    """Perform Greedy Search Algorithm on slopemap and return route as list."""
    final_node, number_explored = greedy_search(slopemap, maxslope)

    solution = []
    if final_node is None:
        print("No path exists!\n")
    else:
        node = final_node
        steps = 0
        while node.parent is not None:
            # reformat solution to return as [ (x1,y1), (x2,y2) ... ]
            state = node.state
            solution.append( [state[1]*gridsize[0], state[0]*gridsize[1]] )
            steps += 1
            node = node.parent

        # reformat solution to return as [ (x1,y1), (x2,y2) ... ]
        state = node.state
        solution.append( [state[1]*gridsize[0], state[0]*gridsize[1]] )
        
        print(f"Total steps on path: {steps}")
        print(f"Total states explored: {number_explored}")
    return solution[::-1]   # reverse to start in correct order