"""
Description:
    -   ...

By Oliver Heilmann
Modified from ...
"""

import numpy as np
from math import sqrt
from priorityqueue import PriorityQueue

class Node:
    def __init__(self, state=(0,0), actual_cost_func=None, parent=None):
        self.state = state
        self.parent = parent
        self.actual_cost_func = actual_cost_func
        if parent is None:
            self.path_cost = 0
        else:
            self.path_cost = parent.path_cost + self.actual_cost_func(self)

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


def valid_space(obstacles, space, maxslope):
    return 0 <= space[0] < len(obstacles) \
           and 0 <= space[1] < len(obstacles[0]) \
           and obstacles[space] < maxslope


def astar_search(maze, obstacles, maxslope, start=(0, 0), goal=None, gridsize=(1,1)):
    if goal is None:
        goal = (len(maze) - 1, len(maze[0]) - 1)

    # heuristic in 3D = h(x,y,z) + g(x,y,z) where h(..) is cost to goal and g(..) is 
    # accumulated cost up to this point (from start) - written as lambda expression
    heuristic = lambda node:node.path_cost + sqrt(  abs( (goal[0] - node.state[0]) * gridsize[0] ) +    \
                                                    abs( (goal[1] - node.state[1]) * gridsize[1] ) +    \
                                                    abs( maze[goal] - maze[node.state] ))
    h_s = lambda node:sqrt( abs( (node.state[0] - start[0]) * gridsize[0] ) +   \
                            abs( (node.state[1] - start[1]) * gridsize[1] ) +   \
                            abs( maze[node.state] - maze[start] ))

    frontier = Frontier( heuristic, Node(state = start, actual_cost_func=h_s) )
    explored = set()

    current_node = frontier.pop()
    number_explored = 0
    
    while not current_node.state == goal:
        current_state = current_node.state

        number_explored += 1
        explored.add(current_state)
        
        # the four neigbouring locations (remember, can only traverse on 2D plane i.e. AGV cannot move 
        # up or down so we exclude these even though our cost heuristic considers height)
        right = (current_state[0], current_state[1] + 1)
        left = (current_state[0], current_state[1] - 1)
        down = (current_state[0] + 1, current_state[1])
        downright = (current_state[0] - 1, current_state[1] + 1)
        downleft = (current_state[0] - 1, current_state[1] - 1)
        up = (current_state[0] - 1, current_state[1])
        upright = (current_state[0] + 1, current_state[1] + 1)
        upleft = (current_state[0] + 1, current_state[1] - 1)
        
        for space in [right, left, down, downright, downleft, up, upright, upleft]:
            # check if space is valid with the slope map (remember, our heuristic 
            # is actually calculating distances in x,y,z however)
            if valid_space(obstacles, space, maxslope) and space not in explored:
                node = Node(state=space, parent=current_node, actual_cost_func=h_s)
                frontier.push(node)

        if frontier.length() == 0:
            return None, number_explored

        current_node = frontier.pop()
    
    return current_node, number_explored


def astarRoute3D( intensitymap : np.array, slopemap : np.array, maxslope=100, gridsize=(1,1) ):
    """Perform A* Search Algorithm on elevation map (using slopemap to determine obstacles) and return route as list."""
    final_node, number_explored = astar_search( maze = intensitymap,
                                                obstacles = slopemap,
                                                maxslope = maxslope,
                                                gridsize = gridsize)
    
    print("A STAR SEARCH:")
    solution = []
    if final_node is None:
        print("No path exists!\n")
    else:
        node = final_node
        steps = 0
        while node.parent is not None:
            if steps == 0:
                 print(f"    Total cost of path: {round(node.path_cost,2)}")

            # reformat solution to return as [ (x1,y1), (x2,y2) ... ]
            state = node.state
            solution.append( [state[1]*gridsize[0], state[0]*gridsize[1]] )
            steps += 1
            node = node.parent

        # reformat solution to return as [ (x1,y1), (x2,y2) ... ]
        state = node.state
        solution.append( [state[1]*gridsize[0], state[0]*gridsize[1]] )
        
        print(f"    Total steps on path: {steps}")
        print(f"    Total states explored: {number_explored}")
    return solution[::-1]   # reverse to start in correct order