"""
Description:
    -   ...

By Oliver Heilmann
Modified from ...
"""
import time
import numpy as np
from math import sqrt, pow
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

    # f(n) = g(n) + h(n) where h(n) is the straight line distance to the goal from current point
    # and g(n) is the total cost already accumulated called path_cost (from start) - written as 
    # lambda expression
    f_n = lambda node : node.path_cost + sqrt(  pow( (goal[0] - node.state[0]) * gridsize[0], 2 ) +    \
                                                pow( (goal[1] - node.state[1]) * gridsize[1], 2 ) +    \
                                                pow( maze[goal] - maze[node.state], 2 ))
    
    g_step = lambda node : sqrt(pow( (node.state[0] - node.parent.state[0]) * gridsize[0], 2 ) +   \
                                pow( (node.state[1] - node.parent.state[1]) * gridsize[1], 2 ) +   \
                                pow( maze[node.state] - maze[node.parent.state], 2 ))

    frontier = Frontier( f_n, Node(state = start, actual_cost_func=g_step) )
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
                node = Node(state=space, parent=current_node, actual_cost_func=g_step)
                frontier.push(node)

        if frontier.length() == 0:
            return None, number_explored

        current_node = frontier.pop()
    
    return current_node, number_explored

def total_distance( route, maze, gridsize ):
    """Return the sum of all the distances traveled on route."""
    curr = route[0]; nxt = route[0]
    total = 0
    for step in route:
        # update to newest value, then perform distance calc between curr and nxt
        nxt = step  
        total += sqrt(  pow((nxt[0] - curr[0]) * gridsize[0], 2 ) +   \
                        pow((nxt[1] - curr[1]) * gridsize[1], 2 ) +   \
                        pow(maze[int(nxt[0]/gridsize[0])][int(nxt[1]/gridsize[1])] -        \
                            maze[int(curr[0]/gridsize[0])][int(curr[1]/gridsize[1])], 2))
        curr = step
    return round(total,2)

def astarRoute3D( intensitymap : np.array, slopemap : np.array, maxslope=100, gridsize=(1,1) ):
    """Perform A* Search Algorithm on elevation map (using slopemap to determine obstacles) and return route as list."""
    starttime = time.time()
    final_node, number_explored = astar_search( maze = intensitymap,
                                                obstacles = slopemap,
                                                maxslope = maxslope,
                                                gridsize = gridsize)
    endtime = round((time.time() - starttime), 2)
    
    print("A* SEARCH:")
    solution = []
    if final_node is None:
        print("No path exists!\n")
        return None
    else:
        node = final_node
        steps = 0
        while node.parent is not None:
            if steps == 0:
                 print(f"    Total cost of path: {round(node.path_cost,2)}")

            # reformat solution to return as [ (x1,y1), (x2,y2) ... ]
            state = node.state
            solution.append( [state[1], state[0]] )
            steps += 1
            node = node.parent

        # reformat solution to return as [ (x1,y1), (x2,y2) ... ]
        state = node.state
        solution.append( [state[1], state[0]] )
        
        print(f"    Total steps on path: {steps}")
        print(f"    Total states explored: {number_explored}")
        print(f"    Total time till solution: {endtime} [s]")

        # consider point to point distances not accounding for variations in elevation (height) resulting
        # in under estimating the actual distance travelled
        print(f"    Estimated distance travelled [m]: {total_distance(solution[::-1], intensitymap, gridsize)}")
    return solution[::-1]   # [::-1] reverse to start in correct order