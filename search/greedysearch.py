"""
Description:
    -   ...

By Oliver Heilmann
Modified from ...
"""
import time
import numpy as np
from math import sqrt
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


def greedy_search(maze, maxvelocity, maxslope, start=(0, 0), goal=None, gridsize=(1,1)):
    if goal is None:
        goal = (len(maze) - 1, len(maze[0]) - 1)

    # here's our euclidean distance heurstic in 3D space, as a lambda expression
    # f(n) = h(n) where h(n) is the straight line distance to the goal from current point
    heuristic = lambda node: sqrt(  pow( (goal[0] - node.state[0]) * gridsize[0], 2 ) +         \
                                    pow( (goal[1] - node.state[1]) * gridsize[1], 2 ) +         \
                                    pow( maze[goal].elevation - maze[node.state].elevation, 2)) \
                                / maxvelocity

    frontier = Frontier(heuristic, Node(start))
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
            if 0 <= space[0] < len(maze) and 0 <= space[1] < len(maze[0]):
                if not maze[space].isobstacle(max_slope=maxslope) and space not in explored:
                    node = Node(space, parent=current_node)
                    frontier.push(node)

        if frontier.length() == 0:
            return None, number_explored

        current_node = frontier.pop()
    
    return current_node, number_explored

def totalDistTime( route, maze, gridsize ):
    """Return the sum of all the distances and times accumulated on route."""
    route = tuple([(r[1],r[0]) for r in route]) # make tuple and swap r, c's for correct lookups
    curr = route[0]; nxt = route[0]
    total_Dist = 0
    total_Time = 0
    dD = lambda c, n, g : sqrt( pow((n[0] - c[0]) * g[0], 2 ) +                 \
                                pow((n[1] - c[1]) * g[1], 2 ) +                 \
                                pow(maze[n].elevation - maze[c].elevation, 2))
    for step in route:
        # update to newest value, then perform distance calc between curr and nxt
        nxt = step
        # if velocity in tile = 0 then this means IOP tile resolution is so much lower
        # than the image pixel resolution that a path is made over obstacle tiles e.g.
        # over two river tiles. The averaging between neighbouring pixels causes this
        # issue. To avoid this, increase XDIMENSION and YDIMENSION or drop the 
        # PIXEL_RESOLUTION value also!
        if (maze[curr].velocity + maze[nxt].velocity) > 0.:
            total_Dist += dD( curr, nxt, gridsize )
            total_Time += 2 * dD( curr, nxt, gridsize ) / (maze[curr].velocity + maze[nxt].velocity)
        else:
            raise ValueError("""\n
    If velocity in tile = 0 then this means IOP tile resolution is so much lower
    than the image pixel resolution that a path is made over obstacle tiles e.g.
    over two river tiles. The averaging between neighbouring pixels causes this
    issue. To avoid this, increase XDIMENSION and YDIMENSION or drop the 
    PIXEL_RESOLUTION value also!
                            """)
        curr = step
    return round(total_Dist,2), round(total_Time,2)

def greedyRoute3D( terrain, maxvelocity=50, maxslope=100, gridsize=(1,1) ):
    """Perform Greedy Search Algorithm on elevation map (using slopemap to determine obstacles) and return route as list."""
    starttime = time.time()
    final_node, number_explored = greedy_search(maze = terrain,
                                                maxvelocity = maxvelocity,
                                                maxslope = maxslope,
                                                gridsize = gridsize)
    endtime = round((time.time() - starttime), 2)
    
    print("GREEDY SEARCH:")
    solution = []
    if final_node is None:
        print("No path exists!\n")
        return None
    else:
        node = final_node
        steps = 0
        while node.parent is not None:
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
        d, t = totalDistTime(solution[::-1], terrain, gridsize)
        print("    Estimated distance travelled [m]: {}\n    Estimated time taken [s]: {}".format(d,t))
    return solution[::-1]   # [::-1] reverse to start in correct order