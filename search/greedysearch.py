from priorityqueue import PriorityQueue
import random
import mazegen

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


def valid_space(maze, space):
    return 0 <= space[0] < len(maze) \
           and 0 <= space[1] < len(maze[0]) \
           and maze[space[0]][space[1]] == '.'


def greedy_search(maze, start=(0, 0), goal=None):
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
        up = (current_state[0] - 1, current_state[1])
        
        for space in [right, left, down, up]:
            if valid_space(maze, space) \
               and space not in explored:
                node = Node(space, parent=current_node)
                frontier.push(node)

        if frontier.length() == 0:
            return None, number_explored

        current_node = frontier.pop()
    
    return current_node, number_explored

# here is the "main" code, we generate a new maze then try the search
# try changing the random seed to try different mazes (not all are solvable)
height = 10
width = 20

random.seed(0)
maze = mazegen.mazegen(height, width)
final_node, number_explored = greedy_search(maze)

if final_node is None:
    print("No path exists!\n")
    mazegen.print_maze(maze)
else:
    node = final_node
    steps = 0
    while node.parent is not None:
        state = node.state
        maze[state[0]][state[1]] = 'X'
        steps += 1
        node = node.parent

    state = node.state
    maze[state[0]][state[1]] = 'X'
    mazegen.print_maze(maze)
    
    print(f"Total steps on path: {steps}")
    print(f"Total states explored: {number_explored}")