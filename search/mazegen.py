import random

def mazegen(height, width, obstacle_weight=0.2):
    """
    generates a grid of '.' and '#'
    obstacle weight is probability that each space will be # (0.2 by default)
    sets top left and bottom right spaces to be clear
    BUT otherwise does not guarantee that there is a clear path between them!    
    """
    maze = [[random.choices(['.', '#'], weights=[1-obstacle_weight, obstacle_weight])[0] \
             for _ in range(width)] for _ in range(height)]
    
    maze[0][0] = '.'
    maze[-1][-1] = '.'
    return maze

def print_maze(maze):
    print('\n'.join([''.join(i) for i in maze]))

def str2array(maze):
    return [list(s) for s in maze.split('\n')]