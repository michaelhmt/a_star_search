import heapq
import os

import matplotlib.pyplot as plt
import numpy as np
import time
import cProfile
import pstats

import massive_maze as mazes

SEARCH_DIRECTIONS = [(0, -1),  # Left
                     (0, 1),   # Right
                     (-1, 0),  # down
                     (1, 0)]   # up


def plot_path(maze, path):
    # Convert the maze (which is a list of lists) to a NumPy array for easier processing.
    maze_array = np.array(maze)

    # Create an array for the path to overlay on the maze.
    path_array = np.zeros_like(maze_array)

    # Set the positions on the path to 1 (or any other value you want to display).
    for position in path:
        path_array[position] = 1

    # Plot the maze and the path.
    plt.imshow(maze_array, cmap='Greys', interpolation='nearest')  # 'Greys' colormap for the maze
    plt.imshow(path_array, cmap='spring', interpolation='nearest',
               alpha=0.5)  # 'spring' colormap for the path with transparency

    # Optionally, you can mark the start and end positions.
    start_x, start_y = path[0]
    end_x, end_y = path[-1]
    plt.scatter(start_y, start_x, marker='o', color='green', label='Start')  # Inverted due to row-column vs x-y
    plt.scatter(end_y, end_x, marker='x', color='red', label='End')

    # Disable the axis.
    plt.axis('off')

    # Add a legend to the plot.
    plt.legend()

    # Show the plot.
    plt.show()



class Node:
    """
    A node that represent a step on the grid we will be solving
    """
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g_cost = 0  # the cost of movement from the start of the path find to this node
        self.h_cost = 0  # the estimated cost to get from this node to th end node, is a guess
        self.f_cost = 0  # the total cost taking g and h into account

    def __eq__(self, other):
        # type: (Node) -> bool
        return self.position == other.position

    def __lt__(self, other):
        # type: (Node) -> bool
        # less than comparison
        return self.f_cost < other.f_cost

def complie_previous_position(current_pos_node):
    # type: (Node) -> list
    taken_path = list()
    previous_node = current_pos_node
    while previous_node is not None:
        taken_path.append(previous_node.position)
        previous_node = previous_node.parent
    # return the position reversed, so we start at the start position
    return taken_path[::-1]

def find_node_potential_directions(current_node, direction, maze):
    # type: (Node, tuple[int, int], list[list[int]]) -> Node | None
    new_node_pos = (current_node.position[0] + direction[0],
                    current_node.position[1] + direction[1])

    is_not_in_maze_x = new_node_pos[0] > (len(maze) - 1) or new_node_pos[0] < 0
    if is_not_in_maze_x:return None
    is_not_in_maze_y = new_node_pos[1] > (len(maze[len(maze)-1]) -1) or new_node_pos[1] < 0
    #print(f"{} {}")
    if is_not_in_maze_y:return None

    # ensure that we are allowed to travel to this position
    if maze[new_node_pos[0]][new_node_pos[1]] != 0:
        return None
    return Node(parent=current_node, position=new_node_pos)


def a_star_search(maze, start, end):

    # make start and end nodes
    start_node = Node(None, start)
    end_node = Node(None, end)

    # will contain a list of nodes that we are aware of as options for moving but have no yet assessed
    frontier_list = list()
    # will track all the nodes that we have been to already
    visted_list = list()

    # heap the frontier, this using the heap queue algorithm, essentially the smallest value will always be at the root
    # of the heap, making it very quick and easy to find an item if it is evaluated as a small number by python
    heapq.heapify(frontier_list)
    # add our start node/ current position to the search
    heapq.heappush(frontier_list, start_node)

    number_of_iterations = 0
    max_iterations = (len(maze) // 2) ** 10

    # loop until we have no potential paths to explore
    print("started search")
    while len(frontier_list) > 0:
        number_of_iterations += 1

        if number_of_iterations > max_iterations:
            # stopping the search as it has gone on to long
            return None

        current_position = heapq.heappop(frontier_list) # type: Node
        visted_list.append(current_position)
        # if we have reached the goal return the path we took
        if current_position == end_node:
            return complie_previous_position(current_position)

        children = list()
        for search_direction in SEARCH_DIRECTIONS:
            direction_node = find_node_potential_directions(current_position,
                                                            search_direction,
                                                            maze)

            if not direction_node:
                # for whatever reason we cant move here
                continue
            children.append(direction_node)

            for child in children:  # type: Node
                if child in visted_list:
                    # we have already been to this node
                    continue

                # set up the values of the astar search formula
                child.g_cost = current_position.g_cost + 1 # we have taken another step

                # the combination of how far we have left to travel, in a straight line
                x_h_cost = abs(child.position[0] - end_node.position[0])
                y_h_cost = abs(child.position[1] - end_node.position[1])
                child.h_cost = x_h_cost + y_h_cost

                child.f_cost = child.g_cost + child.h_cost

                for vistable_node in frontier_list: # type: Node
                    if child == vistable_node and child.g_cost > vistable_node.g_cost:
                        # if we have seen this node before don't add it, it would never be a potential step anyway
                        continue

                heapq.heappush(frontier_list, child)
    return None


def main():
    # maze = [[0, 0, 0, 0, 0, 1, 0, 0],
    #         [0, 0, 1, 0, 1, 0, 0, 0],
    #         [0, 0, 1, 1, 1, 0, 1, 0],
    #         [0, 0, 0, 0, 1, 0, 1, 0],
    #         [1, 1, 0, 1, 1, 0, 1, 0],
    #         [0, 0, 0, 0, 0, 0, 1, 0],
    #         [0, 0, 1, 1, 1, 1, 1, 0]]
    #
    # start = (0, 0)
    # goal = (6, 7)

    start_time = time.time()
    path = a_star_search(mazes.HUGE_MAZE, mazes.HUGE_MAZE_START, mazes.HUGE_MAZE_GOAL)
    end_time = time.time()
    print(f"time taken to solve: {end_time - start_time}")
    return path, mazes.HUGE_MAZE

if __name__ == "__main__":
    profiler = cProfile.Profile()
    profiler.enable()
    path, maze = main()
    profiler.disable()
    stats = pstats.Stats(profiler).dump_stats('profile_output.pstats')
    os.system("snakeviz profile_output.pstats")
    if path:
        plot_path(maze, path)