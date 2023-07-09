import numpy as np
import time
from pqdict import PQDict
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def tic():
    return time.time()

def toc(tstart, nm=""):
    print('%s took: %s sec.\n' % (nm, (time.time() - tstart)))
def load_map(fname):
    '''
    Loads the bounady and blocks from map file fname.

    boundary = [['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']]

    blocks = [['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b'],
              ...,
              ['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']]
    '''
    mapdata = np.loadtxt(fname, dtype={'names': ('type', 'xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax', 'r', 'g', 'b'), \
                                       'formats': ('S8', 'f', 'f', 'f', 'f', 'f', 'f', 'f', 'f', 'f')})
    blockIdx = mapdata['type'] == b'block'
    boundary = mapdata[~blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax', 'r', 'g', 'b']].view('<f4').reshape(
        -1, 11)[:, 2:]
    blocks = mapdata[blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax', 'r', 'g', 'b']].view('<f4').reshape(-1,
                                                                                                                    11)[
             :, 2:]
    return boundary, blocks

def draw_map(boundary, blocks, start, goal):
    '''
    Visualization of a planning problem with environment boundary, obstacle blocks, and start and goal points
    '''
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    hb = draw_block_list(ax, blocks)
    hs = ax.plot(start[0:1], start[1:2], start[2:], 'ro', markersize=7, markeredgecolor='k')
    hg = ax.plot(goal[0:1], goal[1:2], goal[2:], 'go', markersize=7, markeredgecolor='k')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim(boundary[0, 0], boundary[0, 3])
    ax.set_ylim(boundary[0, 1], boundary[0, 4])
    ax.set_zlim(boundary[0, 2], boundary[0, 5])
    return fig, ax, hb, hs, hg


def draw_block_list(ax, blocks):
    '''
    Subroutine used by draw_map() to display the environment blocks
    '''
    v = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0], [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]],
                 dtype='float')
    f = np.array([[0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7], [0, 1, 2, 3], [4, 5, 6, 7]])
    clr = blocks[:, 6:] / 255
    n = blocks.shape[0]
    d = blocks[:, 3:6] - blocks[:, :3]
    vl = np.zeros((8 * n, 3))
    fl = np.zeros((6 * n, 4), dtype='int64')
    fcl = np.zeros((6 * n, 3))
    for k in range(n):
        vl[k * 8:(k + 1) * 8, :] = v * d[k] + blocks[k, :3]
        fl[k * 6:(k + 1) * 6, :] = f + k * 8
        fcl[k * 6:(k + 1) * 6, :] = clr[k, :]

    if type(ax) is Poly3DCollection:
        ax.set_verts(vl[fl])
    else:
        pc = Poly3DCollection(vl[fl], alpha=0.25, linewidths=1, edgecolors='k')
        pc.set_facecolor(fcl)
        h = ax.add_collection3d(pc)
        return h


def runtest(mapfile, start, goal, path, verbose=True):
    """
    This function:
    * loads the provided mapfile
    * creates a motion planner
    * plans a path from start to goal
    * checks whether the path is collision-free and reaches the goal
    * computes the path length as a sum of the Euclidean norm of the path segments
    """
    # Load a map and instantiate a motion planner
    boundary, blocks = load_map(mapfile)

    # Display the environment
    if verbose:
        fig, ax, hb, hs, hg = draw_map(boundary, blocks, start, goal)

    # Call the motion planner

    # Plot the path
    if verbose:
        ax.plot(path[:, 0], path[:, 1], path[:, 2], 'r-')
        plt.show()

    # Check collision and goal reached
    collision = True
    goal_reached = sum((path[-1] - goal) ** 2) <= 0.1
    if len(path) > 1:
        for i in range(len(path) - 1):
            if check_collision(path[i], path[i + 1], boundary[0][0:3], boundary[0][3:6]):
                collision = False
                break

    success = (not collision) and goal_reached
    path_length = np.sum(np.sqrt(np.sum(np.diff(path, axis=0) ** 2, axis=1)))
    return success, path_length


def check_collision(line_start, line_end, box_min, box_max):
    """
    Check collision between a line segment and an AABB (Axis-Aligned Bounding Box) in 3D space.
    """

    # Check if any of the line segment points is inside the AABB
    if (
            box_min[0] <= line_start[0] <= box_max[0] and
            box_min[1] <= line_start[1] <= box_max[1] and
            box_min[2] <= line_start[2] <= box_max[2]
    ) or (
            box_min[0] <= line_end[0] <= box_max[0] and
            box_min[1] <= line_end[1] <= box_max[1] and
            box_min[2] <= line_end[2] <= box_max[2]
    ):
        return True

    # Check if the line segment intersects any of the AABB's faces
    for i in range(3):
        if line_start[i] < box_min[i] and line_end[i] < box_min[i]:
            continue
        if line_start[i] > box_max[i] and line_end[i] > box_max[i]:
            continue
        t = (box_min[i] - line_start[i]) / (line_end[i] - line_start[i] + 0.000000001)
        if 0 <= t <= 1:
            intersection_point = (
                line_start[0] + t * (line_end[0] - line_start[0]),
                line_start[1] + t * (line_end[1] - line_start[1]),
                line_start[2] + t * (line_end[2] - line_start[2])
            )
            if (
                    box_min[(i + 1) % 3] <= intersection_point[(i + 1) % 3] <= box_max[(i + 1) % 3] and
                    box_min[(i + 2) % 3] <= intersection_point[(i + 2) % 3] <= box_max[(i + 2) % 3]
            ):
                return True

    return False


def test_single_cube(res, epsi, verbose=False):
    print('Running single cube test...\n')

    start = np.array([2.3, 2.3, 1.3])
    goal = np.array([7.0, 7.0, 5.5])
    path, node_max = epsilon_a_star(res, epsi, start, goal,'./maps/single_cube.txt')
    success, pathlength = runtest('./maps/single_cube.txt', start, goal, path, verbose)

    print('Success: %r' % success)
    print('Total NODES visited', node_max)
    print('Path length: %d' % pathlength)
    print('\n')


def test_maze(res, epsi, verbose=False):
    print('Running maze test...\n')

    start = np.array([0.0, 0.0, 1.0])
    goal = np.array([12.0, 12.0, 5.0])
    path, node_max = epsilon_a_star(res, epsi, start, goal,
                                    './maps/maze.txt')
    success, pathlength = runtest('./maps/maze.txt', start, goal, path, verbose)
    print('Success: %r' % success)
    print('Total NODES visited', node_max)
    print('Path length: %d' % pathlength)
    print('\n')


def test_window(res, epsi, verbose=False):
    print('Running window test...\n')
    start = np.array([0.2, -4.9, 0.2])
    goal = np.array([6.0, 18.0, 3.0])

    path, node_max = epsilon_a_star(res, epsi, start, goal,'./maps/window.txt')  # Call A* Algo function
    success, pathlength = runtest('./maps/window.txt', start, goal, path, verbose)

    print('Success: %r' % success)
    print('Total NODES visited', node_max)
    print('Path length: %d' % pathlength)
    print('\n')


def test_tower(res, epsi, verbose=False):
    print('Running tower test...\n')

    start = np.array([2.5, 4.0, 0.5])
    goal = np.array([4.0, 2.5, 19.5])
    path, node_max = epsilon_a_star(res, epsi, start, goal,
                                    './maps/tower.txt')  # Call A* Algo function
    success, pathlength = runtest('./maps/tower.txt', start, goal, path, verbose)

    print('Success: %r' % success)
    print('Total NODES visited', node_max)
    print('Path length: %d' % pathlength)
    print('\n')


def test_flappy_bird(res, epsi, verbose=False):
    print('Running flappy bird test...\n')
    start = np.array([0.5, 2.5, 5.5])
    goal = np.array([19.0, 2.5, 5.5])

    path, node_max = epsilon_a_star(res, epsi, start, goal,
                                    './maps/flappy_bird.txt')  # Call A* Algo function
    success, pathlength = runtest('./maps/flappy_bird.txt', start, goal, path, verbose)

    print('Success: %r' % success)
    print('Total NODES visited', node_max)
    print('Path length: %d' % pathlength)
    print('\n')


def test_room(res, epsi, verbose=False):
    print('Running room test...\n')
    start = np.array([1.0, 5.0, 1.5])
    goal = np.array([9.0, 7.0, 1.5])

    path, node_max = epsilon_a_star(res, epsi, start, goal,
                                    './maps/room.txt')  # Call A* Algo function
    success, pathlength = runtest('./maps/room.txt', start, goal, path, verbose)

    print('Success: %r' % success)
    print('Total NODES visited', node_max)
    print('Path length: %d' % pathlength)
    print('\n')


def test_monza(res, epsi, verbose=False):
    print('Running monza test...\n')

    start = np.array([0.5, 1.0, 4.9])
    goal = np.array([3.8, 1.0, 0.1])
    path, node_max = epsilon_a_star(res, epsi, start, goal,
                                    './maps/monza.txt')  # Call A* Algo function
    success, pathlength = runtest('./maps/monza.txt', start, goal, path, verbose)

    print('Success: %r' % success)
    print('Total NODES visited', node_max)
    print('Path length: %d' % pathlength)
    print('\n')


def build_discrete_world(mapfile, res, start, goal):
    """
    Build the 3D discrete world and transform the start and goal positions accordingly.
    """
    boundary, blocks = load_map(mapfile)

    x_min, y_min, z_min, _, _, _ = boundary[0][0:6]
    x_max, y_max, z_max, _, _, _ = boundary[0][3:9]

    x_n = int((x_max - x_min) / res) + 1
    y_n = int((y_max - y_min) / res) + 1
    z_n = int((z_max - z_min) / res) + 1

    discrete_world = np.zeros((x_n, y_n, z_n))
    start_position = ((start - np.array([x_min, y_min, z_min])) / res).astype(int)
    goal_position = ((goal - np.array([x_min, y_min, z_min])) / res).astype(int)

    for block in blocks:
        x_start, y_start, z_start, _, _, _ = block[0:6] - np.array([x_min, y_min, z_min, x_min, y_min, z_min])
        x_start, y_start, z_start = (x_start / res).astype(int), (y_start / res).astype(int), (z_start / res).astype(
            int)
        x_end, y_end, z_end, _, _, _ = block[3:9] - np.array([x_min, y_min, z_min, x_min, y_min, z_min])
        x_end, y_end, z_end = (x_end / res).astype(int), (y_end / res).astype(int), (z_end / res).astype(int)
        discrete_world[x_start:x_end + 1, y_start:y_end + 1, z_start:z_end + 1] = np.inf

    return discrete_world, start_position, goal_position


def children(cell):
    offsets = [-1, 0, 1]
    coords = []
    for dx in offsets:
        for dy in offsets:
            for dz in offsets:
                if dx == 0 and dy == 0 and dz == 0:
                    continue
                coords.append([cell[0] + dx, cell[1] + dy, cell[2] + dz])

    return np.array(coords)


def extract_path(PARENT, start, goal, mapfile, res):
    boundary, blocks = load_map(mapfile)
    x_min, y_min, z_min, _, _, _ = boundary[0][0:6]
    offset = np.array([x_min, y_min, z_min])

    path = [goal]
    current = goal
    while not np.array_equal(current, start):
        current = PARENT[tuple(current)]
        path.append(current)

    path = np.array(path[::-1])  # Reverse the path and convert it to a NumPy array

    # Include start position joining the first node
    path = np.insert(path, 0, start, axis=0)

    path = (path - np.array([1, 1, 1])) * res + offset  # Adjust the path coordinates based on the resolution and offset

    return path


import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)


def epsilon_a_star(resolution, epsilon, start, goal, mapfile):
    print('A* Start')
    t0 = tic()
    discrete_world, start_position, goal_position = build_discrete_world(mapfile, resolution, start, goal)
    boundary, blocks = load_map(mapfile)

    x, y, z = discrete_world.shape
    cost_grid = np.inf * np.ones((x, y, z))
    cost_grid[tuple(start_position.astype(np.int))] = 0

    OPEN = PQDict({tuple(start_position.astype(np.int)): 0})
    CLOSED = PQDict()
    PARENT = {}

    points = start
    node_max = 0
    while tuple(goal_position) not in CLOSED:
        if len(OPEN) > node_max:
            node_max = len(OPEN)

        current, current_cost = OPEN.popitem()
        CLOSED.additem(current, current_cost)
        children_current = children(np.array([current])[0])

        for child in children_current:
            if 0 <= child[0] < x and 0 <= child[1] < y and 0 <= child[2] < z and discrete_world[tuple(child)] == 0:
                if tuple(child) not in CLOSED:
                    collision_free = True
                    for block in blocks:
                        if check_collision(current, child, block[:3], block[3:]):
                            collision_free = False
                            break
                    if collision_free:
                        child_cost = cost_grid[tuple(current)] + round(np.linalg.norm(current - child), 3)
                        if child_cost < cost_grid[tuple(child)]:
                            cost_grid[tuple(child)] = child_cost
                            PARENT[tuple(child)] = tuple(current)

                            if tuple(child) in OPEN:
                                OPEN.updateitem(tuple(child),
                                                child_cost + epsilon * heuristic(child, list(goal_position), epsilon))
                            else:
                                OPEN.additem(tuple(child),
                                             child_cost + epsilon * heuristic(child, list(goal_position), epsilon))

    path = extract_path(PARENT, start_position, goal_position, mapfile, resolution)
    toc(t0, "A* End")
    return path, node_max
def heuristic(cell, goal, epsilon, heuristic_type='euclidean'):
    if heuristic_type == 'euclidean':
        # Euclidean distance
        distance = np.linalg.norm(np.array(goal) - np.array(cell))
    elif heuristic_type == 'manhattan':
        # Manhattan distance
        distance = abs(goal[0] - cell[0]) + abs(goal[1] - cell[1])
    elif heuristic_type == 'chebyshev':
        # Chebyshev distance
        distance = max(abs(goal[0] - cell[0]), abs(goal[1] - cell[1]))
    else:
        raise ValueError('Invalid heuristic type.')

    h = round(epsilon * distance, 3)
    return h


if __name__ == "__main__":
    resolution = 0.1
    epsilon = 200
    #test_single_cube(resolution,epsilon,True) Done
    # test_maze(resolution,epsilon,True)
    test_flappy_bird(resolution,epsilon,True)
    #test_monza(resolution,epsilon,True) Done
    #test_window(resolution,epsilon,True) Done
    #test_tower(resolution,epsilon,True)
    #test_room(resolution,epsilon,True) Done