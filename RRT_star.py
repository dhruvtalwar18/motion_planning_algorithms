import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
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
########################################################### RRT* ###########################################################
class Node:
    def __init__(self, position):
        self.position = position
        self.parent = None
        self.cost = 0.0


class RRTStarPlanner:
    def __init__(self, boundary, blocks, resolution, goal_threshold=0.5, max_iterations=100000000000000, max_distance=2,rewire_radius=0.1,sampling_method='uniform'):
        self.boundary = boundary
        self.blocks = blocks
        self.resolution = resolution
        self.goal_threshold = goal_threshold
        self.max_iterations = max_iterations
        self.max_distance = max_distance
        self.rewire_radius = rewire_radius
        self.sampling_method = sampling_method
        self.nodes = []
        self.kdtree = None
        self.start_node = None
        self.goal_node = None
        self.nodes_visited = 0
        self.ctr = 0

    def plan(self, start, goal):
        self.start_node = Node(start)
        self.goal_node = Node(goal)
        self.nodes = [self.start_node]
        self.kdtree = KDTree([start])

        for iteration in range(self.max_iterations):
            random_point = self.generate_random_point()
            nearest_node = self.find_nearest_node(random_point)
            new_node = self.steer(nearest_node, random_point)

            if self.check_collision_free(nearest_node.position, new_node.position):
                nearby_nodes = self.find_nearby_nodes(new_node)
                best_parent = self.choose_best_parent(nearby_nodes, nearest_node, new_node)
                self.add_node(new_node, best_parent)
                self.rewire_nearby_nodes(nearby_nodes, new_node)

                if self.is_goal_reached(new_node):
                    return self.generate_path(new_node)

            self.nodes_visited += 1

        return None
###################################################### Change Sampling Method ######################################################
    def generate_random_point(self):

        x_min, y_min, z_min, _, _, _ = self.boundary[0][0:6]
        x_max, y_max, z_max, _, _, _ = self.boundary[0][3:9]
        x_range = x_max - x_min
        y_range = y_max - y_min
        z_range = z_max - z_min

        if self.sampling_method == 'uniform':

            if np.random.uniform() < 0.05:
                random_point = self.goal_node.position
            else:
                random_point = np.array([np.random.uniform(x_min, x_max),
                                         np.random.uniform(y_min, y_max),
                                         np.random.uniform(z_min, z_max)])
            if self.ctr ==0:
                print("Uniform Sampling")
                self.ctr= self.ctr+1

        elif self.sampling_method == 'gaussian':
            mean = np.array([(x_min + x_max) / 2, (y_min + y_max) / 2, (z_min + z_max) / 2])
            cov = np.diag([(x_range / 6) ** 2, (y_range / 6) ** 2, (z_range / 6) ** 2])
            random_point = np.random.multivariate_normal(mean, cov)
            if self.ctr ==0:
                print("Gaussian Sampling")
                self.ctr+=1

        elif self.sampling_method == 'goal_biased':
            if np.random.uniform() < 0.40:
                random_point = self.goal_node.position
            else:
                random_point = np.array([np.random.uniform(x_min, x_max),
                                         np.random.uniform(y_min, y_max),
                                         np.random.uniform(z_min, z_max)])
            if self.ctr ==0:
                print("Goal Biased Sampling")
                self.ctr+=1

        return random_point
##################################################################################################################################
    def find_nearest_node(self, point):
        distances, indices = self.kdtree.query(point)
        nearest_node = self.nodes[indices]
        return nearest_node

    def steer(self, from_node, to_point):
        direction = to_point - from_node.position
        distance = np.linalg.norm(direction)
        if distance > self.max_distance:
            direction = (direction / distance) * self.max_distance
        new_position = from_node.position + direction
        new_node = Node(new_position)
        return new_node

    def check_collision_free(self, from_point, to_point):
        for block in self.blocks:
            if check_collision(from_point, to_point, block[:3], block[3:]):
                return False
        return True

    def find_nearby_nodes(self, node):
        r = min(self.rewire_radius * np.sqrt(np.log(len(self.nodes)) / len(self.nodes)), self.max_distance)
        nearby_indices = self.kdtree.query_ball_point(node.position, r)
        nearby_nodes = [self.nodes[i] for i in nearby_indices]
        return nearby_nodes

    def choose_best_parent(self, nearby_nodes, nearest_node, new_node):
        best_cost = nearest_node.cost + np.linalg.norm(nearest_node.position - new_node.position)
        best_node = nearest_node

        for node in nearby_nodes:
            if not self.check_collision_free(node.position, new_node.position):
                continue
            cost = node.cost + np.linalg.norm(node.position - new_node.position)
            if cost < best_cost:
                best_node = node
                best_cost = cost

        return best_node

    def add_node(self, new_node, best_parent):
        new_node.parent = best_parent
        new_node.cost = best_parent.cost + np.linalg.norm(best_parent.position - new_node.position)
        self.nodes.append(new_node)
        self.kdtree = KDTree([node.position for node in self.nodes])

    def rewire_nearby_nodes(self, nearby_nodes, new_node):
        for node in nearby_nodes:
            if node == new_node.parent:
                continue
            if self.check_collision_free(new_node.position, node.position):
                cost = new_node.cost + np.linalg.norm(new_node.position - node.position)
                if cost < node.cost:
                    node.parent = new_node
                    node.cost = cost

    def is_goal_reached(self, node):
        return np.linalg.norm(node.position - self.goal_node.position) <= self.goal_threshold

    def generate_path(self, node):
        path = []
        current = node
        while current is not None:
            path.append(current.position)
            current = current.parent
        path = np.array(path[::-1])  # Reverse the path

        # Check if the last node is within the goal threshold
        if np.linalg.norm(node.position - self.goal_node.position) <= self.goal_threshold:
            path = np.vstack((path, self.goal_node.position))

        return path

###################################################### Collision Checking other than maze ######################################################
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


#For maze map uncomment this
# def check_collision(line_start, line_end, box_min, box_max, resolution=0.1):
#     """
#     Check collision between a line segment and an AABB (Axis-Aligned Bounding Box) in 3D space with finer resolution.
#     """
#
#     # Calculate the number of sub-boxes in each dimension
#     num_sub_boxes = (
#         int((box_max[0] - box_min[0]) / resolution),
#         int((box_max[1] - box_min[1]) / resolution),
#         int((box_max[2] - box_min[2]) / resolution)
#     )
#
#     # Iterate over all the sub-boxes
#     for i in range(num_sub_boxes[0]):
#         for j in range(num_sub_boxes[1]):
#             for k in range(num_sub_boxes[2]):
#                 # Calculate the minimum and maximum points of the current sub-box
#                 sub_box_min = (
#                     box_min[0] + i * resolution,
#                     box_min[1] + j * resolution,
#                     box_min[2] + k * resolution
#                 )
#                 sub_box_max = (
#                     sub_box_min[0] + resolution,
#                     sub_box_min[1] + resolution,
#                     sub_box_min[2] + resolution
#                 )
#
#                 # Check if the line segment intersects the current sub-box
#                 if check_collision_sub_box(line_start, line_end, sub_box_min, sub_box_max):
#                     return True
#
#     return False
#
#
# def check_collision_sub_box(line_start, line_end, box_min, box_max):
#     """
#     Check collision between a line segment and a sub-box (part of an AABB) in 3D space.
#     """
#     # Check if any of the line segment points is inside the sub-box
#     if (
#             box_min[0] <= line_start[0] <= box_max[0] and
#             box_min[1] <= line_start[1] <= box_max[1] and
#             box_min[2] <= line_start[2] <= box_max[2]
#     ) or (
#             box_min[0] <= line_end[0] <= box_max[0] and
#             box_min[1] <= line_end[1] <= box_max[1] and
#             box_min[2] <= line_end[2] <= box_max[2]
#     ):
#         return True
#
#     # Check if the line segment intersects any of the sub-box's faces
#     for i in range(3):
#         if line_start[i] < box_min[i] and line_end[i] < box_min[i]:
#             continue
#         if line_start[i] > box_max[i] and line_end[i] > box_max[i]:
#             continue
#         t = (box_min[i] - line_start[i]) / (line_end[i] - line_start[i] + 0.000000001)
#         if 0 <= t <= 1:
#             intersection_point = (
#                 line_start[0] + t * (line_end[0] - line_start[0]),
#                 line_start[1] + t * (line_end[1] - line_start[1]),
#                 line_start[2] + t * (line_end[2] - line_start[2])
#             )
#             if (
#                     box_min[(i + 1) % 3] <= intersection_point[(i + 1) % 3] <= box_max[(i + 1) % 3] and
#                     box_min[(i + 2) % 3] <= intersection_point[(i + 2) % 3] <= box_max[(i + 2) % 3]
#             ):
#                 return True
#
#     return False


################################################## LOAD MAP #########################################
mapfile = './maps/flappy_bird.txt'
start = np.array([0.5,2.5,5.5])
goal = np.array([19.0,2.5,5.5]) #Flappy_bird
#####################################################################################################
# mapfile = './maps/room.txt'
# start = np.array([1,5,1.5])
# goal = np.array([9,7,1.5]) #room
#####################################################################################################
# mapfile = './maps/monza.txt'
# start = np.array([0.5,1.0,4.9])
# goal = np.array([3.8,1.0,0.1]) #monza
#####################################################################################################
# mapfile = './maps/tower.txt'
# start = np.array([2.5,4,0.5])
# goal = np.array([4,2.5,19.5]) #tower
#####################################################################################################
# mapfile = './maps/window.txt'
# start = np.array([0.2,-4.9,0.2])
# goal = np.array([6.0,18.0,3.0]) #window
#####################################################################################################
# mapfile = './maps/single_cube.txt'
# start = np.array([2.3,2.3,1.3])
# goal = np.array([7,7,5.5]) #cube
#####################################################################################################

#####################################################################################################
boundary, blocks = load_map(mapfile)
resolution = 0.1  # Adjust the resolution as needed
t0 = tic()
############################################## Choose Sampling Method  ##########################################
planner = RRTStarPlanner(boundary, blocks, resolution, sampling_method='uniform')
# planner = RRTStarPlanner(boundary, blocks, resolution, sampling_method='gaussian')
# planner = RRTStarPlanner(boundary, blocks, resolution, sampling_method='goal_biased')

#######################################################################################################

######################################################################################################



path = planner.plan(start, goal)
path_length = np.sum(np.sqrt(np.sum(np.diff(path, axis=0) ** 2, axis=1)))
print('Path length = %.3f' % path_length)
toc(t0, "Planning")
############################################## Plot Figure ##########################################
if path is not None:
# Create the figure and axes
  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')

# Plot the obstacles and get the handles
  hb = draw_block_list(ax, blocks)

# Plot the path
  ax.plot(path[:, 0], path[:, 1], path[:, 2], 'r-', label='Path')

# Plot start and goal dots
  ax.scatter(start[0], start[1], start[2], c='green', marker='o', label='Start')
  ax.scatter(goal[0], goal[1], goal[2], c='blue', marker='o', label='Goal')

# Set the labels and limits
  ax.set_xlabel('X')
  ax.set_ylabel('Y')
  ax.set_zlabel('Z')
  ax.set_xlim(boundary[0, 0], boundary[0, 3])
  ax.set_ylim(boundary[0, 1], boundary[0, 4])
  ax.set_zlim(boundary[0, 2], boundary[0, 5])
  print('Nodes visited =', planner.nodes_visited)
# Add the legend
  ax.legend()
# Show the plot
  plt.show()
else:
    print("Path not found!")

