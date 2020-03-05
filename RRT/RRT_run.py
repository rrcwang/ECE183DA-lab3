import numpy as np
import math
import pygame
import random
from itertools import chain
import networkx as nx

# import dictionary for graph
from collections import defaultdict

BOX_SIZE = [63, 45]
ROBOT_RADIUS = 8

##############
# Defines the rectangular obstacles in our map


class Obstacle:
    # Initializer,
    # takes the corners as input
    def __init__(self, corners):
        self.corners = sort_clockwise(corners)
        self.center = np.mean(corners, axis=0)

    # Returns the coordinates of either the middle point or the corners
    def get_coords(self, n):
        if (n == 0):
            return self.center
        elif (n > 4) or (n < 0):
            print("Invalid point requested.")
            return [np.nan, np.nan]
        else:

            return self.corners[n - 1]

    def get_corners(self):
        return self.corners

    # Does it collide with some other box?
    def collides_with(self, box):
        normals_1 = get_box_normals(self)
        normals_2 = get_box_normals(box)

        result_1 = [get_box_min_max(bx, norm) for norm in [
            normals_1[1], normals_1[0]] for bx in [self, box]]

        result_2 = [get_box_min_max(bx, norm) for norm in [
            normals_2[1], normals_2[0]] for bx in [self, box]]

        b1 = result_1[0][1] < result_1[1][0] or result_1[1][1] < result_1[0][0]
        b2 = result_1[2][1] < result_1[3][0] or result_1[3][1] < result_1[2][0]

        b3 = result_2[0][1] < result_2[1][0] or result_2[1][1] < result_2[0][0]
        b4 = result_2[2][1] < result_2[3][0] or result_2[3][1] < result_2[2][0]


        return not (b1 or b2 or b3 or b4)


# rounds each coordinate point to the nearest tenth in each dimension to reduce complexity
def round_coords(point):
    return [round(point[0], 1), round(point[1], 1)]

class PathGraph:
    ''' Graph containing vertex points of the RRT '''
    def __init__(self, init_position):
        self.graph = nx.Graph()
        self.graph.add_node(tuple(init_position))

    def add_edge(self, u, v):
        u = tuple(round_coords(u))
        v = tuple(round_coords(v))
        self.graph.add_edge(u, v)

    # definition of function that 
    def generate_edges(self):
        edges = []
        # for each node in graph
        for node in self.graph:
            # for each neighbour node of a single node
            for neighbour in self.graph[node]:
                # if edge exists then append
                edges.append((node, neighbour))
        return edges

    def get_all_points(self):
        return list(self.graph.nodes())

    # function to find the shortest path
    # function to find path
    def find_path(self, start, end):
        return nx.shortest_path(self.graph, start, end)

    ### 2.2(e) ###
    # returns the point in points that is nearest the target
    def get_nearest(self, target):
        target = round_coords(target)
        nearest_index = 0
        points = self.get_all_points()

        nearest_dist = np.linalg.norm(
            [points[0][0] - target[0], points[0][1] - target[1]])
        points = np.array(points)

        for i in range(1, np.shape(points)[0]):
            curr_norm = np.linalg.norm(points[i] - target)
            if (np.linalg.norm(points[i] - target) < nearest_dist):
                nearest_index = i
                nearest_dist = curr_norm

        return points[nearest_index]


class ConfigSpace:
    def __init__(self, obstacles):
        self.obstacles = obstacles  # list of obstacles

    def get_obstacle(self, n):
        if (n < 0) or (n > np.shape(self.obstacles)[0]):
            print("Invalid obstacle requested")
        else:
            return self.obstacles[n]

    def gen_path_2pts(self, initial_state, target_state):
        """ Generates the direction and velocity required to travel from the initial_state to the target_state """

        diff = initial_state - target_state
        # velocity is given by the distance/time, time = 1s
        vel = np.linalg.norm(diff[0:2])
        direc = diff[0:2] / vel
        # angle that goes from initial_state to target_state is given by
        heading = math.atan2(diff[1], diff[0])
        ### 2(c) ###
        # To move from initial_state to target_state, we will need to rotate to the heading
        # given above, then travel in that direction at velocity vel to reach the target
        return [direc, vel]

    def point_collides_with(self, position, obstacle):
        """ Detects whether the robot collides with some given obstacle statically
        Implementation taken from:
        https://gamedevelopment.tutsplus.com/tutorials/collision-detection-using-the-separating-axis-theorem--gamedev-169 """

        # get the normalized vector pointing from the obstacle's center to the robot's
        obs_center = obstacle.get_coords(0)
        obs_to_robot = position - obs_center

        obs_to_robot_norm = obs_to_robot
        norm = np.linalg.norm(obs_to_robot)
        if (norm != 0):
            obs_to_robot_norm = obs_to_robot / norm

        # check which corner of the obstacle is nearest to the robot,
        # the max_projection value should always end up positive
        max_projection = -1
        for n in range(1, 5):
            corner = obstacle.get_coords(n)
            center_to_corner = corner - obs_center

            projection = np.dot(center_to_corner, obs_to_robot_norm)

            if (projection > max_projection):
                max_projection = projection

        # Checks if there exists some separating line between the robot and the obstacle
        if ((np.linalg.norm(obs_to_robot) - max_projection - ROBOT_RADIUS) > 0) and (np.linalg.norm(obs_to_robot) > 0):
            return False
        else:
            return True

    def point_collides(self, position):
        collides = False
        for obs in self.obstacles:
            collides = collides or self.point_collides_with(position, obs)
        return collides

    ### 2.2(d) ###
    def check_path_collision(self, initial_state, target_state):
        ''' returns true if the path collides with any of the obstacles '''
        path = self.gen_path_2pts(initial_state, target_state)
        heading = math.atan2(path[0][1], path[0][0])
        side = [math.cos(heading + math.pi), math.sin(heading + math.pi)]

        initial_state = np.array(initial_state)

        path_rect = Obstacle([initial_state + side,
                              initial_state - side,
                              target_state + side,
                              target_state - side])

        collides = False
        for obs in self.obstacles:
            collides = collides and path_rect.collides_with(obs)

        return collides

    def get_random_pt(self):
        return [random.uniform(0, 1)*BOX_SIZE[0], random.uniform(0, 1)*BOX_SIZE[1]]

### 2.2(f) ###


def RRT_create(init_pos, target, config_space):
    ''' CREATE RRT '''
    path_graph = PathGraph(init_pos)
    count = 0
    max_iters = 1000000

    target = np.array(target)

    while (count < max_iters):
        new_pt = np.array(config_space.get_random_pt())
        if count % 10:
            new_pt = target
        nearest_pt = path_graph.get_nearest(new_pt)

        # make step in direction
        direc = (new_pt - nearest_pt)
        direc = direc/np.linalg.norm(direc) * 3
        new_pt = nearest_pt + direc
        if (new_pt[0] < 0) or (new_pt[0] > BOX_SIZE[0]) or (new_pt[1] < 0) or (new_pt[1] > BOX_SIZE[1]):
            new_pt = new_pt - direc*0.9

        if ((config_space.check_path_collision(new_pt, nearest_pt))
            or (config_space.point_collides(new_pt))
                or (config_space.point_collides(nearest_pt))):
            continue
        path_graph.add_edge(new_pt, nearest_pt)

        if (np.linalg.norm(new_pt - target) < 3):
            path_graph.add_edge(new_pt, target)
            return path_graph

    return path_graph


def sort_clockwise(corners):
    ''' Sorts distinct points of a polygon in clockwise order '''
    corners = np.array(corners)
    center = np.mean(corners, axis=0)
    axis = np.array([1, 0])

    angles = [math.atan2(corners[i, 1] - center[1],
                         corners[i, 0] - center[0]) for i in range(np.shape(corners)[0])]
    order = np.argsort(angles)

    return corners[order, :]


def get_box_min_max(box, axis):
    ''' Returns the maximum and minimum magnitudes of the box-to-corner vectors
    projected onto the box-to-box direction '''
    axis = axis / np.linalg.norm(axis)

    corners = box.get_corners()

    min_projection = corners[1, :] @ axis
    max_projection = corners[1, :] @ axis

    for j in range(1, 4):
        curr_proj = corners[j, :] @ axis
        if (min_projection > curr_proj):
            min_projection = curr_proj

        if (max_projection < curr_proj):
            max_projection = curr_proj

    return [min_projection, max_projection]

def get_box_normals(box):
    ''' Returns the normaal vectors to the sides of a box obstacle '''
    corners = box.get_corners()

    normals = []
    for i in range(0, 4):
        curr_norm = np.array(corners[i, :] - corners[i-1, :])
        curr_norm = [-curr_norm[1], curr_norm[0]]
        curr_norm = curr_norm / np.linalg.norm(curr_norm)
        normals.append(curr_norm)
    return (normals[1:] + normals[:1])



### RUN CODE ###
o = Obstacle([[0, 10],
              [0, 13],
              [25, 13],
              [25, 10]])

cs = ConfigSpace([o])

gg = RRT_create([45, 10], [8, 35], cs)
ggs = gg.generate_edges()
path = gg.find_path((45, 10), (8, 35))

# format data and run
passs = [[element for tupl in tupleOfTuples for element in tupl]
       for tupleOfTuples in ggs]
np.savetxt("data.csv", passs, delimiter=",")      # output tree data
puss = [element for tupl in path for element in tupl]
np.savetxt("path.csv", puss, delimiter=",")     # output optimal path data
