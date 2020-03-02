import numpy as np

##############
# Defines the rectangular obstacles in our map

class Obstacle:
    # Initializer,
    # takes the corners as input
    def __init__(self, corners):
        self.corners = corners
        self.center = np.mean(corners)

    # Returns the coordinates of either the middle point or the corners
    def get_point(self, n):
        if (n == 0):
            return self.center
        elif (n > 4):
            print("Invalid point requested.")
            return -1
        else:
            return self.corners[n, :]

##############
# Defines the robot's current state

class Robot:
    # Initializer,
    # takes the state (x, y, theta) as input
    def __init__(self, state):
        self.state = state
        self.radius = 1

    def update_state(self, new_state):
        self.state = new_state

    def get_state(self):
        return self.state
    
    # Detects whether the robot collides with some given obstacle
    # Implementation taken from:
    # https://gamedevelopment.tutsplus.com/tutorials/collision-detection-using-the-separating-axis-theorem--gamedev-169
    def collides_with(self, obstacle):
        # get the normalized vector pointing from the obstacle's center to the robot's
        obs_center = obstacle.get_point(0)
        obs_to_robot = self.state - obs_center
        
        obs_to_robot_norm = obs_to_robot
        norm = np.linalg.norm(obs_to_robot)
        if (norm != 0):
            obs_to_robot_norm = obs_to_robot / norm
        
        # check which corner of the obstacle is nearest to the robot,
        # the max_projection value should always end up positive
        max_projection = -1
        for n in range(1,5):
            corner = obstacle.get_point(n)
            center_to_corner = corner - obs_center
            
            projection = np.dot(center_to_corner, obs_to_robot_norm)
            
            if (projection > max_projection):
                max_projection = projection
        
        # Checks if there exists some separating line between the robot and the obstacle
        if ((np.linalg.norm(obs_to_robot) - max_projection - self.radius) > 0) and (np.linalg.norm(obs_to_robot) > 0):
            return False
        else:
            return True

# A vertex in the path tree
class Path_Vertex:
    def __init__(self, position):
        self.x = position[0]
        self.y = position[1]
    
    
        
    

r = Robot([0,0,0])
o = Obstacle([[0,0],
              [0,1],
              [1,0],
              [1,1]])
