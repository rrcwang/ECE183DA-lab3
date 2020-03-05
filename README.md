# ECE183DA-lab3
Lab 3 - Trajectory Planning
Timothy L., Richard W., Christian Y.

# Files
* /RRT/RRT_run.py
Generates the RRT for some arbitrary configuration space.
It can be queried by setting up a list of rectangular Obstacle objects, each initialized with it's four corner points as the argument. We then pass these Obstacles to the ConfigSpace initializer, which provides functions to extract data from it. Finally, we call RRT_create(init, final, config_space), outputting the graph data structure from which we can find the optimal path.
* /State Detection/camerastate.py
Streams data from camera in order to provide a state estimate of the robot in real time.