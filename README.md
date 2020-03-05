# ECE183DA-lab3
Lab 3 - Trajectory Planning
Timothy L., Richard W., Christian Y.

# Files
* /RRT/RRT_run.py
Generates the RRT for some arbitrary configuration space.
It can be queried by setting up a list of rectangular Obstacle objects, each initialized with it's four corner points as the argument. We then pass these Obstacles to the ConfigSpace initializer, which provides functions to extract data from it. Finally, we call RRT_create(init, final, config_space), outputting the graph data structure from which we can find the optimal path.
* /State Detection/camerastate.py
Streams data from camera in order to provide a state estimate of the robot in real time.
Visualizes the configuration space, RRT tree, and fastest path.
* Video of experiment in action: https://drive.google.com/file/d/1-1QiaZM3S5SvNVL1i6qHEvCh5tFvu6dk/view?usp=sharing

# Resources consulted
* https://gamedevelopment.tutsplus.com/tutorials/collision-detection-using-the-separating-axis-theorem--gamedev-169
* https://medium.com/@theclassytim/robotic-path-planning-rrt-and-rrt-212319121378

# Non-default libraries used
* opencv
* networkx
* numpy
