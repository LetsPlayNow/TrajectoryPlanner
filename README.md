# Trajectory planner

![trajectory_planner example][6]
This planner can calculate movement trajectory from start to goal positions on a [ros::OccupancyGrid][1] using A* or BFS algorithms for a robot primitive that can perform a set of simple movements.

## Features
### 2D planning
Planning works on a 2D occupancy grid (ros::OccupancyGrid), which is just a 2D array, where each cell represents a small area and can be in one of three states: occupied, free, unknown. It is a good map data structure for small wheel platforms and simple walking robots. You can get it from Rtabmap, hector_mapping or gmapping SLAM algorithms.

### rospy based
This package is based on [ROS][2] and built using Python 2, so you don't need to compile it. To run the package, move it into *catkin workspace* and run planning on predefined map: `roslaunch trajectory_planner_py static_planning.launch`

## Description
### How planner works
The planner is searching in the state space, where State is a vector of position and orientation of a robot. The Robot is a rectangular primitive with width and height parameters and a set of simple moves, which are described as vectors (length, dtheta) and represent rotation and moving forward/backward. New states are derived from the previous ones by applying move transformations.

![apply_movement][7]

We also check intermediate sub states on collisions with obstacles by simulating moves with a small step.
![simulate_move][8]

To start planning we need a map, start and goal positions, robot parameters and a set of available moves. Then we can get a tree of states by applying moves to the start state and repeating this for its "children".
We use A* or BFS to search for a goal state in that tree.
I got intuition for A* implementation from [this nice publication][9].
 
## ROS API

### Subscribed topics
* initialpose ([geometry_msgs/PoseWithCovarianceStamped][3])  
Start pose for planner. You can set *initialpose* from RVIZ
* goal ([geometry_msgs/PoseStamped][4])  
Goal pose for planner

### Published topics
* trajectory ([visualization_msgs/MarkerArray][5])  
Array of robot's poses 

## Disclaimer
This package is in development now and may contain bugs.

[1]: http://docs.ros.org/jade/api/nav_msgs/html/msg/OccupancyGrid.html
[2]: http://www.ros.org/
[3]: http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html
[4]: http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html
[5]: http://docs.ros.org/jade/api/visualization_msgs/html/msg/MarkerArray.html
[6]: https://habrastorage.org/web/9af/c3c/da1/9afc3cda194044f2881fb1b942ddead8.png
[7]: https://habrastorage.org/web/8fc/e94/8c1/8fce948c125849f4b2ba01d7dcbbf376.png
[8]: https://habrastorage.org/web/9fd/79a/7ce/9fd79a7cee8e4305bf9f6291e5658be6.png
[9]: http://web.mit.edu/eranki/www/tutorials/search/
