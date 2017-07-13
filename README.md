# Trajectory planner

![trajectory_planner example][6]
This planner is able to calculate movement trajectory on [ros::OccupancyGrid][1] for robot, using set of allowed moves for it.
Given map, start and goal positions and possible movements, planner will try to find the way from start to goal positions, using one of available algorithms (A* and BFS implemented while now).
 
## Features
### 2D planning
Planning works on 2D occupancy grid, which you can get from Rtabmap, hector_mapping or gmapping SLAM algorithms.
OccupancyGrid is just 2D array, where each cell represent small area of real world and can be in one of three states: occupied, free, unknown.
Using ros::OccupancyGrid is a good way for small wheel platforms or walking robots.


### rospy based
This package based on [ROS][2].  
Due to Python 2 as main language you don't need to compile package.  
Just move it into *catkin workspace* and run planning on predefined map: `roslaunch trajectory_planner_py static_planning.launch`

## Description
### How planning works
Basic idea of planner - search in state space, where State is a vector which represents position and orientation of a robot.
Robot is a rectangular primitive with specified width and height.
Robot also has some movements, which it can do.
Each movement is a vector (length, dtheta), which can represent movements such as rotation and moving forward/backward.
When we apply movement to one of available states (for example start state), we can get new state with different position and rotation.

![apply_movement][7]

Of course, on the way of this movement must not be any obstacles. 
This is how we can get new state. It can be done by verifying substates when "simulating" yet another move.
![simulate_move][8]

So to start planning we should have map, start and goal positions, robot parameters and set of available moves for it.
Then we can get huge tree of states by applying moves to start state and then to new states and etc.

Now we need algorithm to effectively find state, which is the nearest to goal state.
Current implementation of A* is not brilliant, but seems to be easy to understand.

I got an idea of how to implement A* using [this nice publication][9].

Architecture of this package is quite easy, so don't be scared to change something in source code ;)
 
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
This package is in development now.
So be careful when using it.
Some important things, like time limitation of path finding were not implemented yet.
 

[1]: http://docs.ros.org/jade/api/nav_msgs/html/msg/OccupancyGrid.html
[2]: http://www.ros.org/
[3]: http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html
[4]: http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html
[5]: http://docs.ros.org/jade/api/visualization_msgs/html/msg/MarkerArray.html
[6]: https://habrastorage.org/web/9af/c3c/da1/9afc3cda194044f2881fb1b942ddead8.png
[7]: https://habrastorage.org/web/8fc/e94/8c1/8fce948c125849f4b2ba01d7dcbbf376.png
[8]: https://habrastorage.org/web/9fd/79a/7ce/9fd79a7cee8e4305bf9f6291e5658be6.png
[9]: http://web.mit.edu/eranki/www/tutorials/search/