# PATH PLANNING

<p align="justify">Path planning is a problem in robotics to find a sequence of valid configurations of movements that moves any object (eg. a robot) from its source to its destination. To navigate successfully, a robot has to avoid all obstacles and get to its destination safely. This is done by using various planning algoithms. In this project, the algorithm used is Dijkstra's Algorithm.</p>

## RUNNING THE SIMULATION

**This simulation has only been tested for ROS Melodic in Ubuntu 18.04. It might not run on other versions.**

<p align="justify">Make sure that all [dependencies](#dependencies) are fulfilled before running the simulation.</p>

First use catkin_make to generate the required files

    cd [DIRECTORY_WHERE_REPO_IS_CLONED]/src/Dijkstra_Path_Planning/
    catkin_make

Now setup the workspace with setup bash script

    source devel/setup.bash

Run the launch xml file

    roslaunch sim simulation.launch

## THE CODE EXPLAINED

### THE ALGORITHM - ([File](https://github.com/brahatesh/RoverSim/blob/main/src/Dijkstra_Path_Planning/src/dijkstra_pp/scripts/path_planning_dijkstra.py))

<p align="justify">Dijkstra's algorithm is an algorithm that finds the path between 2 nodes with the minimum cost. Cost in this case can be treated as the distance between 2 nodes.</p> 

<p align="center">

![Flowchart](https://raw.githubusercontent.com/brahatesh/RoverSim/main/Images%20and%20Videos/Images/djikstra_flowchart.jpg)</p>


<p align="justify">

Some terminologies used:
* **current node:** Node which is currently being processed by the algorithm
* **neighbours:** Nodes next to the current node (i.e., right, left, up, down)
* **open_list:** List containing nodes discovered as neighbours that haven't been processed
* **closed_list:** List containing nodes with discovered neighbours
* **g_cost:** Cost of moving from start to a particular node
* **parent node:** Lowest costing node that discovers the given node</p>

<p align="justify">

1. Make the g_cost of starting node as 0 and add it to open_list.
2. Arrange open_list according to g_cost of nodes and pop the lowest costing node. Call this current node. 
3. Add current node to closed_list.
4. If current node is goal node, go to step 8.
5. Get neighbours of current node.
6. Analyze each neighbour of current node as follows:
    * If neighbour is in closed_list, skip and continue to the next neighbour.
    * If neighbour is in open_list and g_cost of current node is less than g_cost of parent of neighbour, set current node as parent node and update g_cost of neighbour.
    * If neighbour is not in open_list or closed_list, add neighbour to open_list with current node as parent.
7. Go to step 2 if open_list is not empty.
8. Build a path list from end to start by tracing back from target node to start node using each node's parent node. Then reverse this path list to get the path.</p>

### THE PLUGIN - ([File](https://github.com/brahatesh/RoverSim/blob/main/src/Dijkstra_Path_Planning/src/pp_plugin/src/pp_plugin.cpp))

<p align="justify">This plugin makes the call to the path planning server and publishes the path in the format of nav_msgs::Path in the plan topic. It also gets the 2D costmap and converts it to 1D array so that it can be passed on to the algorithm. Other things done by the plugin include converting regular co-ordinates to an equivalent index on the 1D costmap.</p>

## DEPENDENCIES

This simulation depends on the following ROS melodic packages:
* rospy
* message_runtime
* roscpp
* std_msgs
* base_local_planner
* costmap_2d
* geometry_msgs
* nav_core
* nav_msgs
* pluginlib
* tf2
* tf2_geometry_msgs
* tf2_ros
* amcl
* gazebo
* move_base
* rviz
* map_server
* robot_state_publisher