# PATH PLANNING

Path planning is a problem in robotics to find a sequence of valid configurations of movements that moves any object (eg. a robot) from its source to its destination. To navigate successfully, a robot has to avoid all obstacles and get to its destination safely. This is done by using various planning algoithms. In this project, the algorithm used is Dijkstra's Algorithm.

## RUNNING THE SIMULATION

**This simulation has only been tested for ROS Melodic in Ubuntu 18.04. It might not run on other versions.**

Make sure that all [dependencies](#dependencies) are fulfilled before running the simulation.

First use catkin_make to generate the required files

    cd [DIRECTORY_WHERE_REPO_IS_CLONED]/src/Dijkstra_Path_Planning/
    catkin_make

Now setup the workspace with setup bash script

    source devel/setup.bash

Run the launch xml file

    roslaunch sim simulation.launch

## THE CODE EXPLAINED

### THE ALGORITHM - (![File](https://github.com/brahatesh/RoverSim-1/blob/main/src/Dijkstra_Path_Planning/src/dijkstra_pp/scripts/path_planning_dijkstra.py))

Dijkstra's algorithm is an algorithm that finds the path between 2 nodes with the minimum cost. Cost in this case can be treated as the distance between 2 nodes. 

![Flowchart](https://raw.githubusercontent.com/brahatesh/RoverSim-1/main/Images%20and%20Videos/Images/djikstra_flowchart.jpg)


Some terminologies used:
* **current node:** Node which is currently being processed by the algorithm
* **neighbours:** Nodes next to the current node (i.e., right, left, up, down)
* **open_list:** List containing nodes discovered as neighbours that haven't been processed
* **closed_list:** List containing nodes with discovered neighbours
* **g_cost:** Cost of moving from start to a particular node
* **parent node:** Lowest costing node that discovers the given node

1. Make the g_cost of starting node as 0 and add it to open_list.
2. Arrange open_list according to g_cost of nodes and pop the lowest costing node. Call this 
3. 

## DEPENDENCIES