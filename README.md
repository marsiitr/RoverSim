# RoverSim
## Open Projects 2021

<p align="center">

![gazebo_sim](https://github.com/brahatesh/RoverSim/blob/main/Images%20and%20Videos/Gifs/gazebo_sim.gif "Simulation in Gazebo")
<i>Simulation in Gazebo</i></p>

<p align="center">

![rviz_sim](https://github.com/brahatesh/RoverSim/blob/main/Images%20and%20Videos/Gifs/rviz_sim.gif "Simulation in RViz")
<i>Simulation in RViz</i></p>


## Abstract

<p align="justify">The project aims to introduce the Path-Planning phase to a wireless rover to make it capable to navigate autonomously in an environment with the help of path planning algorithms (Dijkstra and SLAM) parsed by a python script. It aims to reach the destination through the shortest path and shortest time. This was done using ROS navigation stack by performing a simulation of a rover model in Gazebo environment, by running ROS (Melodic Morenia) in Ubuntu 18.04. The robot used in this project is Turtlebot 3.</p>


## Motivation

<p align="justify">Watching a robot move on its own without any human input is always fascinating. So, our team of 5 freshers decided to take on the challenge to learn how a robot maps the environment, find its position in the map of the environment and move from one position to another on its own.<br><br>The future will be fully automated with the help of robots and for that automated navigation will be extremely important. So making a small robot move automatically is the first step.</p>

## Workflow
<p align="center">

![Workflow](https://github.com/brahatesh/RoverSim/blob/main/Images%20and%20Videos/Images/Workflow.jpg)</p>

## Software aspects

### Navigation

<p align="justify">Path planning is a problem in robotics to find a sequence of valid configurations of movements that moves any object (eg. a robot) from its source to its destination. To navigate successfully, a robot has to avoid all obstacles and get to its destination safely. This is done by using various planning algoithms. In this project, the algorithm used is Dijkstra's Algorithm.

[Elaborative explanation](https://github.com/brahatesh/RoverSim/blob/main/src/Dijkstra_Path_Planning/README.md)</p>

### SLAM

<p align="justify">When the bot is in an unknown environment, the bot does not have any knowledge about where it is and how it is oriented, this means the bot has no access to the map of the environment, nor does it have access to its own poses, Instead, all it is given are measurements and controls this problem is called as SLAM problem. There are different algorithms formulated to solve this problem. The algorithms are broadly classified as online and full SLAM. In Online SLAM they only involves the estimation of variables that persist at time t, they discard past measurements and controls once they have been processed. In full SLAM, we seek to calculate a posterior over the entire path along with the map, instead of just the current pose.

[Elaborative explanation](https://github.com/brahatesh/RoverSim/blob/main/src/EKF%20SLAM/README.md)</p>

## Applications

* Autonomous Vehicles
* Navigating a fleet of mobile robots to arrange shelves in a warehouse automated cleaner at home
* Finding Shortest path for a robot to serve in restaurant/other services
* **Space**: Terrain mapping on other planets
* Exploration of mines

## Limitations

* While navigating to its destination, the robot tends to go too close to an obstacle and get stuck in a loop that makes it rotate constantly.<br><br>
**EKF Slam has some limitations as listed below:**
* For larger inputs, the application crashes because the time complexity of the algorithm is O(n^2)
* Featured(point) landmarks can only be detected
* All densities involved in the algorithm are modelled as Gaussian. Hence the model is assumed to have a normal distribution which introduces some noise.
* The environment has to be a static environment.

## Cost Structure

|*Components*                    |   *Cost(INR.)*   |
|--------------------------------|------------------|
| ROS                            | Open-Source/None |
| Turtlebot 3 model              | Open-Source/None |
| ROS Navigation Stack           | Open-Source/None |
| Ubuntu 18.04                   | Open-Source/None |
| Python                         | Open-Source/None |
| Numpy                          | Open-Source/None |
| Matplotlib                     | Open-Source/None |


## Future Improvements
* Implementation of fast slam in ROS
* Adding ArUco marker detection capability for pose estimation

## Project Presentation

<https://docs.google.com/presentation/d/1_wiWe1_HqG1ynKELk3Ks25U8nY3r4H1Q/edit?usp=sharing&ouid=118357681341214806113&rtpof=true&sd=true/>

## Team members

1. [Aman Vishwakarma](https://github.com/aman-vishwakarma2024)
2. [Ankit Dudi](https://github.com/fanaticANKIT)
3. [Brahatesh Vasantha](https://github.com/brahatesh)
4. [Harshini Sendhil](https://github.com/cicadaa3301-harshini)
5. [Prashant Gupta](https://github.com/chill3010house)

## Mentors

1. [Pradnesh Pramod Chavan](https://github.com/theobscuredev)
2. [Shubham Goyal](https://github.com/shubham491981)

## References

* [ROS wiki](http://wiki.ros.org/)

* [Path planning](https://www.youtube.com/watch?v=ZmQIkBws4LA)

* [EKF SLAM](https://www.youtube.com/watch?v=X30sEgIws0g&t=1632s&ab_channel=CyrillStachniss)

* [SLAM basics](https://www.youtube.com/watch?v=B2qzYCeT9oQ&list=PLpUPoM7Rgzi_7YWn14Va2FODh7LzADBSm&ab_channel=ClausBrenner)

* [Chapter 10 in Probabilistic Robotics by Dieter Fox, Sebastian Thrun, and Wolfram Burgard](https://books.google.co.in/books/about/Probabilistic_Robotics.html?id=2Zn6AQAAQBAJ&source=kp_book_description&redir_esc=y)

* [SLAM for dummies](https://dspace.mit.edu/bitstream/handle/1721.1/119149/16-412j-spring-2005/contents/projects/1aslam_blas_repo.pdf)
