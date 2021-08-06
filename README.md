# RoverSim
Open Projects 2021

## Abstract

The project aims to introduce the Path-Planning phase to a wireless rover to make it capable to navigate autonomously in an environment with the help of path planning algorithms (Dijkstra and SLAM) parsed by a python script. It aims to reach the destination through the shortest path and shortest time. This was done using ROS navigation stack by performing a simulation of a rover model in Gazebo environment, by running ROS (Melodic Morenia) in Ubuntu 18.04. The robot used in this project is Turtlebot 3.

![gazebo_sim](https://github.com/brahatesh/RoverSim-1/blob/main/Images%20and%20Videos/Gifs/gazebo_sim.gif "Simulation in Gazebo")

![rviz_sim](https://github.com/brahatesh/RoverSim-1/blob/main/Images%20and%20Videos/Gifs/rviz_sim.gif "Simulation in RViz")


## Motivation

<p>Watching a robot move on its own without any human input is always fascinating. So, our team of 5 freshers decided to take on the challenge to learn how a robot maps the environment, find its position in the map of the environment and move from one position to another on its own.<br><br>The future will be fully automated with the help of robots and for that automated navigation will be extremely important. So making a small robot move automatically is the first step.</p>

## Applications

* Autonomous Vehicles
* Navigating a fleet of mobile robots to arrange shelves in a warehouse automated cleaner at home
* Finding Shortest path for a robot to serve in restaurant/other services
* **Space**: Terrain mapping on other planets
* Exploration of mines

## Limitations

* While navigating to its destination, the robot tends to go too close to an obstacle and get stuck in a loop that makes it rotate constantly.
**EKF Slam has some limitations as listed below:**
* For larger inputs, the application crashes because the time complexity of the algorithm is O(n^2)
* Featured(point) landmarks can only be detected
* All densities involved in the algorithm are modelled as Gaussian. Hence the model is assumed to have a normal distribution which introduces some noise.
* The environment has to be a static environment.

## Future Improvements
* **Implementation of fast slam in ROS**
* **Adding ArUco marker detection capability for pose estimation**


## Team members

1. [Aman Vishwakarma](https://github.com/aman-vishwakarma2024)
2. [Ankit Dudi](https://github.com/fanaticANKIT)
3. [Brahatesh Vasantha](https://github.com/brahatesh)
4. [Harshini Sendhil](https://github.com/cicadaa3301-harshini)
5. [Prashant Gupta](https://github.com/chill3010house)

## Mentors

1. [Pradnesh Pramod Chavan](https://github.com/theobscuredev)
2. Shubham Goyal