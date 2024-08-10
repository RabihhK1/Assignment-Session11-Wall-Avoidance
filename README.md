# Assignment-Session11-Wall-Avoidance

## Introduction -VFF Avoidance - TurtleBot3

By: Makram Ghraizi - Rabih Kiwan - Firas Dimashki

This project implements an obstacle avoidance behavior for a TurtleBot3 robot using the Virtual Force Field (VFF) algorithm. The VFF algorithm is a simple and effective method that enables the robot to navigate through an environment while avoiding obstacles.

The VFF algorithm operates by calculating three key vectors:

1. **Attractive Vector**: This vector points in the forward direction, representing the robot's intended path in the absence of obstacles.
2. **Repulsive Vector**: Derived from laser sensor data, this vector pushes the robot away from nearby obstacles, with the force inversely proportional to the distance to the obstacle.
3. **Resultant Vector**: The sum of the Attractive and Repulsive vectors, determining the robot's final movement direction and speed.

This package is structured according to ROS 2 best practices, ensuring modularity and ease of debugging, with visualization tools integrated for monitoring the robot's behavior.

## Control Logic

The AvoidanceNode is responsible for implementing the VFF algorithm and generating appropriate movement commands based on the robot's surroundings. The control logic includes the following steps:

The algorithm is executed at a frequency of 20Hz using a ROS timer.
The node verifies that the laser scan data is valid and up-to-date.
The Attractive, Repulsive, and Resultant vectors are calculated based on the laser scan data.
The cmd_vel message is published to control the robot's movement.

To facilitate debugging and visualization, the AvoidanceNode publishes visual markers that can be viewed in RViz2.

## Running on Gazebo Turtlebot

On one Terminal:
```sh
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py 
```
On another terminal:

In the Ros2 ws:
```sh
colcon build
source install/setup.bash
ros2 launch obstacle_avoidance launch.py
```
