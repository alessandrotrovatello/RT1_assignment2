# 3D Robot Simulation

Second assignment of the Research Track 1 course of the Master's degree in Robotics Engineering at the University of Genoa.

## Assignment

The main task of this assignment is to create a ROS package which must include the following three nodes:
- A node that implements an action client, allowing the user to set a target (x, y) or to cancel it. Trying to use the feedback/status of the action server to know when the target has been reached. The node also publishes the robot position and velocity as a custom message (x,y, vel_x, vel_z), by relying on the values published on the topic /odom;
- A service node that, when called, returns the coordinates of the last target sent by the user;
- Another service node that subscribes to the robot’s position and velocity (using the custom message) and implements a server to retrieve the distance of the robot from the target and the robot’s average speed.
  
Another task is to create a launch file to start the entire simulation, in which a parameter must be used to select the size of the averaging window.

The starting point of the assignment is reachable in the [starting_point](https://github.com/alessandrotrovatello/RT1_assignment2/tree/starting_point) branch.

## How to use

The assignment is developed in [Ubuntu 20.04 LTS](https://ubuntu.com/tutorials/install-ubuntu-desktop#2-download-an-ubuntu-image) using [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu).

To use the code create a new directory:
```bash
mkdir directory_name
```
And move in it:
```bash
cd directory_name
```
Now you can clone this repository by using [GIT](https://github.com/git-guides/install-git):
```bash
git clone https://github.com/alessandrotrovatello/RT1_assignment2
```


