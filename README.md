# 3D Robot Simulation

Second assignment of the Research Track 1 course of the Master's degree in Robotics Engineering at the University of Genoa.
<p align="center">
  <img src="https://github.com/alessandrotrovatello/RT1_assignment2/blob/main/images/grid.png" alt="Environment">
</p>

## Table of contents
- [Assignment description](#assignmentdescription)
- [ROS Architecture](#rosarchitecture)
- [How to download the repository](#rosrep)
  - [Setup your ROS workspace](#rosws)
- [How to run the code](#howto)

<a name="assignmentdescription"></a>
## Assignment description

The main task of this assignment is to create a ROS package which must include the following three nodes:
- A node that implements an action client, allowing the user to set a target (x, y) or to cancel it. Trying to use the feedback/status of the action server to know when the target has been reached. The node also publishes the robot position and velocity as a custom message (x,y, vel_x, vel_z), by relying on the values published on the topic /odom;
- A service node that, when called, returns the coordinates of the last target sent by the user;
- Another service node that subscribes to the robot’s position and velocity (using the custom message) and implements a server to retrieve the distance of the robot from the target and the robot’s average speed.
  
Another task is to create a launch file to start the entire simulation, in which a parameter must be used to select the size of the averaging window.

The starting point of the assignment is reachable in the [starting_point](https://github.com/alessandrotrovatello/RT1_assignment2/tree/starting_point) branch. Nodes already implemented are *go_to_point_service.py*, *wall_follow_service.py* and *bug_as.py* that manages the robot's algorithm in reaching the goal, such as system messages and obstacle avoidance management.

<a name="rosarchitecture"></a>
## ROS Architecture

The ROS architecture consists of a few nodes that communicate with each other via msgs published in topics. The library that allows us to set and send the target (or goal) is [**actionlib**](https://wiki.ros.org/actionlib), this ROS library is used to handling asynchronous tasks. It facilitates communication between a client and a server node, supporting the execution of long-duration tasks with asynchronous feedback. Key features include asynchronous communication, feedback during task execution, result reporting, goal specification, and support for retries in case of failure. It enhances the robustness and efficiency of managing complex actions in robotic systems.

To give a better idea of how the architecture is composed, this below is a graph of the ROS architecture:

![ROS Architecture of the assignment](https://github.com/alessandrotrovatello/RT1_assignment2/blob/main/images/rosgraph.png)

Where we can see how the nodes, the msg and srv communicate with each other, in a better and clearly way.

The nodes developed are an action client and two service node to get information about last target coordinates and the distance between the robot and target and the average speed of the robot:
- **action_client** is the node that allows us to get the goal coordinates from the user to be sent to the server throught the `/reaching_goal` topic; the node allows us to cancel the goal while the robot is reaching the goal. Additionally, the node publish the information about robot position and velocity in a `/robot_pos_vel` topic as a custom message. The following flowchart explains how the action client is structured:

<p align="center">
  <img src="https://github.com/alessandrotrovatello/RT1_assignment2/blob/main/images/action_client_flowchart.png" alt="*action_client*'s flowchart">
</p>

There is a little control on the user input to get only coordinates in range to [-9,9] due to the size of the environment (10x10 grid), furthermore there is a goal threshold to prevent the robot from not reaching the desired position in case that position is occupied by an obstacle.

- **last_target_service** is a service node that allows us to get the last goal coordinates from the `/reaching_goal/goal` topic. This info is can be retrieved calling the service as below:
```bash
rosservice call /last_target
```

- **avg_service** is a service node that allows us to get the goal coordinates from the ROS param, defined in the assignment1.launch, by using *rospy.get_param("param_name")*, the robot position and the robot velocity are get from the `/robot_pos_vel` topic, created on **action_client** node. This service calculate the distance between robot and target using their coordinates in the Euclidean formula $c=\sqrt{a^2+b^2}$, where 'c' is the distance between robot and target (hypotenuse) while 'a' and 'b' are the x and y components difference (cathetuses). In addition, the service calculate the average velocity along robot x-axis and the average around robot z-axis, using a parameter to set the size of the averaging window, this value as default is 10, then the avg is obtained by the arithmetic mean $m=\frac{a_1 + a_2 + \ldots + a_n}{n}$ where 'n' is the window size param. The service can be called writing:
```bash
rosservice call /avg_dist_vel
```
<a name="rosrep"></a>
## How to download the repository

The assignment is developed in [Ubuntu 20.04 LTS](https://ubuntu.com/tutorials/install-ubuntu-desktop#2-download-an-ubuntu-image) using [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu), while the simulation environmnet used is [Gazebo](https://gazebosim.org/docs/harmonic/architecture) (not to be downloaded).

The code must be cloned within your ROS workspace, if you already have a ROS workspace skip the next steps until the [git clone](#clone) command.

<a name="rosws"></a>
### How to setup your ROS workspace
To setup your own ROS workspace, the first step is to create a new directory:
```bash
mkdir directory_name
```
And move in it:
```bash
cd directory_name
```
Now you are inside the *directory_name* folder and you need to create another directory with the name 'src':
```bash
mkdir src
```
At this point you are ready to set up your ROS workspace with the following command:
```bash
catkin_make
```

To help you understand the steps I attach a photo of what your terminal should look like:

![Bash](https://github.com/alessandrotrovatello/RT1_assignment2/blob/main/images/bash.png)

Then, press enter and your ROS workspace will be compiled, two directory are will be created inside your workspace.
The last step is to insert the path of your ROS workspace inside the `.bashrc` file to make sure you are running your ROS workspace every time you open the terminal. So, go back with the following command:
```bash
cd ..
```
And open your `.bashrc` file:
```bash
gedit .bashrc
```
Scroll the page until the end and insert your ROS workspace path, if you have followed the steps it should be:
```bash
source ~/directory_name/devel/setup.bash
```
In this way (note that there should be a command line to start ROS Noetic `source /opt/ros/noetic/setup.bash`):

![Source](https://github.com/alessandrotrovatello/RT1_assignment2/blob/main/images/source.png)

Save the file and restart the terminal, you're Done!

<a name="clone"></a>
To download this repository, move to the `src` folder inside your ROS workspace. Now you can clone this repository by using [GIT](https://github.com/git-guides/install-git) in this way:
```bash
git clone https://github.com/alessandrotrovatello/RT1_assignment2
```
After that, return in your ROS workspace, using `cd ..` command, and re-compile your workspace using `catkin_make`.

You are ready tu run the code!
<a name="howto"></a>
## How to run the code

