# 3D Robot Simulation

Second assignment of the Research Track 1 course of the Master's degree in Robotics Engineering at the University of Genoa.

## Assignment

The main task of this assignment is to create a ROS package which must include the following three nodes:
- A node that implements an action client, allowing the user to set a target (x, y) or to cancel it. Try to use the feedback/status of the action server to know when the target has been reached. The node also publishesthe robot position and velocity as a custom message (x,y, vel_x, vel_z), by relying on the values published on the topic /odom;
- A service node that, when called, returnsthe coordinates of the last target sent by the user;
- Another service node that subscribes to the robot’s position and velocity (using the custom message) and implements a server to retrieve the distance of the robot from the target and the robot’s average speed.
