.. RT1_Assignment2 documentation master file, created by
   sphinx-quickstart on Thu Mar 21 17:38:36 2024.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to RT1_Assignment2's documentation!
===========================================

The main task of this assignment is to create a ROS package which must include the following three nodes:

1. A node that implements an action client, allowing the user to set a target (x, y) or to cancel it. Trying to use the feedback/status of the action server to know when the target has been reached. The node also publishes the robot position and velocity as a custom message (x,y, vel_x, vel_z), by relying on the values published on the topic /odom;
2. A service node that, when called, returns the coordinates of the last target sent by the user;
3. Another service node that subscribes to the robot's position and velocity (using the custom message) and implements a server to retrieve the distance of the robot from the target and the robot's average speed.

.. toctree::
   :maxdepth: 2
   :caption: Contents:



Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`

Action Client
=============
.. automodule:: scripts.action_client
	:members:
	
Last Target Service
===================
.. automodule:: scripts.last_target_service
	:members:
	
Average Service
===============
.. automodule:: scripts.avg_service
	:members:
