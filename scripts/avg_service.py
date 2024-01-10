#! /usr/bin/env python

# Service node that subscribes to the robot's position
# and velocity (using the custom message) and implements
# a server to retrieve the distance of the robot from the
# target and the robot's average speed.

import rospy
import math
from assignment_2_2023.srv import Avg_dist_vel, Avg_dist_velResponse
from assignment_2_2023.msg import Info


def clbk_info(msg):

# Initialize all necessary variables
	global mean_vel_x, mean_vel_z, dist
	
	linear_vel_x = [] 
	angular_vel_z = []
	
	# Retrieve the value of window_size from the ROS parameter
	# written by the user in the terminal
	window_size = rospy.get_param("window_size")

	# Get the position and velocity of the robot
	x_robot = msg.x
	y_robot = msg.y
	linear_vel_x.append(msg.vel_x)
	angular_vel_z.append(msg.vel_z)

	# Get the target coordinates
	x_target = rospy.get_param("des_pos_x")
	y_target = rospy.get_param("des_pos_y")
	
# Calculate the average linear velocity along x axis
# and the average angular velocity around z axis of the robot

	# Create a lists taking the last [window_size] elements
	instant_vel_x = linear_vel_x[-window_size:] # List of instant linear velocity x of the robot
	instant_vel_z = angular_vel_z[-window_size:] # List of instant angular velocity z of the robot

	# Calculate the arithmetic mean
	mean_vel_x = sum(instant_vel_x) / len(instant_vel_x) # Avg of linear velocity along x axis
	mean_vel_z = sum(instant_vel_z) / len(instant_vel_z) # Avg of angular velocity around z axis

# Calculate the distance between the target and the actual position of the robot

	# Euclidean distance formula
	dist = math.sqrt((x_target - x_robot)**2 + (y_target - y_robot)**2)
	

def clbk_avg(request):
# Callback to return the:
# - Distance between target and robot
# - Average of the linear velocity along x axis
# - Average of the angular velocity around z axis
# to the service
	global mean_vel_x, mean_vel_z, dist
	
	dist = round(dist,3)
	mean_vel_x = round(mean_vel_x,3)
	mean_vel_z = round(mean_vel_z,3)
	
	return Avg_dist_velResponse(dist, mean_vel_x, mean_vel_z)


def main():
	
	# Initialize the service node
	rospy.init_node("avg_dist_vel")
	
	# Creating a ROS service server
	rospy.Service("avg_dist_vel", Avg_dist_vel, clbk_avg)
	
	# Creating a ROS subscriber to listens to the /robot_pos_vel topic
	rospy.Subscriber('/robot_pos_vel', Info, clbk_info)
	
	# Keep the node running
	rospy.spin()


if __name__ == "__main__":
	main()
