#! /usr/bin/env python

# A service node that, when called, returns
# the coordinates of the last target sent
# by the user.

import rospy
import assignment_2_2023.msg
from assignment_2_2023.srv import last_target, last_targetResponse

def clbk_service():
# Callback to return the last target coordinates to the service

	return last_targetResponse(last_target_x, last_target_y)


def clbk_goal():
# Callback to retrieve the last target coordinates

	global last_target_x, last_target_y
	
	# Retrieve the target coordinates
	last_target_x = msg.goal.target_pose.pose.position.x
	last_target_y = msg.goal.target_pose.pose.position.y


def main():

	# Initialize the service node
	rospy.init_node("last_target")
	
	# Creating a ROS service server
	rospy.Service("last_target", last_target, clbk_service)
	
	# Creating a ROS subscriber to subscribe on /reaching_goal/goal topic the last target coordinates
	rospy.Subscriber('/reaching_goal/goal', assignment_2_2023.msg.PlanningActionGoal, clbk_goal)
	
	# Keep the node running
	rospy.spin()


if __name__=="__main__":
	main()
