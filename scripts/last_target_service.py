#! /usr/bin/env python

"""
.. module: last_target_service

	:platform: Unix
	:synopsis: Action client node.
.. moduleauthor:: Alessandro Trovatello

A service node that, when called, returns the coordinates of the last target sent by the user.

Subscribes to:
	/reaching_goal/goal
"""

import rospy
import assignment_2_2023.msg
from assignment_2_2023.srv import Last_target, Last_targetResponse

def clbk_service(request):
	"""
	Callback to return the last target coordinates to the service
	
	:param request: parameter representing the request sent to the ROS service. When a client node calls the last_target service, it sends a request to the service, and this request is received as a parameter in the clbk_service function.
	"""
	global last_target_x, last_target_y
	
	print("Last target x coordinate is: ", last_target_x)
	print("Last target y coordinate is: ", last_target_y)
	print("---------------------------------")

	return Last_targetResponse(last_target_x, last_target_y)


def clbk_goal(msg):
	"""
	Callback to return the last target coordinates to the service
	
	:param msg: actual target position.
	"""
	global last_target_x, last_target_y
	
	"""Retrieve the target coordinates"""
	last_target_x = msg.goal.target_pose.pose.position.x
	last_target_y = msg.goal.target_pose.pose.position.y


def main():
	"""
	Main function in which the ros node is initialized and the publisher and subscriber are initialized.
	"""

	"""Initialize the service node"""
	rospy.init_node("last_target")
	
	"""Creating a ROS service server"""
	rospy.Service("last_target", Last_target, clbk_service)
	
	"""Creating a ROS subscriber to subscribe on /reaching_goal/goal topic the last target coordinates"""
	rospy.Subscriber('/reaching_goal/goal', assignment_2_2023.msg.PlanningActionGoal, clbk_goal)
	
	"""Keep the node running"""
	rospy.spin()


if __name__ == "__main__":
	main()
