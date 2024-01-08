#! /usr/bin/env python

# Action client node, allowing the user to set a target (x,y) or to cancel it.
# The node also publishes the robot position and velocity as a custom message
# (x,y,vel_x,vel_z), by relying on the values published on the topic /odom.

import rospy
import select
import time
import sys
import actionlib
import actionlib.msg
import assignment_2_2023.msg
from assignment_2_2023.msg import Info
from std_srvs.srv import *
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry




def clbk_odom(msg):
# Callback function that process incoming Odometry messages,
# extract informations about: position x, position y, linear velocity x and
# angular velocity z; Save these informations in a custom message Info()
# and pubblish that informations on /robot_pos_vel topic.

	global pub
	
	# Initialize a new message
	# The struct of Info() is (x,y,vel_x,vel_z)
	new_info = Info()
	
	# Retrieve the position and velocity from geometry_msgs
	# and save the values inside the new message
	new_info.x = msg.pose.pose.position.x			# x position coordinate
	new_info.y = msg.pose.pose.position.y			# y position coordinate
	new_info.vel_x = msg.twist.twist.linear.x 		# linear velocity along x axis
	new_info.vel_z = msg.twist.twist.angular.z		# angular velocity around z axis
	
	# Pubblish new message on /robot_pos_vel topic 
	pub.publish(new_info)

def clbk_done(status, result):
	print("Action done: ", status)
	print("Result: ", result)
	
def clbk_active():
	print("Action is now active")
	
def clbk_feedback(feedback):
	if feedback.stat == "Target reached!":
		print("Received feedback: ", feedback)


def action():
	
	# Execution of client request to the server
	client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2023.msg.PlanningAction)
	# Block the execution until communication with server is established
	client.wait_for_server()

	time.sleep(5)
	# While loop until the program finished or interrupted
	while not rospy.is_shutdown():

		# Get goal coordinates from user
		print("Set the goal coordinates!")
		try:
			x = float(input("Enter x coordinate: "))
			y = float(input("Enter y coordinate: "))
		# Checking the correctness of inputs
		except:
			print("Invalid input. Please enter a number.")
			continue
		
		# Initialize an instance of PlanningGoal() to pass the goal coordinates.
		goal = assignment_2_2023.msg.PlanningGoal()
		goal.target_pose.pose.position.x = x
		goal.target_pose.pose.position.y = y
		
		# Send the goal to the action server and set the callbacks for when:
		# done_cb = The action is done.
		# active_cb = The action becomes active.
		# feedback_cb = The action sends feedback.
		client.send_goal(goal, clbk_done, None, clbk_feedback)
		
		feedback = assignment_2_2023.msg.PlanningFeedback()
		# Now the robot is reaching the goal. If we want to stop the robot we need
		# to cancel the goal reading the input user without blocking the execution.
		while feedback.stat != "Target reached!":
			print("Robot is reaching the goal. Press 'c' to cancel the goal")
			cancel = select.select([sys.stdin], [], [], 0.1)
			if cancel:
				user_input = sys.stdin.readline().strip()
				if user_input == 'c':
					print("Goal cancelled.")
					client.cancel_goal()
					break
			
		
	
def main():
	global pub, sub
	
	rospy.init_node('action_client')
	
	pub = rospy.Publisher('/robot_pos_vel', Info, queue_size=10)
	sub = rospy.Subscriber('/odom', Odometry, clbk_odom)
	
	action()


if __name__ == "__main__":
	main()
