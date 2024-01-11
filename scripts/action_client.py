#! /usr/bin/env python

# Action client node, allowing the user to set a target (x,y) or to cancel it.
# The node also publishes the robot position and velocity as a custom message
# (x,y,vel_x,vel_z), by relying on the values published on the topic /odom.

import rospy
import select
import actionlib
import actionlib.msg
import assignment_2_2023.msg
from std_srvs.srv import *
from nav_msgs.msg import Odometry
from assignment_2_2023.msg import Info
from geometry_msgs.msg import Point, Pose, Twist



def clbk_odom(msg):
# Callback function that process incoming Odometry messages,
# extract informations about: position x, position y, linear velocity x and
# angular velocity z; Save these informations in a custom message Info()
# and pubblish that informations on /robot_pos_vel topic.
	
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


def clbk_feedback(feedback):
	if feedback.stat == "Target reached!":
		print(feedback)
		print("Press 'Enter' to set a new goal\n")
	if feedback.stat == "Target cancelled!":
		print(feedback)
        
        
def action():
	
	# Execution of client request to the server
	client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2023.msg.PlanningAction)
	# Block the execution until communication with server is established
	client.wait_for_server()
	
	# While loop until the program finished or interrupted
	while not rospy.is_shutdown():

		# Get goal coordinates from user
		print("Set the goal coordinates!")
		try:
			x = float(input("Enter x coordinate: "))
			y = float(input("Enter y coordinate: "))
			# Checking the correctness of inputs
			if -9 <= x <= 9 and -9 <= y <= 9:
				# Prints the set goal
				print(f"Goal coordinates set: (x={x},y={y})")
			else:
				print("Invalid input. Please enter x and y coordinates within the range -9 to 9.")
				continue
		except ValueError:
			print("Invalid input. Please enter a number.")
			continue
		
		# Initialize an instance of PlanningGoal() to pass the goal coordinates.
		goal = assignment_2_2023.msg.PlanningGoal()
		goal.target_pose.pose.position.x = x
		goal.target_pose.pose.position.y = y
		
		# Send the goal to the action server and set callbacks for when:
		# done_cb = The action is done.
		# active_cb = The action becomes active.
		# feedback_cb = The action sends feedback.
		client.send_goal(goal, None, None, clbk_feedback)
		
		# Now the robot is reaching the goal. If we want to stop the robot we need
		# to cancel the goal reading the input user without blocking the execution.
		while not client.get_result():
			print("Robot is reaching the goal. Press 'c' to cancel the goal.")
			cancel = select.select([sys.stdin], [], [], 0.1)
			if cancel:
				user_input = sys.stdin.readline().strip()
				if user_input == 'c':
					client.cancel_goal()
					break
			
		
	
def main():
	global pub, sub
	
	# Initialize the service node
	rospy.init_node('action_client')

	# Creating a ROS publisher to publish on /robot_pos_vel topic the position and velocity of Robot
	pub = rospy.Publisher('/robot_pos_vel', Info, queue_size=10)

	# Creating a ROS subscriber to listens to the /odom topic
	sub = rospy.Subscriber('/odom', Odometry, clbk_odom)
	
	# Run the action function
	action()


if __name__ == "__main__":
	main()
