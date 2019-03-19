#!/usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

goal_state = GoalStatus.LOST

rospy.init_node('map_navigation', anonymous=False)

ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

while(not ac.wait_for_server(rospy.Duration.from_sec(2.0))):
              rospy.loginfo("Waiting for the move_base action server to come up")

goal = MoveBaseGoal()

#Sending a goal to the to a certain position in the map
goal.target_pose.header.frame_id = "map"
goal.target_pose.header.stamp = rospy.Time.now()
goal.target_pose.pose.position.x = 0.0
goal.target_pose.pose.position.y = 0.0
goal.target_pose.pose.orientation.w = 1.0

rospy.loginfo("Sending goal")
ac.send_goal(goal)

while (not goal_state == GoalStatus.SUCCEEDED):

	ac.wait_for_result(rospy.Duration(2))
	goal_state = ac.get_state()
	#Possible States Are: PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST.

	if not goal_state == GoalStatus.SUCCEEDED:
		rospy.loginfo("The goal has not been reached yet! Checking again in 2s.")
	else:
		rospy.loginfo("The goal was reached!")

GoalStatus.SUCCEEDED
