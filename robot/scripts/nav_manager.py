#!/usr/bin/env python
import roslib
import rospy
import sensor_msgs.msg
import message_filters
import tf2_ros

import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3, Quaternion, Twist, Pose
from std_msgs.msg import ColorRGBA

rospy.init_node('nav_manager', anonymous=False)

debug = rospy.get_param('/debug')

class NavManager():
	def __init__(self):
		global debug

		self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

		self.vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)

		self.marker_array = MarkerArray()
		self.marker_num = 1
		self.markers_pub = rospy.Publisher('markers', MarkerArray, queue_size=10000)
	
	def init(self):
		while(not self.ac.wait_for_server(rospy.Duration.from_sec(2.0))):
			rospy.loginfo("Waiting for the move_base action server to come up")
			if rospy.is_shutdown():
				return

	def explore(self):
		print("Implement explore")
		pass

	def go_to(self, goal):
		self.stop()

		self.ac.send_goal(goal)

		goal_state = GoalStatus.LOST
		while (not goal_state == GoalStatus.SUCCEEDED) and len(self.ring_poses) == 0:
			self.ac.wait_for_result(rospy.Duration(0.005))

			goal_state = self.ac.get_state()
			#Possible States Are: PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST.

			if goal_state == GoalStatus.SUCCEEDED:
				rospy.loginfo("The point was reached!")

			if rospy.is_shutdown():
				return

	def rotate(self, speed, angle):
		vel_msg = Twist()

		angular_speed = speed*2*PI/360
		relative_angle = angle*2*PI/360

		vel_msg.linear.x=0
		vel_msg.linear.y=0
  		vel_msg.linear.z=0
  		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.angular.z = abs(angular_speed)

		t0 = rospy.Time.now().to_sec()
		current_angle = 0

		while current_angle < relative_angle and not rospy.is_shutdown():
		    self.vel_pub.publish(vel_msg)
		    t1 = rospy.Time.now().to_sec()
		    current_angle = angular_speed*(t1-t0)

		# Forcing our robot to stop
		vel_msg.angular.z = 0
		self.vel_pub.publish(vel_msg)

	def move_forward(self, speed, distance):
		vel_msg = Twist()

		vel_msg.linear.x=speed
		vel_msg.linear.y=0
  		vel_msg.linear.z=0
  		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.angular.z = 0

		t0 = rospy.Time.now().to_sec()
		current_distance = 0

		while current_distance < distance and not rospy.is_shutdown():
		    self.vel_pub.publish(vel_msg)
		    t1 = rospy.Time.now().to_sec()
		    current_distance = speed*(t1-t0)

		# Forcing our robot to stop
		vel_msg.linear.x = 0
		self.vel_pub.publish(vel_msg)


	def show_point(self, pose, color=ColorRGBA(1, 0, 0, 1)):
		self.marker_num += 1
		marker = Marker()
		marker.header.stamp = rospy.Time.now()
		marker.header.frame_id = '/map'
		marker.pose = pose
		marker.type = Marker.CUBE
		marker.action = Marker.ADD
		marker.frame_locked = False
		marker.id = self.marker_num
		marker.scale = Vector3(0.1, 0.1, 0.1)
		marker.color = color
		self.marker_array.markers.append(marker)

		self.markers_pub.publish(self.marker_array)

	def stop(self):
		self.ac.cancel_goal()

if __name__ == '__main__':
		if debug:
			rospy.loginfo("nav_manager DEBUG mode")

		nav_manager = NavManager()
		nav_manager.init()

		try:
		rospy.spin()
	except KeyboardInterrupt:
		nav_manager.stop()
		print("Shutting down")