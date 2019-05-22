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
import math
from robot.msgs import MoveForward, Rotate

rospy.init_node('nav_manager', anonymous=False)

debug = rospy.get_param('/debug')

class NavManager():
	def __init__(self):
		self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		self.request_queue = []

		rospy.Subscribe("/nav_manager/go_to", Pose, lambda pose : request_queue.append((self.go_to, pose)))
		rospy.Subscribe("/nav_manager/move_forward", MoveForward, lambda move_forward : request_queue.append((self.move_forward, move_forward)))
		rospy.Subscribe("/nav_manager/rotate", Rotate, lambda rotate : request_queue.append((self.rotate, rotate)))

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
		while not goal_state == GoalStatus.SUCCEEDED and not rospy.is_shutdown():
			self.ac.wait_for_result(rospy.Duration(0.005))

			goal_state = self.ac.get_state()
			#Possible States Are: PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST.

			if goal_state == GoalStatus.SUCCEEDED:
				if debug: rospy.loginfo("The point was reached!")

	def rotate(self, rotate):
		vel_msg = Twist()

		angular_speed = rotate.speed*2*math.pi/360
		relative_angle = rotate.angle*2*math.pi/360

		vel_msg.linear.x = 0
		vel_msg.linear.y = 0
  		vel_msg.linear.z = 0
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
		if debug: rospy.loginfo("Rotation done!")

	def move_forward(self, move_forward):
		vel_msg = Twist()

		vel_msg.linear.x = move_forward.speed
		vel_msg.linear.y = 0
  		vel_msg.linear.z = 0
  		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.angular.z = 0

		t0 = rospy.Time.now().to_sec()
		current_distance = 0

		while current_distance < move_forward.distance and not rospy.is_shutdown():
		    self.vel_pub.publish(vel_msg)
		    t1 = rospy.Time.now().to_sec()
		    current_distance = move_forward.speed*(t1-t0)

		# Forcing our robot to stop
		vel_msg.linear.x = 0
		self.vel_pub.publish(vel_msg)
		if debug: rospy.loginfo("MoveForward done!")


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