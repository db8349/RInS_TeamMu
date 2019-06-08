#!/usr/bin/env python
import roslib
roslib.load_manifest('robot')
import rospy
import sensor_msgs.msg
import message_filters
from geometry_msgs.msg import Pose
from random import randint

import tf2_geometry_msgs
import tf2_ros

from robot.msg import Circle



def init_debug_circle_points():
	points = []

	pose = Pose()
	pose.position.x = 1.81
	pose.position.y = 2.1
	pose.position.z = 0.2
	points.append(pose)
	
	return points

def init_ring_points():
	points = []

	pose = Pose()
	pose.position.x = 0.576
	pose.position.y = -1.05
	pose.position.z = 0.2
	points.append(pose)
	
	return points

class Test():
	def __init__(self):
		# Object we use for transforming between coordinate frames
		self.tf_buf = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

	def get_curr_pose(self):
		trans = None
		while trans == None:
			try:
				trans = self.tf_buf.lookup_transform('map', 'base_link', rospy.Time(0))
			except Exception as e:
				print(e)
				rospy.sleep(0.01)
				continue

		curr_pose = Pose()
		curr_pose.position.x = trans.transform.translation.x
		curr_pose.position.y = trans.transform.translation.y
		curr_pose.position.z = trans.transform.translation.z
		curr_pose.orientation = trans.transform.rotation

		return curr_pose

	def get_random_point(self, points):
		return points[randint(0, len(points)-1)]


	def talker(self):
		points = init_debug_circle_points()

		i = 0
		pub = rospy.Publisher("circle_detect/circle", Circle, queue_size=10)
		while not rospy.is_shutdown():
			raw_input("Press Enter to publish a point of interest with position ({}, {})".format(points[i].position.x, points[i].position.y))
			circle = Circle()
			circle.color = ""
			circle.pose = points[i]

			pub.publish(circle)
			i = (i + 1) % len(points)

if __name__ == '__main__':

		rospy.init_node('test_pub', anonymous=False)

		test = Test()
		test.talker()

		try:
			talker()
		except rospy.ROSInterruptException:
			pass