#!/usr/bin/env python
import roslib
roslib.load_manifest('robot')
import rospy
import sensor_msgs.msg
import message_filters
from geometry_msgs.msg import Pose
from random import randint

debug = False

def init_sim_3d_ring_points():
	points = []

	pose = Pose()
	pose.position.x = 2.13
	pose.position.y = 2.51
	pose.position.z = 0.2
	points.append(pose)

	pose = Pose()
	pose.position.x = 3.64
	pose.position.y = 2.38
	pose.position.z = 0.2
	points.append(pose)

	pose = Pose()
	pose.position.x = 4.83
	pose.position.y = 1.17
	pose.position.z = 0.2
	points.append(pose)

	return points

def init_3d_ring_points():
	points = []

	pose = Pose()
	pose.position.x = -0.0393
	pose.position.y = -1.8
	pose.position.z = 0.2
	points.append(pose)

	pose = Pose()
	pose.position.x = 0.593
	pose.position.y = -1.28
	pose.position.z = 0.2
	points.append(pose)

	pose = Pose()
	pose.position.x = -0.79
	pose.position.y = 0.2
	pose.position.z = 0.2
	points.append(pose)

	return points

def get_random_point(points):
	return points[randint(0, len(points)-1)]

def talker():
	points = init_3d_ring_points()
	if debug:
		points = init_sim_3d_ring_points()

	i = 0
	pub = rospy.Publisher('grab_3d_ring', Pose, queue_size=10)
	while not rospy.is_shutdown():
		raw_input("Press Enter to publish a ring with position ({}, {})".format(points[i].position.x, points[i].position.y))
		pub.publish(points[i])
		i = (i + 1) % len(points)

if __name__ == '__main__':

		rospy.init_node('test_pub', anonymous=False)
		try:
			talker()
		except rospy.ROSInterruptException:
			pass