#!/usr/bin/env python
import roslib
roslib.load_manifest('robot')
import rospy
import sensor_msgs.msg
import message_filters
from geometry_msgs.msg import Pose
from random import randint

def init_random_points():
	points = []

	pose = Pose()
	pose.position.x = 2.38
	pose.position.y = 4.53
	pose.position.z = 0.0
	points.append(pose)

	pose = Pose()
	pose.position.x = 1.81
	pose.position.y = 2.71
	pose.position.z = 0.0
	points.append(pose)

	pose = Pose()
	pose.position.x = 4.88
	pose.position.y = 0.714
	pose.position.z = 0.0
	points.append(pose)

	return points

def get_random_point(points):
	return points[randint(0, len(points)-1)]

def talker():
	points = init_random_points()
	pub = rospy.Publisher('pause', Pose, queue_size=10)
	while not rospy.is_shutdown():
		i = raw_input("Press Enter to publish random circle point")
		pub.publish(get_random_point(points))

if __name__ == '__main__':

		rospy.init_node('test_pub', anonymous=False)
		try:
			talker()
		except rospy.ROSInterruptException:
			pass