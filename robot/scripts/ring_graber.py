#!/usr/bin/env python
import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import PointStamped, Vector3, Pose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import message_filters

import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from geometry_msgs.msg import Twist
PI = 3.1415926535897

class Main():
	def __init__(self):
		global debug

		self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

		self.vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)

		self.tf2_buffer = tf2_ros.Buffer()
		self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)

		self.marker_array = MarkerArray()
		self.marker_num = 1
		self.markers_pub = rospy.Publisher('markers', MarkerArray, queue_size=10000)

	def pickup(self, pose):
		rospy.loginfo("Got ring position ({}, {})".format(pose.position.x, pose.position.y))
		self.show_point(pose)

	def show_point(self, pose):
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
		marker.color = ColorRGBA(0, 1, 0, 1)
		self.marker_array.markers.append(marker)

		self.markers_pub.publish(self.marker_array)


if __name__ == '__main__':
		
		rospy.init_node('ring_graber', anonymous=False)
		try:
			m = Main()

			rospy.Subscriber("grab_3d_ring", Pose, m.pickup)

			rospy.spin()
		except rospy.ROSInterruptException:
			pass
