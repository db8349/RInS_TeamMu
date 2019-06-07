#!/usr/bin/env python
import roslib
roslib.load_manifest('robot')
import rospy

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3, Quaternion, Twist, Pose, PointStamped
from std_msgs.msg import ColorRGBA, String
import math

from robot.msg import QRCode, Cylinder
import classifier as cs

import tf2_geometry_msgs
import tf2_ros

rospy.init_node('cylinder_filter', anonymous=False)

debug = rospy.get_param('/debug')

# Cylinder pose filtering
cylinder_exlusion_bounds = float(rospy.get_param("~cylinder_exlusion_bounds"))

class Main():
	def __init__(self):
		# Object we use for transforming between coordinate frames
		self.tf_buf = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

		self.cylinder_pub = rospy.Publisher("cylinder_filter/cylinder", Cylinder, queue_size=100)
		rospy.Subscriber("cylinder_detect/cylinder", PointStamped, self.cylinder)
		rospy.Subscriber("cylinder_color", String, self.cylinder_color)

		self.cylinder_publish = []

		self.color = None # Cylinder color buffer

	def cylinder_color(self, color):
		#rospy.loginfo("New color: {}".format(color.data))
		self.color = color.data

	def cylinder(self, point):
		if math.isnan(point.point.x):
			return

		pose = Pose(Point(point.point.x, point.point.y, point.point.z), Quaternion())
		rospy.loginfo("New Cylinder: {}, {}".format(pose.position.x, pose.position.y))

		# Filter the cylinder and decide if we accept it
		if not self.in_cylinder_publish(pose):
			self.cylinder_publish.append(pose)
			self.publish_cylinder(pose)
			return pose

		return None

	def in_cylinder_publish(self, old_pose):
		for new_pose in self.cylinder_publish:
			if abs(old_pose.position.x - new_pose.position.x) <= cylinder_exlusion_bounds and \
				abs(old_pose.position.y - new_pose.position.y) <= cylinder_exlusion_bounds:
				return True

		return False

	def publish_cylinder(self, pose):
		rospy.loginfo("self.color: {}".format(self.color))

		cylinder = Cylinder()
		cylinder.curr_pose = self.get_curr_pose()
		cylinder.cylinder_pose = pose
		cylinder.color = self.color
		rospy.loginfo("Cylinder Color: ".format(self.color))

		self.cylinder_pub.publish(cylinder)

	def init(self):
		pass

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

if __name__ == '__main__':
	if debug:
		rospy.loginfo("cylinder_filter DEBUG mode")

	main = Main()
	main.init()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		nav_manager.stop()
		print("Shutting down")