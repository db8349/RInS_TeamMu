#!/usr/bin/env python
import roslib
roslib.load_manifest('robot')
import rospy

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3, Quaternion, Twist, Pose
from std_msgs.msg import ColorRGBA, String
import math

from robot.msg import Numbers, Circle, QRCode, Cylinder
import classifier as cs

import tf2_geometry_msgs
import tf2_ros

rospy.init_node('controller', anonymous=False)

debug = rospy.get_param('/debug')

class Detect:
	NONE = 0
	CIRCLE = 1
	CYLINDER = 2

class Main():
	def __init__(self):
		self.marker_array = MarkerArray()
		self.marker_num = 1
		self.markers_pub = rospy.Publisher('markers', MarkerArray, queue_size=10000)

		# Object we use for transforming between coordinate frames
		self.tf_buf = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

		self.detect = Detect.NONE

		# Circle stage
		self.qr_data = None
		self.num = None
		self.classify_result = None

		self.cylinders = []

		self.nav_approach_pub = rospy.Publisher("/nav_manager/approach", Pose, queue_size=100)
		self.qr_running_pub = rospy.Publisher("qr_detect/running", String, queue_size=100)
		self.numbers_running_pub = rospy.Publisher("numbers_detect/running", String, queue_size=100)

		rospy.Subscriber("circle_detect/circle", Circle, self.circle)
		rospy.Subscriber("cylinder_filter/cylinder", Cylinder, self.cylinder)

		rospy.Subscriber("qr_detect/qr_code", QRCode, self.qr)
		rospy.Subscriber("numbers_detect/numbers", Numbers, self.numbers)


	def qr(self, qr):
		data = qr.data

		if self.detect == Detect.CIRCLE and self.qr_data == None:
			# Check if the qr_data is a link
			if "http" not in data:
				return

			self.qr_data = data
			rospy.loginfo("Circle QR data: {}".format(self.qr_data))
			self.detect = Detect.NONE

			self.atempt_classify()
			self.qr_running_pub.publish("False")
			self.numbers_running_pub.publish("False")
		elif self.detect == Detect.CYLINDER:
			self.cylinders[-1].qr_data = data
			rospy.loginfo("Cylinder QR data: {}".format(self.cylinders[-1].qr_data))
			self.detect = Detect.NONE
			self.qr_running_pub.publish("False")
			self.numbers_running_pub.publish("False")


	def circle(self, circle):
		self.qr_running_pub.publish("True")
		self.numbers_running_pub.publish("True")

		# Circle stage
		self.detect = Detect.CIRCLE

		rospy.loginfo("New Circle: {}, {}".format(circle.circle_pose.position.x, circle.circle_pose.position.y))
		self.show_point(circle.circle_pose, ColorRGBA(0, 0, 1, 1))
		circle_approach_pose = self.approach_transform(circle.curr_pose, circle.circle_pose, 0.4)
		#rospy.loginfo("Circle approach: ({}, {})".format(circle_approach_pose.position.x, circle_approach_pose.position.y))
		self.show_point(circle_approach_pose, ColorRGBA(0, 1, 0, 1))
		self.nav_approach_pub.publish(circle_approach_pose)

	def numbers(self, num):
		if self.detect == Detect.CIRCLE and self.classify_result == None and self.num == None:
			rospy.loginfo("Setting Numbers: {}, {}".format(num.first, num.second))
			self.num = num
			self.atempt_classify()
			self.detect = Detect.NONE
			self.qr_running_pub.publish("False")
			self.numbers_running_pub.publish("False")

	def cylinder(self, cylinder):
		self.qr_running_pub.publish("True")
		self.detect = Detect.CYLINDER

		rospy.loginfo("New Cylinder: {}, {}".format(cylinder.cylinder_pose.position.x, cylinder.cylinder_pose.position.y))
		self.cylinders.append(cylinder)

		self.approach_cylinder(cylinder)

	def approach_cylinder(self, cylinder):
		#rospy.loginfo("New Circle: {}, {}".format(circle.circle_pose.position.x, circle.circle_pose.position.y))
		self.show_point(cylinder.cylinder_pose, ColorRGBA(0, 0, 1, 1))
		cylinder_approach_pose = self.approach_transform(cylinder.curr_pose, cylinder.cylinder_pose, 0.4)
		#rospy.loginfo("Circle approach: ({}, {})".format(circle_approach_pose.position.x, circle_approach_pose.position.y))
		self.show_point(cylinder_approach_pose, ColorRGBA(0, 1, 0, 1))
		self.nav_approach_pub.publish(cylinder_approach_pose)

	def init(self):
		pass

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

	def approach_transform(self, curr_pose, point, dist):
		v = Vector3(point.position.x - curr_pose.position.x, point.position.y - curr_pose.position.y, 0)
		v_len = math.sqrt(math.pow(v.x, 2) + math.pow(v.y, 2))
		v_new_len = v_len - dist
		v_mul = v_new_len/v_len

		v = Vector3(v.x * v_mul, v.y * v_mul, 0)
		v = Vector3(v.x + curr_pose.position.x, v.y + curr_pose.position.y, 0)
		
		pose = Pose(v, curr_pose.orientation)

		return pose

	def atempt_classify(self):
		if self.qr_data != None and self.num != None:
			rospy.loginfo("Calling with parameters: {}, {}, {}".format(self.qr_data, self.num.first, self.num.second))
			self.classify_result = cs.classify(self.qr_data, self.num.first, self.num.second)
			rospy.loginfo("Classification classify_result: {}".format(self.classify_result))

			# Turn on the cylinder stage in circle sense
			rospy.Publisher("circle_detect/cylinder_stage", String, queue_size=100).publish("")

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
		rospy.loginfo("controller DEBUG mode")

	main = Main()
	main.init()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		nav_manager.stop()
		print("Shutting down")