#!/usr/bin/env python
import roslib
roslib.load_manifest('robot')
import rospy

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3, Quaternion, Twist, Pose, PointStamped
from std_msgs.msg import ColorRGBA, String
import math

from robot.msg import Numbers, Circle, QRCode, Cylinder, Approaches
import classifier as cs

import tf2_geometry_msgs
import tf2_ros

from tf.transformations import quaternion_from_euler

rospy.init_node('controller', anonymous=False)

debug = rospy.get_param('/debug')

class Detect:
	NONE = 0
	CIRCLE = 1
	CYLINDER = 2

class Main():
	def __init__(self):
		self.approach_dist = (float)(rospy.get_param('~approach_dist'))

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

		self.nav_approach_pub = rospy.Publisher("nav_manager/approach", Pose, queue_size=100)
		self.nav_approaches_pub = rospy.Publisher("nav_manager/approaches", Approaches, queue_size=100)
		self.qr_running_pub = rospy.Publisher("qr_detect/running", String, queue_size=100)
		self.numbers_running_pub = rospy.Publisher("numbers_detect/running", String, queue_size=100)
		self.nav_quit_pub = rospy.Publisher("nav_manager/quit", String, queue_size=100)
		self.nav_skip_request_pub = rospy.Publisher("nav_manager/skip_request", String, queue_size=100)

		rospy.Subscriber("circle_detect/circle", Circle, self.circle)
		rospy.Subscriber("cylinder_filter/cylinder", Cylinder, self.cylinder)

		rospy.Subscriber("qr_detect/qr_code", QRCode, self.qr)
		rospy.Subscriber("numbers_detect/numbers", Numbers, self.numbers)
		rospy.Subscriber("nav_manager/approach_done", String, self.approach_done)

		#rospy.Subscriber("/clicked_point", PointStamped, self.spoffed_point)


	def qr(self, qr):
		data = qr.data

		if "http" in data:
			rospy.loginfo("Got QR data: {}".format(data))
			self.qr_data = data
			self.atempt_classify()

		if self.detect == Detect.CIRCLE and self.classify_result == None:
			self.qr_data = data
			rospy.loginfo("Circle QR data: {}".format(self.qr_data))
			self.atempt_classify()

			self.nav_skip_request_pub.publish("")
			self.detect = Detect.NONE

		elif self.detect == Detect.CYLINDER:
			if "http" not in data:
				self.cylinders[-1].qr_data = data
				rospy.loginfo("Cylinder QR data: {}".format(self.cylinders[-1].qr_data))

				self.nav_skip_request_pub.publish("")
				self.detect = Detect.NONE

		self.qr_running_pub.publish("False")
		self.numbers_running_pub.publish("False")


	def circle(self, circle):
		self.qr_running_pub.publish("True")
		self.numbers_running_pub.publish("True")

		# Circle stage
		self.detect = Detect.CIRCLE

		rospy.loginfo("New Circle: {}, {} -- {}".format(circle.pose.position.x, circle.pose.position.y, circle.color))
		self.show_point(circle.pose, ColorRGBA(0, 0, 1, 1))

		approach_vectors = []
		for approach in circle.approaches:
			circle_approach = self.approach_transform(approach, circle.pose)
			approach_vectors.append(circle_approach)
			self.show_point(circle_approach, ColorRGBA(0, 1, 0, 1))

		if len(approach_vectors) > 0:
			middle = approach_vectors[len(approach_vectors)/2]
			del approach_vectors[len(approach_vectors)/2]
			approach_vectors.insert(0, middle)

		
		approaches = Approaches()
		approaches.poses = approach_vectors
		self.nav_approaches_pub.publish(approaches)

	def numbers(self, num):
		if self.classify_result == None:
			rospy.loginfo("Setting Numbers: {}, {}".format(num.first, num.second))
			self.num = num
			self.atempt_classify()

			self.nav_skip_request_pub.publish("")
			self.detect = Detect.NONE

		self.qr_running_pub.publish("False")
		self.numbers_running_pub.publish("False")

	def cylinder(self, cylinder):
		self.qr_running_pub.publish("True")
		self.detect = Detect.CYLINDER

		rospy.loginfo("New Cylinder: {}, {} -- {}".format(cylinder.pose.position.x, cylinder.pose.position.y, cylinder.color))
		self.cylinders.append(cylinder)

		self.approach_cylinder(cylinder)

		if self.classify_result != None:
			self.check_has_cylinder(self.classify_result)

	def approach_done(self, data):
		rospy.loginfo("Approach done")
		#self.qr_running_pub.publish("False")
		self.numbers_running_pub.publish("False")

	def approach_cylinder(self, cylinder):
		color = ColorRGBA(1, 0, 0, 1)
		if cylinder.color == "green":
			color = ColorRGBA(0, 1, 0, 1)
		elif cylinder.color == "blue":
			color = ColorRGBA(0, 0, 1, 1)
		elif cylinder.color == "yellow":
			color = ColorRGBA(1, 1, 0, 1)

		self.show_cylinder(cylinder.pose, color)

		approach_vectors = []
		for approach in cylinder.approaches:
			cylindre_approach = self.approach_transform(approach, cylinder.pose)
			approach_vectors.append(cylindre_approach)
			self.show_point(cylindre_approach, ColorRGBA(0, 1, 0, 1))

		if len(approach_vectors) > 0:
			middle = approach_vectors[len(approach_vectors)/2]
			del approach_vectors[len(approach_vectors)/2]
			approach_vectors.insert(0, middle)

		approaches = Approaches()
		approaches.poses = approach_vectors
		self.nav_approaches_pub.publish(approaches)

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

	def show_cylinder(self, pose, color=ColorRGBA(1, 0, 0, 1)):
		self.marker_num += 1
		marker = Marker()
		marker.header.stamp = rospy.Time.now()
		marker.header.frame_id = '/map'
		marker.pose = pose
		marker.type = Marker.CYLINDER
		marker.action = Marker.ADD
		marker.frame_locked = False
		marker.id = self.marker_num
		marker.scale = Vector3(0.35, 0.35, 0.6)
		marker.color = color
		self.marker_array.markers.append(marker)

		self.markers_pub.publish(self.marker_array)

	def approach_transform(self, curr_pose, target_pose):
		dx = target_pose.position.x - curr_pose.position.x
		dy = target_pose.position.y - curr_pose.position.y
		v = Vector3(dx, dy, 0)
		v_len = math.sqrt(math.pow(v.x, 2) + math.pow(v.y, 2))
		v_new_len = v_len - self.approach_dist
		v_mul = v_new_len/v_len

		v = Vector3(v.x * v_mul, v.y * v_mul, 0)
		v = Vector3(v.x + curr_pose.position.x, v.y + curr_pose.position.y, 0)


		rad = math.atan2(dy, dx)
		q = quaternion_from_euler(0, 0, rad)
		q = Quaternion(q[0], q[1], q[2], q[3])
		
		pose = Pose(v, q)

		return pose

	def atempt_classify(self):
		if self.qr_data != None and self.num != None and self.classify_result == None:
			rospy.loginfo("Calling with parameters: {}, {}, {}".format(self.qr_data, self.num.first, self.num.second))
			self.classify_result = cs.classify(self.qr_data, self.num.first, self.num.second)
			rospy.loginfo("Classification classify_result: {}".format(self.classify_result))

			# Turn on the cylinder stage in circle sense
			rospy.Publisher("circle_detect/cylinder_stage", String, queue_size=100).publish("")
			rospy

			self.check_has_cylinder(self.classify_result)

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

	def check_has_cylinder(self, color):
		for cylinder in self.cylinders:
			if cylinder.color == color:
				self.approach_cylinder(cylinder)
				self.nav_quit_pub.publish("")
				rospy.loginfo("Done!")

	def spoffed_point(self, point):
		pose = Pose(Point(point.point.x, point.point.y, 0), Quaternion())

		cylinder = Cylinder()
		cylinder.pose = pose

		self.cylinder(cylinder)

		'''
		circle = Circle()
		circle.pose = pose
		self.circle(circle)
		'''



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