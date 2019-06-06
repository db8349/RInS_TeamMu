#!/usr/bin/env python
import roslib
roslib.load_manifest('robot')
import rospy

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3, Quaternion, Twist, Pose
from std_msgs.msg import ColorRGBA, String
import math

from robot.msg import QRCode, Cylinder
import classifier as cs

import tf2_geometry_msgs
import tf2_ros

rospy.init_node('cylinder_filter', anonymous=False)

debug = rospy.get_param('/debug')

# Cylinder pose filtering
cylinder_required_cylinders = float(rospy.get_param("~cylinder_required_cylinders"))
cylinder_grouping_tolerance = float(rospy.get_param("~cylinder_grouping_tolerance"))
cylinder_exlusion_bounds = float(rospy.get_param("~cylinder_exlusion_bounds"))

class Main():
	def __init__(self):
		# Object we use for transforming between coordinate frames
		self.tf_buf = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

		cylinder_pub = rospy.Publisher("cylinder_filter/cylinder", Pose, queue_size=100)
		rospy.Subscriber("cylinder_detect/cylinder", Pose, self.cylinder)
		rospy.Subscriber("qr_detect/qr_code")

		# Object we use for transforming between coordinate frames
		self.tf_buf = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

		self.qr_pub = rospy.Publisher("cylinder_filter/qr_code", QRCode, queue_size=10)

		self.cylinder_poses = dict()
		self.cylinder_publish = []

		self.cylinder_color = None # Cylinder color buffer

		self.qualifying = False

	def image_callback(self, rgb_data, depth_data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
		except CvBridgeError as e:
			print(e)

		#cv_image = cv_image[140:490, 140:490]

		# Set the dimensions of the image
		self.dims = cv_image.shape

		# Tranform image to grayscale
		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

		# Do histogram equlization
		img = cv2.equalizeHist(gray)

		# Binarize the image
		thresh = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 201, 50)

		cv2.Canny(thresh, 50, 100)

		if self.qualifying == True:
			self.process_detect_qr(thresh)

	def process_detect_qr(self, cv_image):   
		# Find a QR code in the image
		decodedObjects = pyzbar.decode(cv_image)
		
		if len(decodedObjects) == 1:
			dObject = decodedObjects[0]

			qr_code = QRCode()
			qr_code.data = dObject.data
			self.qr_pub.publish(qr_code)
			self.qualifying = False

			if debug: rospy.loginfo("Found 1 QR code in the image!")
			if debug: rospy.loginfo("Data: {}".format(q_code.data))
		elif len(decodedObjects) > 0:
			if debug: rospy.loginfo("Found more than 1 QR code")

	def cylinder(self, pose):
		# Filter the cylinder and decide if we accept it
		is_added = False
		for old_pose in self.cylinder_poses.keys():
			if self.in_cylinder_grouping_bounds(old_pose, pose):
				is_added = True
				self.cylinder_poses[old_pose].append(pose)
				rospy.loginfo("{} - {}".format(len(self.cylinder_poses[old_pose]), cylinder_required_cylinders))
				if len(self.cylinder_poses[old_pose]) >= cylinder_required_cylinders:
					avg_pose = self.avg_pose(self.cylinder_poses[old_pose])
					if not self.in_cylinder_publish(avg_pose):
						self.cylinder_publish.append(avg_pose)
						self.publish_cylinder(avg_pose)
						return avg_pose
			break

		if not is_added:
			self.cylinder_poses[pose] = []

		return None

	def in_cylinder_grouping_bounds(self, old_pose, new_pose):
		return abs(old_pose.position.x - new_pose.position.x) <= cylinder_grouping_tolerance and \
				abs(old_pose.position.y - new_pose.position.y) <= cylinder_grouping_tolerance

	def avg_pose(self, poses):
		x = 0
		y = 0
		z = 0
		for pose in poses:
			x = x + pose.position.x
			y = y + pose.position.y
			z = z + pose.position.z

		pose = Pose()
		pose.position.x = x / len(poses)
		pose.position.y = y / len(poses)
		pose.position.z = z / len(poses)

		return pose

	def in_cylinder_publish(self, old_pose):
		for new_pose in self.cylinder_publish:
			if abs(old_pose.position.x - new_pose.position.x) <= cylinder_exlusion_bounds and \
				abs(old_pose.position.y - new_pose.position.y) <= cylinder_exlusion_bounds:
				return True

		return False

	def publish_cylinder(self, pose):
		self.qualifying = True

		cylinder = Cylinder()
		cylinder.curr_pose = self.get_curr_pose()
		cylinder.cylinder_pose = pose

		cylinder_pub.publish(cylinder)

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