#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
import tf2_geometry_msgs
import tf2_ros
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters
from geometry_msgs.msg import PointStamped, Vector3, Pose
from robot.msg import Circle

from visualization_msgs.msg import Marker, MarkerArray

rospy.init_node('circle_sense', anonymous=True)

debug = rospy.get_param("/debug")
circle_required_circles = rospy.get_param("~circle_required_circles")
circle_grouping_tolerance = rospy.get_param("~circle_grouping_tolerance")
circle_exlusion_bounds = rospy.get_param("~circle_exlusion_bounds")
print("{}, {}, {}".format(circle_required_circles, circle_grouping_tolerance, circle_exlusion_bounds))

class CircleSense:
	def __init__(self):
		# An object we use for converting images between ROS format and OpenCV format
		self.bridge = CvBridge()

		# Subscribe to the image and depth topic
		self.image_sub = message_filters.Subscriber(rospy.get_param("/image_topic"), Image)
		self.depth_sub = message_filters.Subscriber(rospy.get_param("/depth_topic"), Image)
		self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], 100, 2)
		self.ts.registerCallback(self.image_callback)

		# Object we use for transforming between coordinate frames
		self.tf_buf = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

		self.circle_pub = rospy.Publisher("circle_detect/circle", Circle, queue_size=10)

		# Stores circle positions used in filtering
		self.circle_poses = dict()
		self.circle_publish = []

		self.marker_array = MarkerArray()
		self.marker_num = 1
		self.markers_pub = rospy.Publisher('markers', MarkerArray, queue_size=10000)

	def image_callback(self, rgb_data, depth_data):
		# Stop processing if we are in cylinder stage
		if self.cylinder_stage == True:
			return

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

		# Extract contours
		im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

		# Fit elipses to all extracted contours
		elps = []
		for cnt in contours:
			#     print cnt
			#     print cnt.shape
			if cnt.shape[0] >= 100 and cnt.shape[0] < 800:
				ellipse = cv2.fitEllipse(cnt)
				elps.append(ellipse)


		# Find two elipses with same centers
		candidates = []
		for n in range(len(elps)):
			for m in range(n + 1, len(elps)):
				e1 = elps[n]
				e2 = elps[m]
				dist = np.sqrt(((e1[0][0] - e2[0][0]) ** 2 + (e1[0][1] - e2[0][1]) ** 2))
				#             print dist
				if dist < 5:
					candidates.append((e1,e2))

		# If we detect some circles process them
		if len(candidates) > 0:
			self.processCirclePose(thresh, depth_data, candidates)

	def processCirclePose(self, cv_image, depth_data, candidates):
		depth_img = depth_data

		# Extract the depth from the depth image
		for c in candidates:

			# the centers of the ellipses
			e1 = c[0]
			e2 = c[1]

			# drawing the ellipses on the image
			cv2.ellipse(cv_image, e1, (0, 255, 0), 2)
			cv2.ellipse(cv_image, e2, (0, 255, 0), 2)

			size = (e1[1][0]+e1[1][1])/2
			center = (e1[0][1], e1[0][0])

			x1 = int(center[0] - size / 2)
			x2 = int(center[0] + size / 2)
			x_min = x1 if x1>0 else 0
			x_max = x2 if x2<cv_image.shape[0] else cv_image.shape[0]

			y1 = int(center[1] - size / 2)
			y2 = int(center[1] + size / 2)
			y_min = y1 if y1 > 0 else 0
			y_max = y2 if y2 < cv_image.shape[1] else cv_image.shape[1]

			depth_image = self.bridge.imgmsg_to_cv2(depth_img, "16UC1")

			circle_pose = self.extract_circle_pos(e1, float(np.mean(depth_image[x_min:x_max,y_min:y_max]))/1000.0)
			if circle_pose != None:
				# TODO: Detect circle color
				color = ""
				circle = Circle()
				circle.curr_pose = self.get_curr_pose()
				circle.circle_pose = circle_pose
				circle.color = color
				self.circle_pub.publish(circle)
				if debug: rospy.loginfo("Found a circle ({}, {}) - {}".format(circle.circle_pose.position.x, circle.circle_pose.position.y, circle.color))

				return circle_pose

			return None

	def extract_circle_pos(self, e, dist):
		# Calculate the position of the detected ellipse

		k_f = 525 # kinect focal length in pixels

		elipse_x = self.dims[1] / 2 - e[0][0]
		elipse_y = self.dims[0] / 2 - e[0][1]

		angle_to_target = np.arctan2(elipse_x,k_f)

		# Get the angles in the base_link relative coordinate system
		x,y = dist*np.cos(angle_to_target), dist*np.sin(angle_to_target)

		# Define a stamped message for transformation - in the "camera rgb frame"
		point_s = PointStamped()
		point_s.point.x = -y
		point_s.point.y = 0
		point_s.point.z = x
		point_s.header.frame_id = "camera_rgb_optical_frame"
		point_s.header.stamp = rospy.Time(0)

		# Get the point in the "map" coordinate system
		try:
			point_world = self.tf_buf.transform(point_s, "map")
		except Exception as e:
			return None

		# Create a Pose object with the same position
		pose = Pose()
		pose.position.x = point_world.point.x
		pose.position.y = point_world.point.y
		pose.position.z = point_world.point.z

		# Filter the circle and decide if we accept it
		is_added = False
		for old_pose in self.circle_poses.keys():
			if self.in_circle_grouping_bounds(old_pose, pose):
				is_added = True
				self.circle_poses[old_pose].append(pose)
				rospy.loginfo("{} - {}".format(len(self.circle_poses[old_pose]), circle_required_circles))
				if len(self.circle_poses[old_pose]) >= circle_required_circles:
					avg_pose = self.avg_pose(self.circle_poses[old_pose])
					if not self.in_circle_publish(avg_pose):
						self.circle_publish.append(avg_pose)
						return avg_pose
			break

		if not is_added:
			self.circle_poses[pose] = []

		return None

	def in_circle_grouping_bounds(self, old_pose, new_pose):
		return abs(old_pose.position.x - new_pose.position.x) <= circle_grouping_tolerance and \
				abs(old_pose.position.y - new_pose.position.y) <= circle_grouping_tolerance

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

	def in_circle_publish(self, old_pose):
		for new_pose in self.circle_publish:
			if abs(old_pose.position.x - new_pose.position.x) <= circle_exlusion_bounds and \
				abs(old_pose.position.y - new_pose.position.y) <= circle_exlusion_bounds:
				return True

		return False

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

def main(args):
	if debug:
		rospy.loginfo("Circle sense is in DEBUG mode")

	ring_rectifier = CircleSense()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	main(sys.argv)
