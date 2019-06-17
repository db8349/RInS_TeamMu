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
from geometry_msgs.msg import Point, PointStamped, Vector3, Pose, Quaternion
from robot.msg import Circle
from std_msgs.msg import ColorRGBA, String
from nav_msgs.msg import OccupancyGrid

from visualization_msgs.msg import Marker, MarkerArray
import math

def pose_distance(pose1, pose2):
	return math.sqrt((pose1.position.x - pose2.position.x)**2 + (pose1.position.y - pose2.position.y)**2)

rospy.init_node('circle_detect', anonymous=True)

debug = rospy.get_param("/debug")
circle_required_circles = rospy.get_param("~circle_required_circles")
circle_grouping_tolerance = rospy.get_param("~circle_grouping_tolerance")
circle_exlusion_bounds = rospy.get_param("~circle_exlusion_bounds")

class CircleSense:
	def __init__(self):
		# An object we use for converting images between ROS format and OpenCV format
		self.bridge = CvBridge()

		rospy.Subscriber("circle_detect/cylinder_stage", String, self.set_cylinder_stage)
		self.cylinder_stage = False

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

		self.map_subscriber = rospy.Subscriber('map', OccupancyGrid, self.map_callback)
		self.map_data = None
		self.costmap_subscriber = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.costmap_callback)
		self.costmap_data = None

	def set_cylinder_stage(self, data):
		rospy.loginfo("Setting cylinder stage to True!")
		self.cylinder_stage = True
		rospy.loginfo("Unsubscribing to the image and depth callbacks")
		# Unsubscribe to the image and depth topic
		self.image_sub.sub.unregister()
		self.depth_sub.sub.unregister()

	def map_callback(self, data):
		#rospy.loginfo("Got the map")
		self.map_data = data

	def costmap_callback(self, data):
		#rospy.loginfo("Got the costmap {}, {}".format(data.info.resolution, data.info.width))
		self.costmap_data = data

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

		#rospy.loginfo("Processing is done! found {} candidates for rings".format(len(candidates)))

		# If we detect some circles process them
		if len(candidates) > 0:
			self.processCirclePose(cv_image, depth_data, candidates)

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
				self.show_point(circle_pose)
				color = self.get_circle_color(cv_image)
				circle = Circle()
				circle.pose = circle_pose
				circle.color = color
				length = 7
				ignore_center_length = 2
				circle.approaches = self.cross_approach(circle, length, ignore_center_length)
				self.circle_pub.publish(circle)
				if debug: rospy.loginfo("Found a circle ({}, {}) -- {}".format(circle.pose.position.x, circle.pose.position.y, circle.color))

				return circle_pose

			return None

	def get_circle_color(self, cv_image):

		image = cv_image # todo cropping

		colors = ["red", "red", "blue", "green", "yellow", "black"]

		detectBoundary = 500
		boundaries = [
        	([0, 30, 30], [12, 255, 255]),
			([160, 30, 30], [180, 255, 255]),
			([90, 30, 30], [130, 255, 255]),
			([36, 30, 30], [66, 255, 255]),
			([17, 70, 70], [36, 255, 255])
    	]

		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

		i = 0
		for (lower, upper) in boundaries:
			lower = np.array(lower, dtype = "uint8")
			upper = np.array(upper, dtype = "uint8")

			mask = cv2.inRange(hsv, lower, upper)
			output = cv2.bitwise_and(image, image, mask = mask)
			countNonZero = np.count_nonzero(output)
			print(colors[i], " ", countNonZero)

			#if countNonZero > detectBoundary:
			#	break
			i += 1

		return colors[i]



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
		rospy.loginfo(len(self.circle_poses))
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
			rospy.loginfo("Adding new pose to circle_poses")
			self.circle_poses[pose] = []

		return None

	def in_circle_grouping_bounds(self, old_pose, new_pose):
		dist = pose_distance(old_pose, new_pose)
		rospy.loginfo("{} <= {}".format(dist, circle_grouping_tolerance))
		return dist <= circle_grouping_tolerance

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
			if pose_distance(old_pose, new_pose) <= circle_exlusion_bounds:
				return True

		return False

	def cross_approach(self, circle, length, ignore_center_length):
		available_poses = []
		curr_pose = self.get_curr_pose()
		circle_dist = pose_distance(curr_pose, circle.pose)

		yaws = [math.radians(0), math.radians(30), math.radians(60), math.radians(90), math.radians(120), math.radians(150),
				 math.radians(180), math.radians(210), math.radians(240), math.radians(270), math.radians(300)]
		for yaw in yaws:
			i = 0
			cell = self.from_map_to_image(circle.pose.position.x, circle.pose.position.y)
			cell = (cell[0] + math.cos(yaw)*ignore_center_length, cell[1] + math.sin(yaw)*ignore_center_length)
			while i < length:
				pixel_value = self.get_pixel(self.map_data, int(cell[0]), int(cell[1]))
				if pixel_value == 100:
					break

				cell = (cell[0] + math.cos(yaw), cell[1] + math.sin(yaw))
				i = i + 1

			pixel_value = self.get_pixel(self.costmap_data, int(cell[0]), int(cell[1]))
			if pixel_value > 50:
				#print("Approach {} is in costmap with pixel value {}".format(yaw, pixel_value))
				#xy = self.from_image_to_map(cell[0], cell[1])
				#pose = Pose(Point(xy[0], xy[1], 0), Quaternion())
				#self.show_point(pose, ColorRGBA(0, 0, 1, 1))
				continue

			if i == length:
				xy = self.from_image_to_map(cell[0], cell[1])
				pose = Pose(Point(xy[0], xy[1], 0), Quaternion())
				dist = pose_distance(curr_pose, pose)
				if dist < circle_dist:
					available_poses.append(pose)

		return available_poses

	def from_map_to_image(self, x, y):
		cell_x = int((x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
		cell_y = int((y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)
		return (cell_x, cell_y)

	def from_image_to_map(self, cell_x, cell_y):
		x = cell_x * self.map_data.info.resolution + self.map_data.info.origin.position.x
		y = cell_y * self.map_data.info.resolution + self.map_data.info.origin.position.y
		return (x, y)

	def get_pixel(self, map_data, cell_x, cell_y):
		return map_data.data[cell_y * map_data.info.width + cell_x]

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
		rospy.loginfo("circle_detect is in DEBUG mode")

	ring_rectifier = CircleSense()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	main(sys.argv)
