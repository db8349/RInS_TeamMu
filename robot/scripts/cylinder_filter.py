#!/usr/bin/env python
import roslib
roslib.load_manifest('robot')
import rospy

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3, Quaternion, Twist, Pose, PointStamped
from std_msgs.msg import ColorRGBA, String
import math
from nav_msgs.msg import OccupancyGrid

from robot.msg import QRCode, Cylinder, PCLCylinder
import classifier as cs

import tf2_geometry_msgs
import tf2_ros
import numpy as np
import cv2
import colorsys

def pose_distance(pose1, pose2):
	return math.sqrt((pose1.position.x - pose2.position.x)**2 + (pose1.position.y - pose2.position.y)**2)

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
		rospy.Subscriber("cylinder_detect/cylinder", PCLCylinder, self.cylinder)

		self.cylinder_publish = []

		self.map_subscriber = rospy.Subscriber('map', OccupancyGrid, self.map_callback)
		self.map_data = None
		self.costmap_subscriber = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.costmap_callback)
		self.costmap_data = None

		self.marker_array = MarkerArray()
		self.marker_num = 1
		self.markers_pub = rospy.Publisher('markers', MarkerArray, queue_size=10000)

		#rospy.Subscriber("/clicked_point", PointStamped, self.spoffed_point)

	def map_callback(self, data):
		#rospy.loginfo("Got the map {}, {}".format(data.info.resolution, data.info.width))
		self.map_data = data

	def costmap_callback(self, data):
		#rospy.loginfo("Got the costmap {}, {}".format(data.info.resolution, data.info.width))
		self.costmap_data = data

	def cylinder(self, pcl_cylinder):
		if math.isnan(pcl_cylinder.point.point.x):
			return

		pose = Pose(Point(pcl_cylinder.point.point.x, pcl_cylinder.point.point.y, pcl_cylinder.point.point.z), Quaternion())
		# Filter the cylinder and decide if we accept it
		if self.in_cylinder_publish(pose):
			return False

		cylinder = Cylinder()
		cylinder.pose = pose
		cylinder.color = self.get_color(pcl_cylinder.r, pcl_cylinder.g, pcl_cylinder.b)
		if cylinder.color == "unknown": #false positive
			return False

		length = 7
		ignore_center_length = 2
		clear_bound_length = 5
		cylinder.approaches = self.cross_approach(cylinder, length, ignore_center_length)
		rospy.loginfo("New Cylinder: {}, {} --- {}".format(pose.position.x, pose.position.y, cylinder.color))

		self.publish_cylinder(cylinder)


		return True

	def in_cylinder_publish(self, old_pose):
		for new_pose in self.cylinder_publish:
			dist = pose_distance(old_pose, new_pose)
			#print("{} <= {}".format(dist, cylinder_exlusion_bounds))
			if pose_distance(old_pose, new_pose) <= cylinder_exlusion_bounds:
				return True

		return False

	def publish_cylinder(self, cylinder):
		rospy.loginfo("Publishing cylinder: {}".format(cylinder.color))
		self.cylinder_publish.append(cylinder.pose)

		self.cylinder_pub.publish(cylinder)

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

	def get_color(self, arr1, arr2, arr3):
		tmp = []
		for r, g, b in zip(arr1, arr2, arr3):
			tmp.append((r, g, b))
		arr = np.array(tmp)
		colors, count = np.unique(arr.reshape(-1,arr.shape[-1]), axis=0, return_counts=True)
		rgb = colors[count.argmax()]
		#print(rgb)
		img = np.zeros((1,1,3), np.uint8)
		img[0, 0] = (sum(arr1)/len(arr1), sum(arr2)/len(arr2), sum(arr3)/len(arr3))

		boundaries = [
			([0, 30, 30], [12, 255, 255]),
			([160, 30, 30], [180, 255, 255]),
			([90, 30, 30], [130, 255, 255]),
			([36, 30, 30], [66, 255, 255]),
			([17, 70, 70], [36, 255, 255])
		]

		colors = ["red", "red", "blue", "green", "yellow"]

		hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
		print(hsv)

		i = 0
		for (lower, upper) in boundaries:
			lower = np.array(lower, dtype = "uint8")
			upper = np.array(upper, dtype = "uint8")
		
			mask = cv2.inRange(hsv, lower, upper)
			output = cv2.bitwise_and(img, img, mask = mask)
			countNonZero = np.count_nonzero(output)
			#print(colors[i], " ", countNonZero)

			if countNonZero > 0:
				break
			
			i += 1
		if i == 5:
			return "unknown"

		return colors[i]

	def cross_approach(self, cylinder, length, ignore_center_length):
		available_poses = []
		curr_pose = self.get_curr_pose()
		circle_dist = pose_distance(curr_pose, cylinder.pose)

		yaws = [math.radians(45), math.radians(75), math.radians(105), math.radians(135), math.radians(175), math.radians(195), math.radians(225),
				 math.radians(255), math.radians(275), math.radians(305), math.radians(335)]
		for yaw in yaws:
			i = 0
			cell = self.from_map_to_image(cylinder.pose.position.x, cylinder.pose.position.y)
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
				if pose_distance(curr_pose, pose) < circle_dist:
					available_poses.append(pose)

		return available_poses

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

	def spoffed_point(self, point):
		pcl_cylinder = PCLCylinder()
		pcl_cylinder.point = point
		pcl_cylinder.r = [255]
		pcl_cylinder.g = [0]
		pcl_cylinder.b = [0]

		self.cylinder(pcl_cylinder)

if __name__ == '__main__':
	if debug:
		rospy.loginfo("cylinder_filter DEBUG mode")

	main = Main()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		nav_manager.stop()
		print("Shutting down")