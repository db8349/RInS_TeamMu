#!/usr/bin/env python
import roslib
roslib.load_manifest('robot')
import rospy
import sensor_msgs.msg
import message_filters
import tf2_ros

import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3, Quaternion, Twist, Pose
from std_msgs.msg import ColorRGBA, String
import math
from robot.msg import MoveForward, Rotate
import numpy as np
from nav_msgs.msg import OccupancyGrid
import sys

from std_srvs.srv import Empty, EmptyRequest

# Helper methods
def pixel_distance(p1, p2):
	return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

###################################################################################################################################################

rospy.init_node('nav_manager', anonymous=False)

debug = rospy.get_param('/debug')
explore_point_radius = int(rospy.get_param('~explore_point_radius'))
map_array_file = rospy.get_param('~map_array_file')
#if debug:
#	explore_point_radius = int(rospy.get_param('~explore_point_radius_debug'))
#	map_array_file = rospy.get_param('~map_array_file_debug')

class NavManager():
	def __init__(self):
		self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		self.request_queue = []

		self.vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)

		self.marker_array = MarkerArray()
		self.marker_num = 1
		self.markers_pub = rospy.Publisher('markers', MarkerArray, queue_size=10000)

		self.map_subscriber = rospy.Subscriber('map', OccupancyGrid, self.map_callback)
		self.map_data = None

		self.current_explore_point = 0
		self.explore_points = []
		self.stop_operations = False

		rospy.Subscriber("/nav_manager/go_to", Pose, lambda pose : self.request_queue.append((self.go_to, pose)))
		rospy.Subscriber("/nav_manager/move_forward", MoveForward, lambda move_forward : self.request_queue.append((self.move_forward, move_forward)))
		rospy.Subscriber("/nav_manager/rotate", Rotate, lambda rotate : self.request_queue.append((self.rotate, rotate)))
		#rospy.Subscriber("/nav_manager/stop", String, lambda data : self.stop_operations = True)
	
	def init(self):
		while(not self.ac.wait_for_server(rospy.Duration.from_sec(2.0))):
			rospy.loginfo("Waiting for the move_base action server to come up")
			if rospy.is_shutdown():
				return

		while self.map_data == None:
			rospy.sleep(0.01)

		# Setup our explore points
		self.init_explore()

	def init_explore(self):
		rospy.loginfo("Loading map {}".format(map_array_file))

		center_squares = np.loadtxt(map_array_file)

		explore_array = [center_squares[len(center_squares)/2]]
		candidates = []
		while True:
			for center_point in center_squares:
				combined_distance = 0
				valid = True
				# Check if the center point is in the radius of already checked points
				for p in explore_array:
					dist = pixel_distance(center_point, p)
					if dist < explore_point_radius:
						valid = False
						break
					combined_distance += dist

				if not valid:
					continue

				candidates.append((center_point, combined_distance))

			if len(candidates) == 0:
				break

			candidates.sort(key=lambda x: x[1], reverse=True)
			top_candidate = candidates[0][0]

			explore_array.append(top_candidate)
			del candidates[:]

		for p in explore_array:
			rospy.loginfo('## {}, {}'.format(p[1], p[0]))
			# Transform into poses
			p = self.from_image_to_map(p[1], p[0]) # Numpy has convention rows, columns (y, x)
			pose = Pose(Point(p[0], p[1], 0), Quaternion(0, 0, 0, 1))
			rospy.loginfo('{}, {}'.format(pose.position.x, pose.position.y))
			self.explore_points.append(pose)

		rospy.loginfo("Explore points loaded")

	def from_image_to_map(self, cell_x, cell_y):
		x = cell_x * self.map_data.info.resolution + self.map_data.info.origin.position.x
		y = cell_y * self.map_data.info.resolution + self.map_data.info.origin.position.y
		return (x, y)

	def from_map_to_image(self, x, y):
		cell_x = int((x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
		cell_y = int((y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)
		return (cell_x, cell_y)

	def map_callback(self, data):
		rospy.loginfo("Got the map")
		self.map_data = data

	def explore(self):
		rospy.loginfo("Starting exploration")
		self.stop_operations = False
		self.stop()

		rospy.loginfo('{} and {} and {}'.format(self.current_explore_point < len(self.explore_points), not rospy.is_shutdown(), not self.stop_operations))
		while self.current_explore_point < len(self.explore_points) and not rospy.is_shutdown() and not self.stop_operations:
			self.clear_costmaps()

			if len(self.request_queue) > 0:
				self.process_request_queue()

			rospy.loginfo("Exploring point {}".format(self.current_explore_point))
			self.go_to(self.explore_points[self.current_explore_point])
			self.rotate(15, 360)

			self.current_explore_point = (self.current_explore_point + 1) % len(self.explore_points)

		self.stop()

	def go_to(self, pose):
		self.stop()

		rospy.loginfo("Going to ({}, {})".format(pose.position.x, pose.position.y))
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position = pose.position
		goal.target_pose.pose.orientation = pose.orientation

		self.ac.send_goal(goal)

		goal_state = GoalStatus.LOST
		while not goal_state == GoalStatus.SUCCEEDED and not rospy.is_shutdown() and not self.stop_operations:
			if len(self.request_queue) > 0:
				return

			self.ac.wait_for_result(rospy.Duration(0.005))

			goal_state = self.ac.get_state()
			#Possible States Are: PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST.

			if goal_state == GoalStatus.SUCCEEDED:
				if debug: rospy.loginfo("The point was reached!")

	def rotate(self, speed, angle):
		vel_msg = Twist()

		angular_speed = speed*2*math.pi/360
		relative_angle = angle*2*math.pi/360

		vel_msg.linear.x=0
		vel_msg.linear.y=0
		vel_msg.linear.z=0
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.angular.z = abs(angular_speed)

		t0 = rospy.Time.now().to_sec()
		current_angle = 0

		while current_angle < relative_angle and not rospy.is_shutdown():
			if len(self.request_queue) > 0:
				return

			self.vel_pub.publish(vel_msg)
			t1 = rospy.Time.now().to_sec()
			current_angle = angular_speed*(t1-t0)

		#Forcing our robot to stop
		vel_msg.angular.z = 0
		self.vel_pub.publish(vel_msg)
		if debug: rospy.loginfo("Rotation completed!")

	def move_forward(self, move_forward):
		vel_msg = Twist()

		vel_msg.linear.x = move_forward.speed
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.angular.z = 0

		t0 = rospy.Time.now().to_sec()
		current_distance = 0

		while current_distance < move_forward.distance and not rospy.is_shutdown() and not self.stop_operations:
			self.vel_pub.publish(vel_msg)
			t1 = rospy.Time.now().to_sec()
			current_distance = move_forward.speed*(t1-t0)

		# Forcing our robot to stop
		vel_msg.linear.x = 0
		self.vel_pub.publish(vel_msg)
		if debug: rospy.loginfo("MoveForward done!")


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

	def process_request_queue(self):
		i = 0
		while i < len(self.request_queue) and not rospy.is_shutdown():
			rospy.loginfo("Processing request: {}".format(i))
			self.request_queue[i][0](self.request_queue[i][1])
			if self.request_queue[i][0] == self.go_to:
				rospy.sleep(0.5)
				rospy.Publisher('circle_pose/stop_qualifying', String, queue_size=100).publish("")
			i = i + 1

		del self.request_queue[:]

	def clear_costmaps(self):
		rospy.wait_for_service('/move_base/clear_costmaps')
		try:
			clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
			clear_costmaps_service(EmptyRequest())
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def stop(self):
		self.ac.cancel_goal()

if __name__ == '__main__':
	if debug:
		rospy.loginfo("nav_manager DEBUG mode")

	nav_manager = NavManager()
	nav_manager.init()
	nav_manager.clear_costmaps()
	nav_manager.explore()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		nav_manager.stop()
		print("Shutting down")