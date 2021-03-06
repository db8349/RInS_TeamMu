#!/usr/bin/env python
import roslib
roslib.load_manifest('robot')
import rospy
import sensor_msgs.msg
import message_filters
import tf2_ros
from geometry_msgs.msg import Point, Twist


import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from geometry_msgs.msg import Twist
PI = 3.1415926535897

from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

from geometry_msgs.msg import Pose

import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3, Quaternion

import math
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import OccupancyGrid

debug = False

def init_explore_points():
	explore_points = []
	
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = 0.775
	goal.target_pose.pose.position.y = -1.71
	goal.target_pose.pose.orientation.w = 1.0
	explore_points.append(goal)

	
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = 0.963
	goal.target_pose.pose.position.y = -0.355
	goal.target_pose.pose.orientation.w = 1.0
	explore_points.append(goal)

	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = 0.81
	goal.target_pose.pose.position.y = 0.584
	goal.target_pose.pose.orientation.w = 1.0
	explore_points.append(goal)

	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = 0.983
	goal.target_pose.pose.position.y = 1.65
	goal.target_pose.pose.orientation.w = 1.0
	explore_points.append(goal)

	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = -0.701
	goal.target_pose.pose.position.y = 1.67
	goal.target_pose.pose.orientation.w = 1.0
	explore_points.append(goal)
	
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = -0.436
	goal.target_pose.pose.position.y = 0.558
	goal.target_pose.pose.orientation.w = 1.0
	explore_points.append(goal)

	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = -0.683
	goal.target_pose.pose.position.y = -0.338
	goal.target_pose.pose.orientation.w = 1.0
	explore_points.append(goal)

	return explore_points

def init_explore_points_debug():
	explore_points = []
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = 2.39
	goal.target_pose.pose.position.y = 1.51
	goal.target_pose.pose.orientation.w = 1.0
	explore_points.append(goal)

	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = 2.37
	goal.target_pose.pose.position.y = 3.83
	goal.target_pose.pose.orientation.w = 1.0
	explore_points.append(goal)

	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = 3.61
	goal.target_pose.pose.position.y = 2.67
	goal.target_pose.pose.orientation.w = 1.0
	explore_points.append(goal)

	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = 4.79
	goal.target_pose.pose.position.y = 1.98
	goal.target_pose.pose.orientation.w = 1.0
	explore_points.append(goal)

	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = 3.65
	goal.target_pose.pose.position.y = 1.42
	goal.target_pose.pose.orientation.w = 1.0
	explore_points.append(goal)

	return explore_points


class Main():
	def __init__(self):
		global debug

		self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

		self.vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)

		if debug:
			self.explore_points = init_explore_points_debug()
		else:
			self.explore_points = init_explore_points()
		self.curr_explore_point = 0
		self.ring_poses = []
		self.ring_counter = 0
		self.ring_heap = dict()

		self.ring_graber_pub = rospy.Publisher('main_grab_3d_ring', Pose, queue_size=10)
		self.paused = False

		self.map_subscriber = rospy.Subscriber('map', OccupancyGrid, self.map_callback)
		self.map_data = None

		self.marker_array = MarkerArray()
		self.marker_num = 1
		self.markers_pub = rospy.Publisher('markers', MarkerArray, queue_size=10000)
	
	def init(self):
		while(not self.ac.wait_for_server(rospy.Duration.from_sec(2.0))):
			rospy.loginfo("Waiting for the move_base action server to come up")
			if rospy.is_shutdown():
				return

	def explore(self):
		goal_state = GoalStatus.LOST

		while self.curr_explore_point < len(self.explore_points) and not rospy.is_shutdown():
			if len(self.ring_poses) > 0:
				# Wait till the ring graber is done
				while self.paused:
					rospy.sleep(0.01)

				if self.ring_counter > 3:
					rospy.loginfo("Found {} rings, ending exploration".format(self.ring_counter + 1))
					break

			explore_string = "Exploring point number {}".format(self.curr_explore_point)
			rospy.loginfo(explore_string)

			self.got_to(self.explore_points[self.curr_explore_point])
			self.rotate(15, 360)

			self.curr_explore_point = (self.curr_explore_point + 1) % len(self.explore_points)

	def call_ring_graber(self, ring_pose):
		rospy.loginfo("Passing ring pose to ring graber")
		self.ring_graber_pub.publish(ring_pose)
		self.ring_counter = self.ring_counter + 1
		self.paused = True

	def ring_graber_done(self, pose):
		rospy.loginfo("Ring graber is done!")
		self.paused = False
		del self.ring_poses[:]

	def got_to(self, goal):
		self.stop()

		self.ac.send_goal(goal)

		goal_state = GoalStatus.LOST
		while (not goal_state == GoalStatus.SUCCEEDED) and len(self.ring_poses) == 0:
			self.ac.wait_for_result(rospy.Duration(0.01))

			goal_state = self.ac.get_state()
			#Possible States Are: PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST.

			if goal_state == GoalStatus.SUCCEEDED:
				rospy.loginfo("The point was reached!")

			if rospy.is_shutdown():
				return

	def rotate(self, speed, angle):
		vel_msg = Twist()

		angular_speed = speed*2*PI/360
		relative_angle = angle*2*PI/360

		vel_msg.linear.x=0
		vel_msg.linear.y=0
  		vel_msg.linear.z=0
  		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.angular.z = abs(angular_speed)

		t0 = rospy.Time.now().to_sec()
		current_angle = 0

		while(current_angle < relative_angle) and len(self.ring_poses) == 0:
		    self.vel_pub.publish(vel_msg)
		    t1 = rospy.Time.now().to_sec()
		    current_angle = angular_speed*(t1-t0)

		    if rospy.is_shutdown():
				return


		#Forcing our robot to stop
		vel_msg.angular.z = 0
		self.vel_pub.publish(vel_msg)

	def map_callback(self, data):
		rospy.loginfo("Got the map")
		self.map_data = data

	def from_map_to_image(self, x, y):
		cell_x = int((x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
		cell_y = int((y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)
		return (cell_x, cell_y)

	def from_image_to_map(self, cell_x, cell_y):
		x = cell_x * self.map_data.info.resolution + self.map_data.info.origin.position.x
		y = cell_y * self.map_data.info.resolution + self.map_data.info.origin.position.y
		return (x, y)

	def get_pixel(self, cell_x, cell_y):
		return self.map_data.data[cell_y * self.map_data.info.width + cell_x]

	def add_ring_pose(self, ring_pose):
		rospy.loginfo("Recieved ring pose")
		if math.isnan(ring_pose.position.x) or math.isnan(ring_pose.position.y) or math.isnan(ring_pose.position.z):
			rospy.loginfo("is nan!")
			return None
		#ring_pose = self.filter_ring(ring_pose)
		if ring_pose == None:
			return
		
		self.show_point(ring_pose)
		#publish_ring_pose = ring_pose
		'''
		publish_ring_pose = None
		is_added = False
		for pose in self.ring_heap.keys():
			if self.in_bounds(pose, ring_pose):
				required_rings = 2
				self.ring_heap[pose].append(ring_pose)
				is_added = True
				if len(self.ring_heap[pose]) >= required_rings:
					if not self.is_in_publish_points(ring_pose):
						rospy.loginfo("Pose: {}, {}".format(pose.position.x, pose.position.y))
						rospy.loginfo("Ring pose: {}, {}".format(ring_pose.position.x, ring_pose.position.y))
						publish_ring_pose = ring_pose
				else:
					rospy.loginfo("Required rings len: {}".format(len(self.ring_heap[pose])))
				break

		if publish_ring_pose is None:
			if not is_added:
				rospy.loginfo("Registered new ring pose")
				self.ring_heap[ring_pose] = [ring_pose]
			return
		'''

		#rospy.loginfo("Appending ring pose!")
		self.ring_poses.append(ring_pose)
		found_string = "Found ring number {}".format(self.ring_counter)
		rospy.loginfo(found_string)

		self.call_ring_graber(ring_pose)
		self.paused = True

	def filter_ring(self, ring_pose):
		while self.map_data == None:
			rospy.sleep(0.01)

		if math.isnan(ring_pose.position.x) or math.isnan(ring_pose.position.y) or math.isnan(ring_pose.position.z):
			return None

		
		offset = 0.2
		ring_cell = self.from_map_to_image(ring_pose.position.x, ring_pose.position.y)
		possible_points_x = [Pose(Point(ring_pose.position.x + offset, ring_pose.position.y, ring_pose.position.z), ring_pose.orientation),
							Pose(Point(ring_pose.position.x - offset, ring_pose.position.y, ring_pose.position.z), ring_pose.orientation)]
		possible_points_y = [Pose(Point(ring_pose.position.x, ring_pose.position.y + offset, ring_pose.position.z), ring_pose.orientation),
							Pose(Point(ring_pose.position.x, ring_pose.position.y - offset, ring_pose.position.z), ring_pose.orientation)]

		x_coll = False
		i = 0
		for point in possible_points_x:
			point_cell = self.from_map_to_image(point.position.x, point.position.y)
			if i == 0:
				cell = (ring_cell[0], ring_cell[1])
				while cell[0] <= point_cell[0]:
					pixel_value = self.get_pixel(int(cell[0]), int(cell[1]))
					if pixel_value == 100:
						x_coll = True
					cell = [cell[0] + 1, cell[1]]
			else:
				cell = (ring_cell[0], ring_cell[1])
				while cell[0] >= point_cell[0]:
					pixel_value = self.get_pixel(int(cell[0]), int(cell[1]))
					if pixel_value == 100:
						x_coll = True
					cell = [cell[0] - 1, cell[1]]
			i = i + 1

		y_coll = False
		i = 0
		for point in possible_points_y:
			point_cell = self.from_map_to_image(point.position.x, point.position.y)
			if i == 0:
				cell = (ring_cell[0], ring_cell[1])
				while cell[1] <= point_cell[1]:
					pixel_value = self.get_pixel(int(cell[0]), int(cell[1]))
					if pixel_value == 100:
						y_coll = True
					cell = [cell[0], cell[1] + 1]
			else:
				cell = (ring_cell[0], ring_cell[1])
				while cell[1] >= point_cell[1]:
					pixel_value = self.get_pixel(int(cell[0]), int(cell[1]))
					if pixel_value == 100:
						y_coll = True
					cell = [cell[0], cell[1] - 1]
			i = i + 1

		if x_coll or y_coll:
			return None

		ring_cell = self.from_map_to_image(ring_pose.position.x, ring_pose.position.y)
		pixel_value = self.get_pixel(int(ring_cell[0]), int(ring_cell[1]))
		if (pixel_value != 100):
			#rospy.loginfo("Showing {}, {}".format(ring_pose.position.x, ring_pose.position.y))
			#self.show_point(ring_pose)
			return ring_pose

		return None

	def in_bounds(self, old_pose, new_pose):
		tolerance = 0.1
		return abs(old_pose.position.x - new_pose.position.x) <= tolerance and \
				abs(old_pose.position.y - new_pose.position.y) <= tolerance

	def is_in_publish_points(self, old_pose):
		bounds = 0.8
		for new_pose in self.ring_poses:
			if abs(old_pose.position.x - new_pose.position.x) <= bounds and \
				abs(old_pose.position.y - new_pose.position.y) <= bounds:
				return True

		return False

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

	def stop(self):
		self.ac.cancel_goal()


if __name__ == '__main__':

		rospy.init_node('main', anonymous=False)

		if debug:
			rospy.loginfo("main DEBUG mode")

		m = Main()

		rospy.Subscriber("grab_3d_ring", Pose, m.add_ring_pose)
		rospy.loginfo("Subscriber to grab_3d_ring")
		rospy.Subscriber("done", Pose, m.ring_graber_done)
		m.init()

		#m.explore()
		rospy.spin()
		m.stop()
		rospy.loginfo("All tasks finished, I'm going to bed")