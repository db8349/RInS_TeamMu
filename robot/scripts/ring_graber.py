#!/usr/bin/env python
import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import PointStamped, Vector3, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import message_filters
from nav_msgs.msg import OccupancyGrid

import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from geometry_msgs.msg import Twist
PI = 3.1415926535897

import tf.transformations as tr

class Main():
	def __init__(self):
		global PI

		self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

		self.vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)

		self.tf2_buffer = tf2_ros.Buffer()
		self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)

		self.marker_array = MarkerArray()
		self.marker_num = 1
		self.markers_pub = rospy.Publisher('markers', MarkerArray, queue_size=10000)

		self.map_subscriber = rospy.Subscriber('map', OccupancyGrid, self.map_callback)
		self.map_data = None

	def pickup(self, ring_pos):
		rospy.loginfo("Got ring position ({}, {})".format(ring_pos.position.x, ring_pos.position.y))
		self.show_point(ring_pos)

		# Get current position
		trans = self.get_trans()
		qua = (trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w)
		yaw = tr.euler_from_quaternion(qua)[2]
		d = yaw * 180/PI
		rospy.loginfo(d)

		approach = self.get_possible_approach(ring_pos)
		if approach == None:
			return
		self.got_to(approach)
		self.move_forward(0.5, 2)



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

	def get_possible_approach(self, ring_pos):
		rate = rospy.Rate(0.01)
		while self.map_data == None:
			rate.sleep()

		offset = 0.5
		ring_cell = self.from_map_to_image(ring_pos.position.x, ring_pos.position.y)
		possible_points_x = [(ring_pos.position.x + offset, ring_pos.position.y),
							(ring_pos.position.x - offset, ring_pos.position.y)]
		possible_points_y = [(ring_pos.position.x, ring_pos.position.y + offset),
							(ring_pos.position.x, ring_pos.position.y - offset)]

		x_axis_available = True
		i = 0
		for point in possible_points_x:
			point_cell = self.from_map_to_image(point[0], point[1])
			if i == 0:
				cell = (ring_cell[0], ring_cell[1])
				while cell[0] <= point_cell[0]:
					pixel_value = self.get_pixel(int(cell[0]), int(cell[1]))
					if pixel_value == 100:
						x_axis_available = False
					cell = [cell[0] + 1, cell[1]]
			else:
				cell = (ring_cell[0], ring_cell[1])
				while cell[0] >= point_cell[0]:
					pixel_value = self.get_pixel(int(cell[0]), int(cell[1]))
					if pixel_value == 100:
						x_axis_available = False
					cell = [cell[0] - 1, cell[1]]
			i = i + 1

		y_axis_available = True
		i = 0
		for point in possible_points_y:
			point_cell = self.from_map_to_image(point[0], point[1])
			if i == 0:
				cell = (ring_cell[0], ring_cell[1])
				while cell[1] <= point_cell[1]:
					pixel_value = self.get_pixel(int(cell[0]), int(cell[1]))
					if pixel_value == 100:
						y_axis_available = False
					cell = [cell[0], cell[1] + 1]
			else:
				cell = (ring_cell[0], ring_cell[1])
				while cell[1] >= point_cell[1]:
					pixel_value = self.get_pixel(int(cell[0]), int(cell[1]))
					if pixel_value == 100:
						y_axis_available = False
					cell = [cell[0], cell[1] - 1]
			i = i + 1
		
		offset_ring_pos = Pose(Point(ring_pos.position.x, ring_pos.position.y, ring_pos.position.z), ring_pos.orientation)
		point_color = ColorRGBA(1, 0, 0, 1)
		offset_ring_pos.position.x = offset_ring_pos.position.x + offset 
		self.show_point(offset_ring_pos, point_color)
		offset_ring_pos.position.x = offset_ring_pos.position.x - 2*offset
		self.show_point(offset_ring_pos, point_color)
		offset_ring_pos.position.x = offset_ring_pos.position.x + offset
		offset_ring_pos.position.y = offset_ring_pos.position.y + offset 
		self.show_point(offset_ring_pos, point_color)
		offset_ring_pos.position.y = offset_ring_pos.position.y - 2*offset
		self.show_point(offset_ring_pos, point_color)
		offset_ring_pos.position.y = offset_ring_pos.position.y + offset
		
		if not x_axis_available and not y_axis_available:
			rospy.loginfo("x and y axis not available!")
			return None
		elif x_axis_available and y_axis_available:
			rospy.loginfo("x and y axis IS available!!!")
			return None
		if x_axis_available:
			rospy.loginfo("x axis is available")
			return possible_points_x[0]

		else:
			rospy.loginfo("y axis is available")
			return possible_points_y[0]

	def get_trans(self):
		trans = None
		try:
			while trans == None:
				trans = self.tf2_buffer.lookup_transform('map', 'base_link', rospy.Time(0))
		except Exception as e:
			print e
			return None

		return trans

	def got_to(self, point):
		base_goal = MoveBaseGoal()
		base_goal.target_pose.header.frame_id = "map"
		base_goal.target_pose.header.stamp = rospy.Time.now()
		base_goal.target_pose.pose.position.x = point[0]
		base_goal.target_pose.pose.position.y = point[0]
		base_goal.target_pose.pose.orientation.w = 1.0

		self.stop()

		self.ac.send_goal(base_goal)

		goal_state = GoalStatus.LOST
		while (not goal_state == GoalStatus.SUCCEEDED):
			self.ac.wait_for_result(rospy.Duration(0.01))

			goal_state = self.ac.get_state()
			#Possible States Are: PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST.

			if goal_state == GoalStatus.SUCCEEDED:
				rospy.loginfo("The point was reached!")

			if rospy.is_shutdown():
				return

	def move_forward(self, speed, distance):
		vel_msg = Twist()

		vel_msg.linear.x=speed
		vel_msg.linear.y=0
  		vel_msg.linear.z=0
  		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.angular.z = 0

		t0 = rospy.Time.now().to_sec()
		current_distance = 0

		while(current_distance < distance):
		    self.vel_pub.publish(vel_msg)
		    t1 = rospy.Time.now().to_sec()
		    current_distance = speed*(t1-t0)

		    if rospy.is_shutdown():
				return

		vel_msg.linear.x = 0
		self.vel_pub.publish(vel_msg)

	def show_point(self, pose, color=ColorRGBA(0, 1, 0, 1)):
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

	def map_callback(self, data):
		print("Got the map")
		self.map_data = data


if __name__ == '__main__':

		rospy.init_node('ring_graber', anonymous=False)
		try:
			m = Main()

			rospy.Subscriber("grab_3d_ring", Pose, m.pickup)

			rospy.spin()
		except rospy.ROSInterruptException:
			pass
