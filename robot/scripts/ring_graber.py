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

from geometry_msgs.msg import Twist, Quaternion

PI = 3.14159265358979323846264338327950288419716939937510582097494459230781640628620899862803482534211706798214808651328230664709384460955058223172535940812848111745028410270193852110555964462294895493038196442881097566593344612847564823378678316527120190914

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
		curr_pose = Pose(trans.transform.translation, qua)
		yaw = tr.euler_from_quaternion(qua)[2]
		d = yaw * 180/PI
		rospy.loginfo(d)

		approach = self.get_possible_approach(curr_pose, ring_pos)
		if approach == None:
			return
		self.got_to(approach)
		self.move_forward(0.25, 1)

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

	def get_possible_approach(self, curr_pose, ring_pos):
		rate = rospy.Rate(0.01)
		while self.map_data == None:
			rate.sleep()

		offset = 0.5
		ring_cell = self.from_map_to_image(ring_pos.position.x, ring_pos.position.y)
		possible_points_x = [Pose(Point(ring_pos.position.x + offset, ring_pos.position.y, ring_pos.position.z), ring_pos.orientation),
							Pose(Point(ring_pos.position.x - offset, ring_pos.position.y, ring_pos.position.z), ring_pos.orientation)]
		possible_points_y = [Pose(Point(ring_pos.position.x, ring_pos.position.y + offset, ring_pos.position.z), ring_pos.orientation),
							Pose(Point(ring_pos.position.x, ring_pos.position.y - offset, ring_pos.position.z), ring_pos.orientation)]

		x_axis_available = True
		x_coll = -1
		i = 0
		for point in possible_points_x:
			point_cell = self.from_map_to_image(point.position.x, point.position.y)
			if i == 0:
				cell = (ring_cell[0], ring_cell[1])
				while cell[0] <= point_cell[0]:
					pixel_value = self.get_pixel(int(cell[0]), int(cell[1]))
					if pixel_value == 100:
						x_axis_available = False
						x_coll = 0
					cell = [cell[0] + 1, cell[1]]
			else:
				cell = (ring_cell[0], ring_cell[1])
				while cell[0] >= point_cell[0]:
					pixel_value = self.get_pixel(int(cell[0]), int(cell[1]))
					if pixel_value == 100:
						x_axis_available = False
						x_coll = i
					cell = [cell[0] - 1, cell[1]]
			i = i + 1

		y_axis_available = True
		y_coll = -1
		i = 0
		for point in possible_points_y:
			point_cell = self.from_map_to_image(point.position.x, point.position.y)
			if i == 0:
				cell = (ring_cell[0], ring_cell[1])
				while cell[1] <= point_cell[1]:
					pixel_value = self.get_pixel(int(cell[0]), int(cell[1]))
					if pixel_value == 100:
						y_axis_available = False
						y_coll = 0
					cell = [cell[0], cell[1] + 1]
			else:
				cell = (ring_cell[0], ring_cell[1])
				while cell[1] >= point_cell[1]:
					pixel_value = self.get_pixel(int(cell[0]), int(cell[1]))
					if pixel_value == 100:
						y_axis_available = False
						y_coll = i
					cell = [cell[0], cell[1] - 1]
			i = i + 1
		
		pose = None
		if not x_axis_available and not y_axis_available:
			rospy.loginfo("x and y axis not available!")
		elif x_axis_available and y_axis_available:
			rospy.loginfo("x and y axis IS available!!!")
		if x_axis_available:
			rospy.loginfo("x axis is available")
			self.show_point(possible_points_x[0], ColorRGBA(1, 0, 0, 1))
			self.show_point(possible_points_x[1], ColorRGBA(1, 0, 0, 1))
			pose = possible_points_x[y_coll]

			deg = 0
			if pose == possible_points_x[0]:
				deg = 180
			q = tr.quaternion_from_euler(0, 0, deg * PI/180)
			pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
		else:
			rospy.loginfo("y axis is available")
			self.show_point(possible_points_y[0], ColorRGBA(1, 0, 0, 1))
			self.show_point(possible_points_y[1], ColorRGBA(1, 0, 0, 1))
			pose = possible_points_y[x_coll]

			deg = 90
			if pose == possible_points_y[0]:
				deg = -90
			q = tr.quaternion_from_euler(0, 0, deg * PI/180)
			pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

		return pose

	def find_nearest_point(self, pose, poses):
		nearest_dist = self.get_dist(pose, poses[0])
		nearest = poses[0]
		for p in poses:
			dist = self.get_dist(pose, p)
			if dist < nearest_dist:
				nearest = p
				nearest_dist = dist

		return nearest

	def get_dist(self, pose1, pose2):
		return (pose1.position.x - pose2.position.x)**2 + (pose1.position.y - pose2.position.y)**2

	def get_trans(self):
		trans = None
		try:
			while trans == None:
				trans = self.tf2_buffer.lookup_transform('map', 'base_link', rospy.Time(0))
		except Exception as e:
			print e
			return None

		return trans

	def got_to(self, pose):
		base_goal = MoveBaseGoal()
		base_goal.target_pose.header.frame_id = "map"
		base_goal.target_pose.header.stamp = rospy.Time.now()
		base_goal.target_pose.pose.position.x = pose.position.x
		base_goal.target_pose.pose.position.y = pose.position.y
		base_goal.target_pose.pose.orientation = pose.orientation

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

	def stop(self):
		self.ac.cancel_goal()


if __name__ == '__main__':

		rospy.init_node('ring_graber', anonymous=False)
		try:
			m = Main()

			rospy.Subscriber("grab_3d_ring", Pose, m.pickup)

			rospy.spin()
		except rospy.ROSInterruptException:
			pass
