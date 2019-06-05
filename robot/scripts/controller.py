#!/usr/bin/env python
import roslib
roslib.load_manifest('robot')
import rospy

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3, Quaternion, Twist, Pose
from std_msgs.msg import ColorRGBA, String
import math

from robot.msg import Numbers, Circle, QRCode
import classifier as cs

rospy.init_node('nav_manager', anonymous=False)

debug = rospy.get_param('/debug')

class Main():
	def __init__(self):
		self.marker_array = MarkerArray()
		self.marker_num = 1
		self.markers_pub = rospy.Publisher('markers', MarkerArray, queue_size=10000)

		# Variables that store the latest detected data
		self.qr_data = None
		self.num = None
		self.result = None
		# If this is not None we wait for incoming QR or Numbers data that we then store into correct variables
		self.curr_circle = None

		self.rings = []
		self.cilinders = []

		self.nav_goto_publisher = rospy.Publisher("/nav_manager/go_to", Pose, queue_size=100)

		rospy.Subscriber("circle_sense/numbers", Numbers, self.numbers)
		rospy.Subscriber("circle_sense/qr_code", QRCode, self.qr)
		rospy.Subscriber("circle_sense/circle", Circle, self.circle)

	def qr(self, data):
		self.qr_data = data.split(" ")[1]
		rospy.loginfo("Setting QR data: {}".format(self.qr_data))
		self.curr_circle = None
		
		self.atempt_classify()

	def circle(self, circle):
		self.curr_circle = circle
		#rospy.loginfo("New Circle: {}, {}".format(circle.circle_pose.position.x, circle.circle_pose.position.y))
		self.show_point(circle.circle_pose, ColorRGBA(0, 0, 1, 1))
		circle_approach_pose = self.approach_transform(circle.curr_pose, circle.circle_pose, 0.4)
		#rospy.loginfo("Circle approach: ({}, {})".format(circle_approach_pose.position.x, circle_approach_pose.position.y))
		self.show_point(circle_approach_pose, ColorRGBA(0, 1, 0, 1))
		self.nav_goto_publisher.publish(circle_approach_pose)

	def numbers(self, num):
		print("Pozdravljen svet!")
		rospy.loginfo("Setting Numbers: {}, {}".format(num.first, num.second))
		self.num = num
		self.curr_circle = None

		self.atempt_classify()

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
			self.result = cs.classify(self.qr_data, self.num.first, self.num.second)
			rospy.loginfo("Classification result: {}".format(self.result))

if __name__ == '__main__':
	if debug:
		rospy.loginfo("main DEBUG mode")

	main = Main()
	main.init()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		nav_manager.stop()
		print("Shutting down")