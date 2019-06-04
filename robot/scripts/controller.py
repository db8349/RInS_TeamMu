#!/usr/bin/env python
import roslib
roslib.load_manifest('robot')
import rospy

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3, Quaternion, Twist, Pose
from std_msgs.msg import ColorRGBA, String

from robot.msg import Numbers, Circle, QRCode

rospy.init_node('nav_manager', anonymous=False)

debug = rospy.get_param('/debug')

class Main():
	def __init__(self):
		self.marker_array = MarkerArray()
		self.marker_num = 1
		self.markers_pub = rospy.Publisher('markers', MarkerArray, queue_size=10000)

		nav_goto_publisher = rospy.Publisher("/nav_manager/go_to", Pose, queue_size=100)

		rospy.Subscriber("circle_sense/numbers", Numbers, self.numbers)
		rospy.Subscriber("circle_sense/qr_code", QRCode, self.qr)
		rospy.Subscriber("circle_sense/circle", Circle, self.circle)

	def qr(self, data):
		rospy.loginfo("QR data: {}".format(data))

	def circle(self, circle):
		rospy.loginfo("New Circle: {}, {}".format(circle.circle_pose.position.x, circle.circle_pose.position.y))
		#self.show_point(circle.circle_pose)
		#circle_approach_pose = self.approach_transform(circle.curr_pose, circle.circle_pose, 0.4)
		#rospy.loginfo("Circle approach: ({}, {})".format(circle_approach_pose.position.x, circle_approach_pose.position.y))
		#nav_goto_publisher.publish(circle_approach_pose)

	def numbers(self, numbers):
		rospy.loginfo("Numbers: {}, {}".format(numbers.first, numbers.second))

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

		yaw = math.atan(v.y/v.x)

		q = Quaternion()
		q.w = math.cos(yaw/2)
		q.x = 0
		q.y = 0
		q.z = math.sin(yaw/2)
		
		pose = Pose(v, q)

		return pose

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