#!/usr/bin/env python
import roslib
roslib.load_manifest('robot')
import rospy

from geometry_msgs.msg import Point, Vector3, Quaternion, Twist, Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import pyzbar.pyzbar as pyzbar

from robot.msg import QRCode

import pyzbar.pyzbar as pyzbar

rospy.init_node('qr_detect', anonymous=False)

debug = rospy.get_param('/debug')

class Main():
	def __init__(self):
		# An object we use for converting images between ROS format and OpenCV format
		self.bridge = CvBridge()

		# Subscribe to the image topic
		self.image_sub = rospy.Subscriber(rospy.get_param("/image_topic"), Image, self.image_callback)

		self.qr_pub = rospy.Publisher("qr_detect/qr_code", QRCode, queue_size=10)

	def image_callback(self, rgb_data):
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

if __name__ == '__main__':
	if debug:
		rospy.loginfo("qr_detect DEBUG mode")

	main = Main()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		nav_manager.stop()
		print("Shutting down")