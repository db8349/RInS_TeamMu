#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from geometry_msgs.msg import PointStamped, Vector3, Pose
from robot.msg import Numbers

rospy.init_node('circle_sense', anonymous=True)

debug = rospy.get_param("/debug")

dictm = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

# The object that we will pass to the markerDetect function
params =  cv2.aruco.DetectorParameters_create()

# To see description of the parameters
# https://docs.opencv.org/3.3.1/d1/dcd/structcv_1_1aruco_1_1DetectorParameters.html

# You can set these parameters to get better marker detections
params.adaptiveThreshConstant = 25
adaptiveThreshWinSizeStep = 2

class CircleSense:
	def __init__(self):
		# An object we use for converting images between ROS format and OpenCV format
		self.bridge = CvBridge()

		# Subscribe to the image topic
		self.image_sub = rospy.Subscriber(rospy.get_param("/image_topic"), Image, self.image_callback)

		self.numbers_pub = rospy.Publisher("numbers_detect/numbers", Numbers, queue_size=10)

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
		if len(candidates) == 1:
			self.processDetectNumbers(thresh)

	def processDetectNumbers(self, cv_image):
		corners, ids, rejected_corners = cv2.aruco.detectMarkers(cv_image,dictm,parameters=params)
		
		# Increase proportionally if you want a larger image
		image_size=(351,248,3)
		marker_side=50

		img_out = np.zeros(image_size, np.uint8)
		out_pts = np.array([[marker_side/2,img_out.shape[0]-marker_side/2],
							[img_out.shape[1]-marker_side/2,img_out.shape[0]-marker_side/2],
							[marker_side/2,marker_side/2],
							[img_out.shape[1]-marker_side/2,marker_side/2]])

		src_points = np.zeros((4,2))
		cens_mars = np.zeros((4,2))

		if not ids is None:
			if len(ids)==4:
				#if debug: rospy.loginfo('4 Markers detected')

				for idx in ids:
					# Calculate the center point of all markers
					cors = np.squeeze(corners[idx[0]-1])
					cen_mar = np.mean(cors,axis=0)
					cens_mars[idx[0]-1]=cen_mar
					cen_point = np.mean(cens_mars,axis=0)
			
				for coords in cens_mars:
					#  Map the correct source points
					if coords[0]<cen_point[0] and coords[1]<cen_point[1]:
						src_points[2]=coords
					elif coords[0]<cen_point[0] and coords[1]>cen_point[1]:
						src_points[0]=coords
					elif coords[0]>cen_point[0] and coords[1]<cen_point[1]:
						src_points[3]=coords
					else:
						src_points[1]=coords

				h, status = cv2.findHomography(src_points, out_pts)
				img_out = cv2.warpPerspective(cv_image, h, (img_out.shape[1],img_out.shape[0]))
				
				################################################
				#### Extraction of digits starts here
				################################################
				
				# Cut out everything but the numbers
				img_out = img_out[125:221,50:195,:]
				
				# Convert the image to grayscale
				img_out = cv2.cvtColor(img_out, cv2.COLOR_BGR2GRAY)
				
				# Option 1 - use ordinairy threshold the image to get a black and white image
				#ret,img_out = cv2.threshold(img_out,100,255,0)

				# Option 1 - use adaptive thresholding
				img_out = cv2.adaptiveThreshold(img_out,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,5)
				
				# Use Otsu's thresholding
				#ret,img_out = cv2.threshold(img_out,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
				
				# Pass some options to tesseract
				config = '--psm 13 outputbase nobatch digits'
		
				# Extract text from image
				text = pytesseract.image_to_string(img_out, config = config)
				
				# Remove any whitespaces from the left and right
				text = text.strip()
				
  #             # If the extracted text is of the right length
				if len(text)==2:
					if not text[0].isdigit() or not text[1].isdigit():
						return

					first=int(text[0])
					second=int(text[1])

					numbers = Numbers()
					numbers.first = first
					numbers.second = second
					self.numbers_pub.publish(numbers)
					
					self.qualifying = False
					if debug: rospy.loginfo("Publishing numbers {} and {}".format(first, second))
				else:
					pass
					#if debug: rospy.loginfo('The extracted text has is of length %d. Aborting processing' % len(text))
				
			else:
				pass
				#if debug: rospy.loginfo('The number of markers is not ok: {}'.format(len(ids)))

def main(args):
	if debug:
		rospy.loginfo("numers_detect is in DEBUG mode")

	main = Main()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	main(sys.argv)
