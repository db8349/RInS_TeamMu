#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import tf2_geometry_msgs
import tf2_ros
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import message_filters

import math

import numpy as np
import cv2

from std_msgs.msg import String

bridge = CvBridge()
curr_color = None

def detectColor(image_data):
    global curr_color

    try:
        image = bridge.imgmsg_to_cv2(image_data, "bgr8")
    except CvBridgeError as e:
        print(e)
        return
    #image = cv2.imread("frame0008.jpg")

    #crop image as the ring will only be seen in upper half of camera image
    image = image[0:240, 0:690]

    #boundary for how many nonZero pixels have to be found to consider color detected
    detectBoundary = 800

    #boundaries for red, blue, green respectively
    boundaries = [
        ([0, 100, 100], [12, 255, 255]),
        ([110, 100, 100], [130, 255, 255]),
        ([36, 50, 50], [86, 255, 255])
    ]

    colors = ["red", "blue", "green"]

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    i = 0
    for (lower, upper) in boundaries:
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")
    
        mask = cv2.inRange(hsv, lower, upper)
        output = cv2.bitwise_and(image, image, mask = mask)
        countNonZero = np.count_nonzero(output)
        #print(countNonZero)

        #color detected
        if countNonZero > detectBoundary:
            #print("Found {}".format(colors[i])) 
            #return i
            break

        #cv2.imshow("images", np.hstack([image, output]))
        #cv2.waitKey(0)
        i += 1
    
    if i == 0:
        curr_color = "red"
    elif i == 1:
        curr_color = "blue"
    elif i == 2:
        curr_color = "green"
    elif i == 3:
        curr_color = "black"

    return i

if __name__ == '__main__':
        rospy.init_node('color_detection', anonymous=False)
        try:
            color_pub = rospy.Publisher("color_detection", String)
            rospy.Subscriber("/camera/rgb/image_color", Image, detectColor)
            while not rospy.is_shutdown():
                color_pub.publish(curr_color)
                rospy.sleep(0.5)

        except rospy.ROSInterruptException:
            pass