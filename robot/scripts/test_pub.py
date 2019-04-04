#!/usr/bin/env python
import roslib
roslib.load_manifest('robot')
import rospy
import sensor_msgs.msg
import message_filters
from std_msgs.msg import String

def talker():
	pub = rospy.Publisher('pause', String, queue_size=10)
	while not rospy.is_shutdown():
		i = raw_input("Press Enter to pause/resume the search")
		pub.publish("hello world")

if __name__ == '__main__':

		rospy.init_node('test_pub', anonymous=False)
		try:
			talker()
		except rospy.ROSInterruptException:
			pass