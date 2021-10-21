#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

global encoder_msg
encoder_msg = "S102410241024000000000000F"

global check
check = False

def callback(data):
	global encoder_msg
	rospy.loginfo(data.data)
	encoder_msg = data.data
	check = True


if __name__ == '__main__':
	rospy.init_node('encoder_test')
	encoder_pub = rospy.Publisher('encoder',String,queue_size=50)
	rospy.Subscriber('rover_serial_arm',String,callback)
	
	rate = rospy.Rate(150)

	while not rospy.is_shutdown():
		encoder_pub.publish(encoder_msg)
		rate.sleep()
		check = False


