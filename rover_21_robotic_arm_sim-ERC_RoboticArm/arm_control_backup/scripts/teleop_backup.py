#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy
from arm_class import *


arm = Arm()

global switch_btn_current
switch_btn_current = 0

global switch_btn_prev
switch_btn_prev = 0

global switch
switch = False


def joyCallback(data):
	global arm, switch, switch_btn_current, switch_btn_prev

	#TODO: Alt yurur surulurken rk donmemeli

	if (data.buttons[0] and data.buttons[2] and data.buttons[4]):
		arm.probe_deploy_flg = 1
	if (data.buttons[1] and data.buttons[3] and data.buttons[4]):
		arm.probe_pickup_flg = 1
	
	elif (data.buttons[4] == 0):

		switch_btn_current = data.buttons[7]
		if (switch_btn_current == 1 and switch_btn_prev == 0):
			switch = True
			arm.mode += 1
			if arm.mode == 2:
				arm.mode = 0
			elif arm.mode == 0:
				print("\n\n\n\n\nMODE: SLOW \n\n")
			elif arm.mode == 1:
				print("\n\n\n\n\nMODE: SPEEDY GONZALES \n\n")

		switch_btn_prev = switch_btn_current

		if arm.mode == 0:
			arm.speed_frac = 80
			arm.forwardKinematics(data)
			arm.servo_action ^= data.buttons[5]
		elif arm.mode == 1:
			arm.speed_frac = 150
			arm.forwardKinematics(data)
			arm.servo_action ^= data.buttons[5]
		arm.activity_mode = 2
		arm.teleop_start_time = time.time()

	else:
		arm.actuator_velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]

if __name__ == '__main__':
	rospy.init_node("arm_backup_controller")
	rospy.Subscriber('joy',Joy,joyCallback)
	joint_values_pub = rospy.Publisher('joint_states/send',Float64MultiArray,queue_size=1000)
	deploy_check_pub = rospy.Publisher('probe/deploy',String,queue_size=1000)
	pickup_check_pub = rospy.Publisher('probe/pickup',String,queue_size=1000)
	rate = rospy.Rate(5)


	while not rospy.is_shutdown():

		if arm.mode == 0:
			rospy.loginfo("SLOW FK MODE\n")
			pub_msg = arm.returnActuatorVel()
			joint_values_pub.publish(pub_msg)
			

		elif arm.mode == 1:
			rospy.loginfo("SPEEDY GONZALES FK MODE\n")
			pub_msg = arm.returnActuatorVel()
			joint_values_pub.publish(pub_msg)
		
		if arm.probe_deploy_flg:
			time.sleep(0.4)
			deploy_check_pub.publish('F')
			arm.probe_deploy_flg = 0
			
		
		if arm.probe_pickup_flg:
			time.sleep(0.4)
			pickup_check_pub.publish('F')
			arm.probe_pickup_flg = 0

		end = time.time()
		if (end-arm.teleop_start_time > 5):
			arm.activity_mode = 1

		#rospy.loginfo(arm.actuator_velocities)

		rate.sleep()
		

