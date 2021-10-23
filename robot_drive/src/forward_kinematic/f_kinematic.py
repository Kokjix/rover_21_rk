#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64 as F64
from sensor_msgs.msg import Joy
from F_Arm import *
from std_msgs.msg import Float64MultiArray

""" Bu kod ileri kinamtik sürüş için yapılmıştır yani kolun eklemlerine ayrı ayrı joystick verileri basılmaktadır hiç bir ters kinematik kodu içermemektedir."""
#global joint_names, detlta_thetas
#joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#delta_thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

arm = F_ARM()


def joy_callback(data):

    arm.delta_thetas[0] = data.axes[0] * 0.003
    arm.delta_thetas[1] = data.axes[1] * 0.003
    arm.delta_thetas[2] = data.axes[4] * 0.004
    arm.delta_thetas[3] = -data.axes[3] * 0.003
    arm.delta_thetas[4] = -data.axes[7] * 0.003
    arm.delta_thetas[5] = data.axes[6] * 0.006

    if data.buttons[4] != 0:
        arm.delta_thetas[6] = data.buttons[4] * 0.004

    else:
        arm.delta_thetas[6] = -data.buttons[5] * 0.004


if __name__ == "__main__":
    rospy.init_node('joy_control')
    joint_1_publisher = rospy.Publisher('/rover_arm_joint_1/command', F64, queue_size=10)
    joint_2_publisher = rospy.Publisher('/rover_arm_joint_2/command', F64, queue_size=10)
    joint_3_publisher = rospy.Publisher('/rover_arm_joint_3/command', F64, queue_size=10)
    joint_4_publisher = rospy.Publisher('/rover_arm_joint_4/command', F64, queue_size=10)
    joint_5_publisher = rospy.Publisher('/rover_arm_joint_5/command', F64, queue_size=10)
    joint_6_publisher = rospy.Publisher('/rover_arm_joint_6/command', F64, queue_size=10)
    axis_state = rospy.Publisher('/axis_states/send', Float64MultiArray, queue_size=10)
    axis=[0,0,0,0,0,0]

    right_finger_publisher = rospy.Publisher('/rover_arm_right_finger/command', F64, queue_size=10)
    left_finger_publisher = rospy.Publisher('/rover_arm_left_finger/command', F64, queue_size=10)
    rospy.Subscriber('joy', Joy, joy_callback)

    rate = rospy.Rate(150)
    counter = 0

    while not rospy.is_shutdown():
        #rospy.loginfo("I am ready")
        
        rospy.loginfo_throttle(2,"joint1 data is %s" %arm.joint_angles[0])
        rospy.loginfo_throttle(2,"------------------------------------")
        rospy.loginfo_throttle(2,"joint2 data is %s" %arm.joint_angles[1])
        rospy.loginfo_throttle(2,"------------------------------------")
        rospy.loginfo_throttle(2,"joint3 data is %s" %arm.joint_angles[2])
        rospy.loginfo_throttle(2,"------------------------------------")
        rospy.loginfo_throttle(2,"joint4 data is %s" %arm.joint_angles[3])
        rospy.loginfo_throttle(2,"------------------------------------")
        rospy.loginfo_throttle(2,"joint5 data is %s" %arm.joint_angles[4])
        rospy.loginfo_throttle(2,"------------------------------------")
        rospy.loginfo_throttle(2,"joint6 data is %s" %arm.joint_angles[5])
        rospy.loginfo_throttle(2,"------------------------------------")

        

        arm.joint_angles[0] += arm.delta_thetas[0]
        arm.joint_angles[1] += arm.delta_thetas[1]
        arm.joint_angles[2] += arm.delta_thetas[2]
        arm.joint_angles[3] += arm.delta_thetas[3]
        arm.joint_angles[4] += arm.delta_thetas[4]
        arm.joint_angles[5] += arm.delta_thetas[5]

        joint_1_publisher.publish(arm.joint_angles[0])
        joint_2_publisher.publish(arm.joint_angles[1])
        joint_3_publisher.publish(arm.joint_angles[2])
        joint_4_publisher.publish(arm.joint_angles[3])
        joint_5_publisher.publish(arm.joint_angles[4])
        joint_6_publisher.publish(arm.joint_angles[5])
        if counter == 50: #Serial'e gönderilen açı verileri.
            axis[0] = arm.joint_angles[0]
            axis[1] = arm.joint_angles[1]
            axis[2] = arm.joint_angles[2]
            axis[3] = arm.joint_angles[3]
            axis[4] = arm.joint_angles[4]
            axis[5] = arm.joint_angles[5]

            message = Float64MultiArray(data=axis)
            axis_state.publish(message)
            counter = 0

        counter += 1

    

        right_finger_publisher.publish(arm.joint_angles[6])
        left_finger_publisher.publish(arm.joint_angles[6])

        rate.sleep()


        

    
    




