#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import serial
import time
from std_msgs.msg import String
import rosparam


'''baudrate vs???'''


class SerialCom():

    def __init__(self):
        rospy.init_node("arm_serial_com")
        #self.serialString = "/dev/ttyUSB0"
        self.namespace = '[RoverSerial : ] '
        rospy.Subscriber("/rover_serial/arm", String, self.serial_callback) #0-1-2-5-6. eksenler icin
        rate = rospy.Rate(1)




        self.serialMsg = ""
        #gonderilecek serial verisi

    def serial_callback(self,data):
        self.serialMsg = data.data
        rospy.loginfo(self.serialMsg)
        #topicten gelen mesaji sinifin serial mesajina atar
        #buradan seriale yazdir

    def serial_func(self):

        #ser = serial.Serial(port=self.serialString, baudrate=int(115200), parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)
        #open serial

        #ser.timeout = 1

        #print(self.serialMsg)
        rospy.loginfo(self.serialMsg)



if __name__ == "__main__":

    com = SerialCom()
    counter = 0

    while not rospy.is_shutdown():

        counter += 1

        if counter == 20:  
            com.serial_func()
            counter = 0
        
