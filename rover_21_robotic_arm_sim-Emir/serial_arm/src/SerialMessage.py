#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import serial
import rosparam
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

global axes
axes = ["0000","0000","0000","0000","0000","0000","0000"]
'''Eksenlere gonderilecek aci ve hiz degerlerinin tutuldugu liste'''

global wheel_vel
wheel_vel = ["0000","0000"]
'''Alt yururu surerken sag ve sol tekerlere gonderilecek hizlarin tutuldugu liste'''

global mode_bit
mode_bit = 9 
'''mode_bit = 9 (inverse) / mode_bit = 8 (forward) / mode_bit = 1 (wheel)
Serial mesajindaki modu belirleyen degisken'''
 
global serialString 
serialString = "/dev/ttyUSB0"
'''Iletisimi baslatirken tanimlanan port ismi'''

global ser
ser = serial.Serial(port=serialString, baudrate=int(115200), parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS,timeout=1)
'''Serial iletisimi baslatir '''

global encoder_angles
encoder_angles = [0,0]

def callback_send(data):
	'''Ters kinematik algoritmasindan gelen aci verilerini isleyen fonksiyon. Daha sonra bu verileri serial porttan gonderir'''
	global ser, mode_bit
	mode_bit = data.data[7]

	if mode_bit == 9:
		angle_to_string(0,data.data[0])
		angle_to_string(1,data.data[1])
		angle_to_string(2,data.data[2])
		angle_to_string(3,data.data[3])
		angle_to_string(4,data.data[4])
		velocity_to_string(5,data.data[5])
		velocity_to_string(6,data.data[6])
		
		'''Bu kosul ters kinematik modundayken calisir. Ilk bes eksenin aci olarak konum degerlerini gonderir'''

	elif mode_bit == 8:
		angle_to_string(0,data.data[0])
		angle_to_string(1,data.data[1])
		angle_to_string(2,data.data[2])
		velocity_to_string(3,data.data[3])
		velocity_to_string(4,data.data[4])
		velocity_to_string(5,data.data[5])
		velocity_to_string(6,data.data[6])

		'''Bu kosul ileri kinematik modundayken calisir. Ilk uc eksenin konum, geri kalan eksenlerin hiz bilgilerini gonderir'''

	
	final_message = create_final_message()
	rospy.loginfo(final_message)
	print("\n\n\n")
	serial_pub.publish(final_message)
	ser.write(final_message+"\n")

	'''Serial mesajinin son hali olusturulup porttan yazdirilir'''

def angle_to_string(axis_index,radian):	
	global axes
	degrees = 180*radian/math.pi
	print("Axis: "+ str(axis_index+1) + " DEGREES: "+ str(degrees))
	if degrees >= 0:
		string_angle = "1"
	elif degrees < 0:
		string_angle = "0"
	degrees = abs(degrees)
	if degrees > 180:
		degrees = 180

	mapped_degree = str(int(degrees*5))

	string_angle = string_angle + "0"*(3-len(mapped_degree)) + mapped_degree
	print(string_angle)
	axes[axis_index] = string_angle

	'''Gelen acilari isleyip string halinde axes listesinin icine atar.
	Acilar oncelikle radyandan dereceye dondurulur ve enkoderler el verdigince
	daha yuksek cozunurluklu bir degerle ifade edilir (Burada 5 ile carpiyoruz)'''



def create_final_message():
	global axes, mode_bit, wheel_vel
	final_message = "S" + wheel_vel[0] + wheel_vel[1]
	for i in axes:
		final_message += i
	final_message = final_message + 16*"0" + str(int(mode_bit)) + "F"
	return final_message

	'''Serial mesajin son halini olusturur. Burada alt yurur vs. tek bir
	serial mesaji halinde iletiliyor'''	



def velocity_to_string(axis_index,data):
	global axes

	if data >= 0:
		direction = "1"
	else:
		direction = "0"
	if axis_index == 3:
		data = axis_4_except(data)
	data = abs(data)
	
	if axis_index != 3:
		if data >= 0.97:
			data *= 150
		elif data < 0.97:
			data *= 75
	print("Axis: " + str(axis_index+1) + " VELOCITY: " + str(int(data)))
	data = str(int(abs(data)))
	string_vel = direction + "0"*(3-len(data)) + data
	axes[axis_index] = string_vel

	'''Robot kol eksenlerine hiz olarak veri gonderilecekse bu fonksiyon cagirilir
	Hiz degerlerini elektronik kismi tarafindan istenilen sekle sokar'''

def axis_4_except(data):
	min_vel = 55
	if data >= 0.97 or data <= -0.97:
		data *= 200
	elif data > -0.97 and data < 0:
		data = min_vel - data*40
	elif data < 0.97 and data > 0:
		data *= 80
	return data

	'''Kolun dorduncu ekseninde istisnai bir durum oldugundan ayri bir fonksiyon daha yazdik'''


def callback_wheel(data):	
	global ser, mode_bit
	mode_bit = data.data[2]

	wheel_vel_to_string(0,data.data[0])
	wheel_vel_to_string(1,data.data[1])
	final_message = create_final_message()
	print("\n\n\n")
	rospy.loginfo(final_message)
	serial_pub.publish(final_message)
	ser.write(final_message+"\n")

	'''Alt yurur icin ana programdan gelecek verileri okuyup isleyen callback fonksiyonu.
	Okuyup isledikten sonra seri porta yazdirir'''
	

def wheel_vel_to_string(index,data):
	global wheel_vel

	if data > 0:
		direction = "0"
	else:
		direction = "1"
	data *= 70
	data = int(data)
	print("Wheel index: " + str(index) + " Data: " + str(data))
	data = str(abs(data))
	string_vel = direction + "0"*(3-len(data)) + data
	wheel_vel[index] = string_vel

	'''Alt yururdeki hiz verilerini elektronigin okuyacagi sekle sokan fonksiyon
	Daha sonra serial mesaja eklenmek uzere listede tutulur'''



if __name__ == '__main__':

	rospy.init_node('serial_arm_node')
	rospy.Subscriber('/axis_states/send',Float64MultiArray,callback_send)
	rospy.Subscriber('/wheels',Float64MultiArray,callback_wheel)

	serial_pub = rospy.Publisher('rover_serial/arm',String,queue_size=50)
	axis_pub = rospy.Publisher('/axis_states/get',Float64MultiArray,queue_size=50)
	
	rate = rospy.Rate(150)

	while not rospy.is_shutdown():
		
		message = Float64MultiArray(data=(encoder_angles))
		axis_pub.publish(message)
		rate.sleep()