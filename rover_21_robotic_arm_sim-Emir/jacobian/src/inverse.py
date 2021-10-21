#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy
from RoboticArm import *
from RoverJacobianSolver import *
import time
#from SerialMessage import *


arm = RoboticArm()
ikSolver = JacobianSolver()
'''Ters kinematik cozumleri icin kullanilacak nesneler olusturuluyor '''

global control
control = 0

global serial_mode_bit
serial_mode_bit = 9
# serial_mode = 9, inverse
# serial_mode = 8, forward
# serial_mode = 1, alt yurur
'''Surus modunu kontrol eden degisken (serial mesaj baglaminda).
Yukaridaki uc moda gore surus farklilik gosteriyor '''

global mode
mode = 0
# mode = 0, jacobian
# mode = 1, ileri kinematik
# mode = 2, alt yurur

global switch
switch = False

global delta_thetas
delta_thetas = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]

global vel_axes
vel_axes = [0,0]

global incoming_thetas
incoming_thetas = [0,0]

global wheels
wheels = [0,0]

global gripper_thetas
gripper_thetas = [0,0]

global switch_btn_current
switch_btn_current = 0

global switch_btn_prev
switch_btn_prev = 0

def joy_callback(data):
	global control, mode, switch, switch_btn_current, switch_btn_prev


	switch_btn_current = data.buttons[7]
	if (switch_btn_current == 1 and switch_btn_prev == 0):
		switch = True
		mode += 1
		if mode == 3:
			mode = 0
		if mode == 0:
			print("\n\n\n\n\nMODE: JK INVERSE CONTINUOUS")
		elif mode == 1:
			print("\n\n\n\n\nMODE: FORWARD KINEMATICS")
		elif mode == 2:
			print("\n\n\n\n\nMODE: WHEELS")

	switch_btn_prev = switch_btn_current
	''' Modlar arasi tek tusla gecis yapmayi saglayan bir buton algoritmasi'''

	if mode == 0:
		''' Ters kinematik modu. Joystickten gelen veriler ile ucuncu eksen ile dorduncu ekseni
		baglayan noktanin konumu kontrol ediliyor '''

		delta_x = data.axes[1]
		if check_joy_deadband(delta_x):
			arm.delta_pos[0] = delta_x*0.1
		else:
			arm.delta_pos[0] = 0.0

		delta_y = data.axes[0]
		if check_joy_deadband(delta_y):
			arm.delta_pos[1] = delta_y*0.03
		else:
			arm.delta_pos[1] = 0.0

		delta_z = data.axes[7]
		arm.delta_pos[2] = delta_z*0.1

		control = data.buttons[0]


	elif mode == 1:
		''' Ileri kinematik modu. Joystickten gelen verilerle her bir eksen ayri ayri kontrol edilebiliyor'''
		
		global delta_thetas, vel_axes	
		

		if data.buttons[0] == 0 and data.buttons[1] == 0 and data.buttons[2] == 0 and data.buttons[3] == 0:
			delta_theta1 = data.axes[0]
			if check_joy_deadband(delta_theta1):
				delta_thetas[0] = delta_theta1*0.001
			else:
				delta_thetas[0] = 0

			delta_theta2 = data.axes[1]
			if check_joy_deadband(delta_theta2):
				delta_thetas[1] = delta_theta2*0.001
			else:
				delta_thetas[1] = 0

			delta_theta3 = data.axes[4]
			if check_joy_deadband(delta_theta3):
				delta_thetas[2] = -delta_theta3*0.001
			else:
				delta_thetas[2] = 0

		if data.buttons[0] == 1:
			delta_thetas[3] = -data.axes[1]
		else:
			delta_thetas[3] = 0

		if data.buttons[1] == 1:
			delta_thetas[4] = -data.axes[0]
		else:
			delta_thetas[4] = 0

		if data.buttons[2] == 1:
			vel_axes[0] = -data.axes[0]
		else:
			vel_axes[0] = 0

		if data.buttons[3] == 1:
			vel_axes[1] = data.axes[0]
		else:
			vel_axes[1] = 0

	elif mode == 2:
		'''Alt yurur modu. Joystickten gelen veriye gore sag ve sol tekerlere hiz gonderiliyor'''

		global wheels

		duz = data.axes[1]
		if check_joy_deadband(duz):
			wheels[0] = duz
			wheels[1] = duz
		else:
			wheels[0] = 0
			wheels[1] = 0

		if duz == 0:
			tank_turn = data.axes[3]
			if check_joy_deadband(tank_turn):
				wheels[0] = -tank_turn
				wheels[1] = tank_turn
			else:
				wheels[0] = 0
				wheels[1] = 0
		

def check_joy_deadband(value):
	deadband_limit = 0.5
	if abs(value) >= deadband_limit:
		return True
	else:
		return False
	'''Joystick analoglarindaki veriler icin deadband. Belirlenen 
	sinirin altinda veri geldiginde hareket etmiyor.'''

def angle_calculator(coordinates):
	value = coordinates[1]/coordinates[0] 
	angle = math.atan(value)
	return angle
	'''Atanan noktanin gazebodaki x ekseni ile yaptigi aciyi hesapliyor.
	Daha sonra ilk eksen bu aci kadar dondurulecek. Bu islemden sonrasi aslinda 
	iki boyuta indirgenmis oluyor'''

def encoder_callback(data):
	global incoming_thetas
	incoming_thetas[0] = data.data[0]
	incoming_thetas[1] = data.data[1]
	'''Enkoderlerden okunan veri icin yazilmis bir fonksiyon. Henuz bir islevi yok'''

if __name__ == '__main__':
	rospy.init_node('joy_controller')
	rospy.Subscriber('joy',Joy,joy_callback)
	joint_1_publisher = rospy.Publisher('/rover_arm_joint_1/command',Float64,queue_size=10)
	joint_2_publisher = rospy.Publisher('/rover_arm_joint_2/command',Float64,queue_size=10)
	joint_3_publisher = rospy.Publisher('/rover_arm_joint_3/command',Float64,queue_size=10)
	joint_4_publisher = rospy.Publisher('/rover_arm_joint_4/command',Float64,queue_size=10)
	joint_5_publisher = rospy.Publisher('/rover_arm_joint_5/command',Float64,queue_size=10)
	joint_6_publisher = rospy.Publisher('/rover_arm_joint_6/command',Float64,queue_size=10)
	joint_serial_publisher = rospy.Publisher('/axis_states/send',Float64MultiArray,queue_size=1)
	wheel_publisher = rospy.Publisher('/wheels',Float64MultiArray,queue_size=1)
	rospy.Subscriber('/axis_states/get',Float64MultiArray,encoder_callback)
	rate = rospy.Rate(150)

	angle_list = [0.0,0.0,0.0,0.0,0.0]
	temp_angle_list = [0.0,0.0,0.0,0.0,0.0]
	joint_1_publisher.publish(0.0)
	joint_3_publisher.publish(0.0)
	joint_2_publisher.publish(0.0)
	joint_4_publisher.publish(0.0)
	joint_5_publisher.publish(0.0)
	joint_6_publisher.publish(0.0)

	print("MODE 0: JK INVERSE CONTINUOUS")
	#baslangic modu 

	counter = 0
	'''Bastirdigimiz veriyi yavaslatmak icin kullandigimiz degisken. Bunu rate'den de yapabiliriz.
	Simdilik degistirmedik'''

	while not rospy.is_shutdown():


		if (mode == 0):
			'''TERS KINEMATIK'''

			if switch == True:
				
				arm.ee_orientation[0] = angle_list[3]
				arm.ee_orientation[1] = angle_list[4]
				counter = 0
				switch = False
				serial_mode_bit = 9

				angle_list[3] = incoming_thetas[0]
				angle_list[4] = incoming_thetas[1]
				'''Mod gecisinden sonra acilari guncelleyecek kisim. Tam olarak dogru calismamakta.
				Uzerine calisilacak'''


			arm.desired_pos = arm.desired_pos + arm.delta_pos
			'''Istenilen konumu gunceller, daha sonra bu konumun sinirlar dahilinde olup olmadigini kontrol eder'''


			joint_1_angle = angle_calculator(arm.desired_pos)	

			if arm.check_joint_limit(0,joint_1_angle) and arm.check_distance(arm.desired_pos) and arm.check_vertical_limit(arm.desired_pos):

				arm.current_pos = arm.desired_pos
				arm.r_modulus = arm.calculate_modulus(arm.current_pos)
				angle_list[0] = joint_1_angle 
				joint_1_publisher.publish(joint_1_angle)

				ikSolver.get_goal_pos(arm.r_modulus, arm.current_pos[2])
				ikSolver.solve()
				joint_2_angle = ikSolver.theta_pub[0]
				joint_3_angle = ikSolver.theta_pub[1] 


				if arm.check_joint_limit(1,joint_2_angle):
					angle_list[1] = joint_2_angle
					joint_2_publisher.publish(joint_2_angle)
					

				if arm.check_joint_limit(2,joint_3_angle):
					angle_list[2] = joint_3_angle
					joint_3_publisher.publish(joint_3_angle)

				'''Burada hedef olarak belirlenecek noktanin calisma sinirlari icinde olup olmadigi kontrol ediliyor
				ve sinirlar icerisindeyse eksen acilarini hesaplayip bastiriyor.'''

			else:
				arm.desired_pos = arm.current_pos
				rospy.logerr("OUT OF LIMITS")
				joint_1_angle = angle_calculator(arm.current_pos)
				joint_1_publisher.publish(joint_1_angle)
				'''Sinirlar disindaysa isleme sokmuyor'''
			
			if counter == 25 and mode == 0:

				message = Float64MultiArray(data=(angle_list+vel_axes+[serial_mode_bit]))
				joint_serial_publisher.publish(message)
				print("Angle list: (INVERSE CONT.): ",angle_list)
				counter = 0
				print(arm.current_pos)
				'''Sayac denk geldikce buradan seri porta veri yazdiriliyor.'''

			counter += 1

		elif (mode == 1):
			'''ILERI KINEMATIK'''

			if switch == True:

				counter = 0
				switch = False
				serial_mode_bit = 8


			temp_angle_list = angle_list

			temp_angle_list[0] += delta_thetas[0]
			temp_angle_list[1] += delta_thetas[1]
			temp_angle_list[2] += delta_thetas[2]
			arm.update_desired_pos(temp_angle_list)	

			if arm.check_all_joints(temp_angle_list) and arm.check_distance(arm.desired_pos) and arm.check_vertical_limit(arm.desired_pos):
				angle_list = temp_angle_list
				joint_1_publisher.publish(angle_list[0])
				joint_2_publisher.publish(angle_list[1])
				joint_3_publisher.publish(angle_list[2])
				'''Ilk uc eksenin sinirlarini pozisyon bakimindan kontrol eder. Sinirlar disinda degillerse
				acilari publishler'''
			
			else:
				arm.desired_pos = arm.current_pos
				arm.get_angles(angle_list)
				arm.update_position()
				ikSolver.get_thetas(angle_list[1],angle_list[2])
				temp_angle_list[0] -= delta_thetas[0]
				temp_angle_list[1] -= delta_thetas[1]
				temp_angle_list[2] -= delta_thetas[2]
				'''Pozisyon sinirlari asiyorsa islem yapilmaz, son degismeler geri alinir'''
				
			
			angle_list[3] = delta_thetas[3]
			angle_list[4] = delta_thetas[4]


			if counter == 15:
				message = Float64MultiArray(data=(angle_list+vel_axes+[serial_mode_bit]))
				joint_serial_publisher.publish(message)
				print("Angle list (FORWARD KINE.): ",angle_list)
				counter = 0
				print(arm.current_pos)

			
			counter +=1

		if (mode == 2):
			'''ALT YURUR'''
			'''Alt yurur verilerini liste olarak alir ve serial mesaji olusturacak node'a publishler'''

			if switch == True:

				counter = 0
				switch = False
				serial_mode_bit = 1

			if counter == 15:
				message = Float64MultiArray(data=(wheels+[serial_mode_bit]))
				wheel_publisher.publish(message)
				print(wheels)
				counter = 0

			counter += 1

				
		rate.sleep()