#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
''' Robot kolu ilgilendiren nitelik ve metotlar bu dosya icinde bir sinif olarak 
tanimlanmistir. Algoritmanin calisacagi ana dosya bu dosyayi icine aktarir ve robot kol
ozelliklerine buradaki sinif uzerinden ulasir '''

class RoboticArm:

	def __init__(self):

		self.link_1 = 37.5
		self.link_2 = 27.5
		'''Ilk iki eksenin uzunluklari'''

		self.angles = [0.0,0.0,0.0]
		'''ileri kinematik sonrasi acilari guncellemek icin bir liste'''

		self.current_pos = np.array([27.5,0.0,43.5]) #z ekseni 6cm + 37.5cm 
		self.delta_pos = np.array([0.0,0.0,0.0])
		self.desired_pos = np.array([27.5,0.0,43.5])

		'''Ustteki arrayler kartezyen sistemde kontrol ettigimiz referans
		noktasini ilgilendiren degiskenlerdir. desired_pos delta_pos ile birlestirilerek
		gidilmek istenen noktanin testleri yapilir. Sinirlar icindeyse desired artik current_pos'a
		atanir'''

		self.r_modulus = self.calculate_modulus(self.current_pos)
		''' Atanan noktanin modulusu. Robot kolun kendi x ve y ekseninin dondugunu dusunursek, r_modulus
		degiskeni her zaman robot kolun x ekseni uzerinde bir buyukluk oluyor '''

		self.r_upper_limit = 80 #48
		self.r_lower_limit = 0 #5
		self.vertical_upper_limit = 55	#43 onceki
		self.vertical_lower_limit = 0
		''' Modulus (teknik olarak robot kolun uzandigi ya da katlandigi mesafe). Boyle bi limit
		kullanditkan sonra x ve y'yi ayri ayri kontrol etmeye gerek kalmaz. Dikey eksende de verilen degerler
		arasinda bir kontrol yapiyor'''

		self.upper_joint_limits = [3.14,3.14,3.14,3.14,3.14,3.14] #1. 2. 3. 4. 5. eksenler
		self.lower_joint_limits = [-3.14,-3.14,-3.14,-3.14,-3.14,-3.14] # 1. 2. 3. 4. eksenler
		'''Eksenler icin tanimlanmak istenen (yazilimsal) alt ve ust sinirlari iceren listeler. 
		Son halinde sinirlari kaldirdik. Istenildigi sekilde degistirilebilir'''
		

		self.ee_orientation = [0.0,0.0,0.0]
		'''End effector yonelimi acilarini tutan liste'''

		self.delta_orientation = [0.0,0.0,0.0]
		'''End effector kontrolu icin son uc eksenin aci degisimleri
		[eksen4,eksen5,eksen6] seklinde'''

	def get_angles(self,angle_list):
		self.angles[0] = angle_list[0]
		self.angles[1] = angle_list[1] 
		self.angles[2] = angle_list[2]
		'''Acilarin guncellenmesi icin kullanilacak fonksiyon'''

	def update_position(self):
		theta_1 = self.angles[0]
		theta_2 = self.angles[1] + math.pi/2
		theta_3 = self.angles[2] - math.pi/2
		z = self.link_2*math.sin(theta_2+theta_3)+self.link_1*math.sin(theta_2)
		z = z + 6
		r_vector = self.link_2*math.cos(theta_2+theta_3)+self.link_1*math.cos(theta_2)
		x = r_vector*math.cos(theta_1)
		y = r_vector*math.sin(theta_1)
		self.current_pos[0] = x
		self.current_pos[1] = y
		self.current_pos[2] = z
		self.desired_pos = self.current_pos
		'''Mod gecislerinden sonra/once kolun konumunu acilara gore guncelleyen fonksiyon '''


	def update_desired_pos(self,angle_list):
		theta_1 = angle_list[0]
		theta_2 = angle_list[1] + math.pi/2
		theta_3 = -angle_list[2] - math.pi/2
		z = self.link_2*math.sin(theta_2+theta_3)+self.link_1*math.sin(theta_2)
		z = z + 6
		r_vector = self.link_2*math.cos(theta_2+theta_3)+self.link_1*math.cos(theta_2)
		x = r_vector*math.cos(theta_1)
		y = r_vector*math.sin(theta_1)
		self.desired_pos[0] = x
		self.desired_pos[1] = y
		self.desired_pos[2] = z
		'''Kolun istenilen konumunu guncelleyen fonksiyon'''

	def calculate_modulus(self,coordinates):
		modulus = math.sqrt(coordinates[0]**2 + coordinates[1]**2)
		return modulus
		'''Modulusu hesaplar, limit kontrolu ve daha sonra iki boyuta aktarirken de x eksenindeki 
		konumu modulus verecek'''

	def check_joint_limit(self,joint_index,angle):
		if self.upper_joint_limits[joint_index] <= angle or self.lower_joint_limits[joint_index] >= angle:
			rospy.logerr("EKSEN DAHA FAZLA DONEMEZ")
			print(joint_index+1,". eksen sinirda")
			return False
		else:
			return True
		'''Verilen eksenin sinirlarini kontrol ediyor, radyan, buyukluk olarak bakiyor'''

	def check_all_joints(self,angle_list):
		control = True
		index = 0
		for i in angle_list:
			if i >= self.upper_joint_limits[index] or i <= self.lower_joint_limits[index]:
				print("-------------- limitleri asiyosun cok dikkat cekiyosun --------")
				control = False
				break
			else:
				index += 1
		return control
		''' Gerektigi takdirde tum eksenlerin aci sinirlarini ayni anda kontrol eder.
		Eksenlerden biri dahi kendi sinirini astiginde control degiskenini False olarak doner '''

	def check_distance(self,coordinates):
		given_modulus = self.calculate_modulus(coordinates)
		if self.r_upper_limit <= given_modulus:
			rospy.logerr("---ROBOT KOL DAHA UZAGA ERISEMEZ---")
			return False
		elif self.r_lower_limit >= given_modulus:
			rospy.logerr("---ROBOT KOL DAHA FAZLA KATLANAMAZ---")
			return False
		else:
			return True
		'''Modulusun buyuklugu uzerinden sinir kontrolunu gerceklestiriyor. Robot kolun x-y
		uzerinde uzanabildigi ve katlanabildigi degerleri kontrol ediyor
		NOT: modulus buyuklugu dikey ust ya da dikey alt sinirda uzanilabilecek en uzak nokta seklinde
		secilecek.'''

	def check_vertical_limit(self,coordinates):
		given_vertical_pos = coordinates[2]
		if given_vertical_pos <= self.vertical_lower_limit:
			rospy.logerr("---ROBOT KOL DAHA ASAGIYA INEMEZ---")
			return False
		elif given_vertical_pos >= self.vertical_upper_limit:
			rospy.logerr("---ROBOT KOL DAHA COK YUKSELEMEZ---")
			return False
		else:
			return True
		'''Kontrol edilen noktanin ust ve alt sinirlarini dikey eksende kontrol ediyor.
		Sinirlarin disina cikacaksa ilerlemesine izin vermiyor'''

	def update_orientation(self,joint_index):
		if joint_index == 0:
			self.ee_orientation[0] += self.delta_orientation[0]
		elif joint_index == 1:
			self.ee_orientation[1] += self.delta_orientation[1]
		elif joint_index == 2:
			self.ee_orientation[2] += self.delta_orientation[2]
		''' ileri kinematik koduyla son 3 ekseni surerken gerekiyor. Son uc eksenin 
		aci guncellemelerini yapiyor'''