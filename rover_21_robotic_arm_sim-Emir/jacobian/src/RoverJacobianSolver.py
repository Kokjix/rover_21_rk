#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
'''Robot kolun ilk uc ekseninde kullandigimiz ters kinematik cozucuyu iceren dosya.
Ilk eksen hedefe yoneldikten sonra acilar pseudo inverse jacobian yontemiyle hesaplaniyor.
Gazebo ve gercek dunyadaki acilarin farklilik gostermesi nedeniyle gerekli islemler ters kinematik
cozumun oncesinde yapiliyor'''


class JacobianSolver:

	def __init__(self):
		self.link_1 = 37.5 
		self.link_2 = 27.5 
		''' ilk iki baglantinin uzunluklari '''

		self.theta = np.array([1.328,-1.15])
		''' 2. ve 3. eksenin baslangic acilarini tutan array.
		Burada baslangic acilarini urdfteki gibi aldim. Enkoder okunduktan sonra bu acilar guncellenmeli'''

		#TODO: acilar sabit degil enkoderden geldigi sekilde alinacak.

		#self.theta_difference = [0,0]
		'''orientationda kullanilacak acilar'''

		self.theta_pub = np.array([0,0],dtype=np.float)
		'''Robot kola gonderilecek acilar. hesaplamalardakilerle tam uymadiklarindan
		donusumu yapilip bu degerler gonderiliyor'''

		self.ee_pos = np.array([0,0,0],dtype=np.float)
		'''Kontrol edilen noktanin pozisyon bilgisini tutacak vektor. z her zaman sifir'''

		self.goal_pos = np.array([0,0,0],dtype=np.float)
		'''Hedef noktanin x,y,z seklinde koordinatlari'''

		self.joint_pos = np.array([[0,0,0],[0,0,0]],dtype=np.float)
		'''Eklemlerin uzaydaki pozisyonlari. ilk eleman 1. eklem, 2. eleman 2. eklem'''

		self.delta_s = np.array([0,0,0],dtype=np.float)
		'''Hedef nokta ile anlik konum arasindaki farki belirten vektor'''

		self.update_current_pos()
		self.update_joint_pos()

	def get_thetas(self,theta_1,theta_2):
		self.theta[0] = theta_1 + math.pi/2
		self.theta[1] = -theta_2 - math.pi/2
		self.update_current_pos()
		'''Ters kinematigin calistigi ana dosyadan acilari aliyor.
		Gazebo ve cozumdeki acilar farkli oldugu icin bir donusum yapiliyor'''

	def update_current_pos(self):
		self.ee_pos[0] = self.calculateX()
		self.ee_pos[1] = self.calculateY()
		''' End effector konumunu guncelleyen metot'''

	def calculateX(self):
		x = self.link_2*math.cos(self.theta[0]+self.theta[1])+self.link_1*math.cos(self.theta[0])
		return x

	def calculateY(self):
		y = self.link_2*math.sin(self.theta[0]+self.theta[1])+self.link_1*math.sin(self.theta[0])
		return y

	def update_joint_pos(self):
		self.joint_pos[1,0] = math.cos(self.theta[0])*self.link_1
		self.joint_pos[1,1] = math.sin(self.theta[0])*self.link_1
		'''Eklemlerin bulundugu noktalari gunceller.
		ilk eklem konumu sabit oldugu icin sadece ikinci eklemin
		konumunu temel trigonometri ile tekrar hesaplar 
		NOT: Daha iyi bir cozum olarak bunlar homojen matrislerle hesaplanabilir
		'''

	def get_goal_pos(self,x,y,z=0):
		self.goal_pos[0] = x
		self.goal_pos[1] = y-6
		''' ilk eksen biraz yukarida. Simdilik boyle cozuyorum problemi. 
		Daha sonra kod tumden degisebilir'''
		self.goal_pos[2] = z
		
	def calculate_jacobian(self):
		k_unit = np.array([0,0,1],dtype=np.float)
		jacobian = np.zeros((3,2),dtype=np.float)
		'''Bos bir jacobian matrisi olusturuluyor ve donus ekseni belirleniyor
		Bu durum icin donus eksenimiz iki boyuttan disari bakacak olan z ekseni'''

		for i in range(2):
			joint_coordinates = self.joint_pos[[i],:3]
			jacobian[:,i] = np.cross(k_unit,(self.ee_pos - joint_coordinates).reshape(3,))
		return jacobian
		'''Metot sifirlardan olusturdugu jacobian matrisinin elemanlarini analitik olarak
		hesapliyor ve dolduruyor. Hesaplanan matrisi fonksiyon disariya donuyor'''

	def update_angles(self,delta_theta):
		self.theta += delta_theta
		'''Sadece acilari guncellemeye yarayan metot'''

	def solve(self):
	
		frac = 0.01
		'''Degisim miktarini ifade edecek fraksiyon'''

		self.delta_s = self.goal_pos - self.ee_pos
		distance = self.find_goal_distance()

		while distance > 0.05:
			gucuk_delta = self.delta_s*frac
			Jacob = self.calculate_jacobian()
			PinvJacob = np.linalg.pinv(Jacob)
			delta_theta = PinvJacob.dot(gucuk_delta)
			self.update_angles(delta_theta)
			self.update_current_pos()
			self.update_joint_pos()
			self.delta_s = self.goal_pos - self.ee_pos
			distance = self.find_goal_distance()

		self.angle_transformation()

		'''Bu metot pseudo inverse jacobian matrisini kullanarak tekrarli sekilde hesaplama yapar.
		Hedef verilen konum ile anlik bulunan konum arasindaki mesafe ihmal edilebilir bir noktaya 
		geldiginde isleme son verilir.'''


	def find_goal_distance(self):
		dist = math.sqrt(self.delta_s[0]**2 + self.delta_s[1]**2 + self.delta_s[2]**2)
		return dist
		'''Algoritmadaki mesafe kontrolu icin mesafeyi hesaplayan metot '''

	def angle_transformation(self):
		self.theta_pub[0] = self.theta[0] - (math.pi/2)
		self.theta_pub[1] = -(self.theta[1] + (math.pi/2))
		''' Acilarin gazeboya uygun hale getirilmesini saglayan metot. Bu donusum yapildiktan sonra
		acilar publishlenmeye uygun hale gelir'''