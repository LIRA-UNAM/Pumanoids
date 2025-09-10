#!/usr/bin/env python

"""
Created on Tue Oct 9 14:50:16 2017

@author: Allen
"""
import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np

a_0_1  = 0.251041666666667
a_1_1  = -0.00507291666666667
a_2_1  = 2.5625e-5
a_3_1  = 0.0

a_0_2  = -0.202083333333333
a_1_2  = 0.00338541666666667
a_2_2  = -1.4375e-5
a_3_2  = 2.5e-8

a_0_3  = 0.202083333333333
a_1_3  = -0.000385416666666666
a_2_3  = -6.25e-7
a_3_3  = 0.0

a_0_4  = 0.00246892265193384
a_1_4  = 0.00124004373848987
a_2_4  = -2.79350828729282e-6
a_3_4  = -4.55801104972376e-9



def splines():
	pub_foot_right = rospy.Publisher('spline_right_foot',Float32MultiArray, queue_size=10)
	pub_foot_left = rospy.Publisher('spline_left_foot',Float32MultiArray, queue_size=10)
	rospy.init_node('splines_node')
	rate = rospy.Rate(10) # 10hz

	foot_pos_z_right = Float32MultiArray()
	foot_pos_z_right.data = np.zeros(1500)#[0.5,0.3,0.5]

	foot_pos_z_left = Float32MultiArray()
	foot_pos_z_left.data = np.zeros(1500)#[0.5,0.3,0.5]

	""" FOOT RIGHT Z"""
	for k in range(0,1500):
		#Trayectoria del ZMP en Y*)
		if(k >= 0 and k < 100):
			foot_pos_z_right.data[k]= 0
		    
		if(k >= 100 and k < 150):#f1
			foot_pos_z_right.data[k]= a_0_1  + a_1_1  * k + a_2_1  * (k**2) + a_3_1  * (k**3)
		    
		if(k >= 150 and k < 200):
			foot_pos_z_right.data[k]= a_0_2  + a_1_2  * k + a_2_2  * (k**2) + a_3_2  * (k**3)
		    
		if(k >= 200 and k < 250):
			foot_pos_z_right.data[k]= a_0_3  + a_1_3  * k + a_2_3  * (k**2) + a_3_3  * (k**3)
		    
		if(k >= 250 and k < 300):
		    foot_pos_z_right.data[k]= a_0_4  + a_1_4  * k + a_2_4  * (k**2) + a_3_4  * (k**3)
		    
		if(k >= 300 and k < 550):
		    foot_pos_z_right.data[k]= 0
		    
		if(k >= 550 and k < 600):
		    foot_pos_z_right.data[k]= a_0_1  + a_1_1  * (k-450) + a_2_1  * ((k-450)**2) + a_3_1  * ((k-450)**3)
		    
		if(k >= 600 and k < 650):
		    foot_pos_z_right.data[k]= a_0_2  + a_1_2  * (k-450) + a_2_2  * ((k-450)**2) + a_3_2  * ((k-450)**3)
		    
		if(k >=650 and k < 700):
		    foot_pos_z_right.data[k]= a_0_3  + a_1_3  * (k-450) + a_2_3  * ((k-450)**2) + a_3_3  * ((k-450)**3)
		    
		if(k >= 700 and k < 750):
		    foot_pos_z_right.data[k]= a_0_4  + a_1_4  * (k-450) + a_2_4  * ((k-450)**2) + a_3_4  * ((k-450)**3)
		    
		if(k >= 750 and k < 1000):
		    foot_pos_z_right.data[k]= 0
		    
		if(k >= 1000 and k < 1050):
		    foot_pos_z_right.data[k]= a_0_1  + a_1_1  * (k-900) + a_2_1  * ((k-900)**2) + a_3_1  * ((k-900)**3)
		    
		if(k >= 1050 and k < 1100):
		    foot_pos_z_right.data[k]= a_0_2  + a_1_2  * (k-900) + a_2_2  * ((k-900)**2) + a_3_2  * ((k-900)**3)
		    
		if(k >=1100 and k < 1150):
		    foot_pos_z_right.data[k]= a_0_3  + a_1_3  * (k-900) + a_2_3  * ((k-900)**2) + a_3_3  * ((k-900)**3)
		    
		if(k >= 1150 and k < 1200):
		    foot_pos_z_right.data[k]= a_0_4  + a_1_4  * (k-900) + a_2_4  * ((k-900)**2) + a_3_4  * ((k-900)**3)
		    
		if(k >= 1200 and k < 1500):
		    foot_pos_z_right.data[k]= 0

	""" FOOT LEFT Z"""
	for k in range(0,1500):
	    #Trayectoria del ZMP en Y*)
	    if(k >= 0 and k < 325):
	        foot_pos_z_left.data[k]= 0
	        
	    if(k >= 325 and k < 375):#f1
	        foot_pos_z_left.data[k]= a_0_1 + a_1_1 * (k-225) + a_2_1 * ((k-225)**2) + a_3_1 * ((k-225)**3)
	        
	    if(k >= 375 and k < 425):
	        foot_pos_z_left.data[k]= a_0_2 + a_1_2 * (k-225) + a_2_2 * ((k-225)**2) + a_3_2 * ((k-225)**3)
	        
	    if(k >= 425 and k < 475):
	        foot_pos_z_left.data[k]= a_0_3 + a_1_3 * (k-225) + a_2_3 * ((k-225)**2) + a_3_3 * ((k-225)**3)
	        
	    if(k >= 475 and k < 525):
	        foot_pos_z_left.data[k]= a_0_4 + a_1_4 * (k-225) + a_2_4 * ((k-225)**2) + a_3_4 * ((k-225)**3)
	        
	    if(k >= 525 and k < 775):
	        foot_pos_z_left.data[k]= 0
	        
	    if(k >= 775 and k < 825):
	        foot_pos_z_left.data[k]= a_0_1 + a_1_1 * (k-675) + a_2_1 * ((k-675)**2) + a_3_1 * ((k-675)**3)
	        
	    if(k >= 825 and k < 875):
	        foot_pos_z_left.data[k]= a_0_2 + a_1_2 * (k-675) + a_2_2 * ((k-675)**2) + a_3_2 * ((k-675)**3)
	        
	    if(k >=875 and k < 925):
	        foot_pos_z_left.data[k]= a_0_3 + a_1_3 * (k-675) + a_2_3 * ((k-675)**2) + a_3_3 * ((k-675)**3)
	        
	    if(k >= 925 and k < 975):
	        foot_pos_z_left.data[k]= a_0_4 + a_1_4 * (k-675) + a_2_4 * ((k-675)**2) + a_3_4 * ((k-675)**3)
	        
	    if(k >= 975 and k < 1500):
	        foot_pos_z_left.data[k]= 0

	while not rospy.is_shutdown():


		# splines_z_feet_left_right.data[0] = 0.5
		# splines_z_feet_left_right.data[1] = 0.6

		pub_foot_right.publish(foot_pos_z_right)
		pub_foot_left.publish(foot_pos_z_right)
		print(foot_pos_z_right.data, foot_pos_z_left.data)
		rate.sleep()

if __name__ == '__main__':
    try:
        splines()
    except rospy.ROSInterruptException:
        pass

# import rospy
# from std_msgs.msg import String

# def talker():
#     pub = rospy.Publisher('chatter', String, queue_size=10)
#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         hello_str = "hello world %s" % rospy.get_time()
#         rospy.loginfo(hello_str)
#         pub.publish(hello_str)
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         talker()
#     except rospy.ROSInterruptException:
#         pass