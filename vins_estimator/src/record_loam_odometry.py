#!/usr/bin/env python
# Autor> Clayder Gonzalez

import time
import rospy
import math
import numpy as np
import tf

from nav_msgs.msg import Odometry     
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

import matplotlib
import matplotlib.pyplot as plt
import math

trajectoryFile = None
timeStamp = None
pitch = None

def odom_callback_loam(data):
	global trajectoryFile
	global timeStamp
	global pitch
	timeStampNum = data.header.stamp.secs + (data.header.stamp.nsecs * 10**(-9))
	timeStamp = str(timeStampNum)
	quaternion = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	roll = euler[0]
	pitch = euler[1]
	yaw = euler[2]
	trajectoryFile.write(timeStamp + " " + str(data.pose.pose.position.z) + " " + str(data.pose.pose.position.x) +" "+str(data.pose.pose.position.y) + " " + str(data.pose.pose.orientation.z) +" "+str(data.pose.pose.orientation.x)+" "+str(data.pose.pose.orientation.y)+" "+str(data.pose.pose.orientation.w)+ "\n")	
        #trajectoryFile.write(timeStamp + " " + str(data.pose.pose.position.x) + " " + str(data.pose.pose.position.y) +" "+str(0) + " " + str(0) +" "+str(0)+" "+str(0)+" "+str(0)+ "\n")	
        #trajectoryFile.write(timeStamp + " " + str(data.pose.pose.position.z) + " " + str(data.pose.pose.position.x) + " " + str(pitch) + "\n")	

def writeFile(file):
	global trajectoryFile
	# inicializa nodo ROS
	rospy.init_node('record_loam_odometry');
	trajectoryFile = open(file, 'w')

	rospy.Subscriber('/aft_mapped_to_init_high_frec', Odometry, odom_callback_loam)
	rospy.spin()
	trajectoryFile.close()
        
if __name__ == '__main__':
	writeFile('result_tum_01.txt')
