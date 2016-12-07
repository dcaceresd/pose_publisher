#!/usr/bin/env python

"""Module to publish the necesary user data"""

import roslib
roslib.load_manifest('pose_publisher')
import rospy
from kinect_posture import User
from geometry_msgs.msg import Vector3
from pose_publisher.msg import SkeletonData

if __name__ == '__main__':

	pos = User()
	rospy.loginfo('Initializing node...')

	users = 10
	data_points = 45

	v = Vector3(0, 0, 0)
	#v.x = 0
	#v.y = 0
	#v.z = 0

	#m = [v, v]
	
	m = []	
	for i in range(data_points):
		m.append(v)
		
	
	"""Creates an array to store the last posture of each user"""
	lastPosture = []
	for i in range(users):
   		lastPosture.append([])
    		for j in range(15):
       			lastPosture[i].append(None)

	"""Array to store the past velocities"""
	velocities = []
 
	rate = rospy.Rate(10)    

	while not rospy.is_shutdown():
		for i in range(users):
			pos.setUser(str(i+1))

			if pos.userExistance():
				if not (lastPosture[i] == pos.getPosture()):
					rospy.loginfo('User: ' + str(i+1) + ' moving!')
					velocities.append(pos.getAllData())
					print pos.getAllData()
				else:
					velocities.append(m)
				lastPosture[i] = pos.getAllData()
		rate.sleep()
