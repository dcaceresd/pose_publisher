#!/usr/bin/env python 

"""Module to publish the necesary user data
"""

import roslib
roslib.load_manifest('pose_publisher')
import rospy
from kinect_posture import User

if __name__ == '__main__':

	pos = User()
	rospy.loginfo('Initializing node...')

	users = 10
	
	"""Creates an array to store the last posture of each user"""
	lastPosture = []
	for i in range(users):
   		lastPosture.append([])
    		for j in range(15):
       			lastPosture[i].append(None)

	rate = rospy.Rate(1)    

	while not rospy.is_shutdown():
		try:
			for i in range(users):
				pos.setUser(str(i+1))

				if pos.userExistance():
					if not (lastPosture[i] == pos.getPosture()):
						rospy.loginfo('User: ' + str(i+1) + ' moving!')	

					lastPosture[i] = pos.getPosture()
					
			rate.sleep()

		except:
			rospy.loginfo('Unexpected error')
