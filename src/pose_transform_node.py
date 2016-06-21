#!/usr/bin/env python 

import roslib
roslib.load_manifest('pose_publisher')
import rospy
from kinect_posture import User
import tf

if __name__ == '__main__':
	
	pos = User()
	rospy.loginfo('Initializing node...')
	listener2 = tf.TransformListener()

	rate = rospy.Rate(1)

	while not rospy.is_shutdown():
		n = 0

		if listener2.frameExists('/tracker/user_1/head'):
			for i in range(10):
				if listener2.frameExists('/tracker/user_{}/head'.format(str(i))):
					n+=1

			if n != 0:
				for i in xrange(n):
					pos.setUser(str(i+1))
					print pos.getPosture()


		else:
			rospy.loginfo('User not detected')

		rate.sleep()

