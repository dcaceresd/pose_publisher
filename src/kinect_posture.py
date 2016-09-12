#!/usr/bin/env python

"""Module to connect a Kinect v2 through ROS + OpenNI2"""

import roslib
roslib.load_manifest('pose_publisher')
import rospy
import tf
import roslib, roslib.message

from geometry_msgs.msg import Twist, Vector3
from pose_publisher.msg import HandsTwist
 
BASE_FRAME = 'tracker_depth_frame'
REFERENCE_FRAME = 'torso'
FRAMES = [
	'head',
	'neck',
	'torso',
	'left_shoulder',
	'left_elbow',
	'left_hand',
	'left_hip',
	'left_knee',
	'left_foot',
	'right_shoulder',
	'right_elbow',
	'right_hand',
	'right_hip',
	'right_knee',
	'right_foot',
	]
LAST = rospy.Duration()


class User:
	
	"""Creates new users to store NiTE positions"""

	def __init__(self, name='kinect_posture', user=1):
		self.pub = rospy.Publisher('velocities', HandsTwist, queue_size=10)
		rospy.init_node(name, anonymous=True)
		self.listener = tf.TransformListener()
		self.user = user

	def getUser(self):
		"""Funtion to get the number of the actual user
		
		Returns:
			The user number definined by NITE
		
		"""
		return self.user

	def setUser(self, n):
		"""Function to change the actual user
		
		Args:
			n (int): The number of the new user
			
		"""
		self.user = n

	def userExistance(self):
		"""Function to determine if a user exists
		
		Returns:
			True for success, False otherwise
			
		"""
		if self.listener.frameExists('/tracker/user_{}/head'.format(self.user)):
			return True
		else:
			return False

	    
	def getPosture(self):
		"""Function to store the frame transformations and rotations of a user
		
		Args:
			frames: matrix that contains the trans and rot
			trans: position vector of the frame
			rot: rotation vector of the frame
				
		Returns:
			a translation and a rotation matrix of frames
			
        	"""
		try:
			frames = []
			for frame in FRAMES:

				self.listener.waitForTransform(BASE_FRAME, 
					'tracker/user_{}/{}'.format(self.user, frame), rospy.Time(), 
					rospy.Duration(10.0))
				
				(trans, rot) = self.listener.lookupTransform(BASE_FRAME,
					'tracker/user_{}/{}'.format(self.user, frame), LAST)
				
				frames.append((trans, rot))
			return frames

		except (tf.LookupException, tf.ConnectivityException, 
			tf.ExtrapolationException):
			rospy.loginfo('Unable to perform the tranformation!')
			
	def getVelocities(self):
		"""Function that returns the linear and angular velocities 
		
		Returns: 
			a matrix with the velocity values of the user
			
		"""
		
		try:
			t = rospy.Time(0)
			self.listener.waitForTransform(BASE_FRAME, 
				'tracker/user_{}/head'.format(self.user),t, rospy.Duration(10.0))
			(matrix1, matrix2) = self.listener.lookupTwist(
				'tracker/user_{}/right_hand'.format(self.user),
				'tracker/user_{}/{}'.format(self.user, REFERENCE_FRAME), t, rospy.Duration(1))
			(matrix3, matrix4) = self.listener.lookupTwist(
				'tracker/user_{}/left_hand'.format(self.user),
				'tracker/user_{}/{}'.format(self.user, REFERENCE_FRAME), t, rospy.Duration(1))
				
			matrix = (matrix1, matrix2, matrix3, matrix4)
			
			right_linear = Vector3(matrix1[0], matrix1[1], matrix1[2])
			#v1.x = matrix1[0]
			#v1.y = matrix1[1]
			#v1.z = matrix1[2]
			
			right_angular = Vector3(matrix2[0], matrix2[1], matrix2[2])
			

			left_linear = Vector3(matrix3[0], matrix3[1], matrix3[2])
			
			left_angular = Vector3(matrix4[0], matrix4[1], matrix4[2])
			

			if not matrix == None:
				self.pub.publish(right_linear, right_angular, left_linear, left_angular)
				
			return matrix
			
		except (tf.ExtrapolationException):
			pass
