#!/usr/bin/env python

"""Module to connect a Kinect v2 through ROS + OpenNI2"""

import roslib
roslib.load_manifest('pose_publisher')
import rospy
import tf
import roslib, roslib.message

from geometry_msgs.msg import Twist, Vector3
from pose_publisher.msg import SkeletonData
 
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
		self.pub = rospy.Publisher('velocities', SkeletonData, queue_size=10)
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

	
	def getData(self):

		try:

		
			t = rospy.Time(0)
			self.listener.waitForTransform(BASE_FRAME, 

				'tracker/user_{}/head'.format(self.user),t, rospy.Duration(10.0))

			(trans1, rot1) = self.listener.lookupTransform(BASE_FRAME,
				'tracker/user_{}/right_hand'.format(self.user), LAST)
			(trans2, rot2) = self.listener.lookupTransform(BASE_FRAME,
				'tracker/user_{}/left_hand'.format(self.user), LAST)
			(trans3, rot3) = self.listener.lookupTransform(BASE_FRAME,
				'tracker/user_{}/right_elbow'.format(self.user), LAST)
			(trans4, rot4) = self.listener.lookupTransform(BASE_FRAME,
				'tracker/user_{}/left_elbow'.format(self.user), LAST)

			(matrix1, matrix2) = self.listener.lookupTwist(
				'tracker/user_{}/right_hand'.format(self.user),
				'tracker/user_{}/{}'.format(self.user, REFERENCE_FRAME), t, rospy.Duration(1))
			(matrix3, matrix4) = self.listener.lookupTwist(
				'tracker/user_{}/left_hand'.format(self.user),
				'tracker/user_{}/{}'.format(self.user, REFERENCE_FRAME), t, rospy.Duration(1))
			
			(matrix5, matrix6) = self.listener.lookupTwist(
				'tracker/user_{}/right_elbow'.format(self.user),
				'tracker/user_{}/{}'.format(self.user, REFERENCE_FRAME), t, rospy.Duration(1))
			(matrix7, matrix8) = self.listener.lookupTwist(
				'tracker/user_{}/left_elbow'.format(self.user),
				'tracker/user_{}/{}'.format(self.user, REFERENCE_FRAME), t, rospy.Duration(1))
				

			matrix = (trans1, matrix1, matrix2, trans3, matrix5, matrix6, trans2, matrix3, matrix4, trans4, matrix7, matrix8)

			
			right_hand_pos = Vector3(trans1[0], trans1[1], trans1[2])
			left_hand_pos = Vector3(trans2[0], trans2[1], trans2[2])
			right_elbow_pos = Vector3(trans3[0], trans3[1], trans3[2])
			left_elbow_pos = Vector3(trans4[0], trans4[1], trans4[2])


			right_hand_linear = Vector3(matrix1[0], matrix1[1], matrix1[2])
			
			right_hand_angular = Vector3(matrix2[0], matrix2[1], matrix2[2])

			
			left_hand_linear = Vector3(matrix3[0], matrix3[1], matrix3[2])
			
			left_hand_angular = Vector3(matrix4[0], matrix4[1], matrix4[2])


			right_elbow_linear = Vector3(matrix5[0], matrix5[1], matrix5[2])
			
			right_elbow_angular = Vector3(matrix6[0], matrix6[1], matrix6[2])
			
	
			left_elbow_linear = Vector3(matrix7[0], matrix7[1], matrix7[2])
			
			left_elbow_angular = Vector3(matrix8[0], matrix8[1], matrix8[2])
			

			if not matrix == None:
				self.pub.publish(right_hand_pos,right_hand_linear, right_hand_angular, 
					right_elbow_pos, right_elbow_linear, right_elbow_angular,
					left_hand_pos, left_hand_linear, left_hand_angular,
					left_elbow_pos, left_elbow_linear, left_elbow_angular)
				
			return matrix
			
		except (tf.ExtrapolationException):
			pass

	def getAllData(self):
		try:
			frames = []
			skeleton = SkeletonData()
			t = rospy.Time(0)

			for frame in FRAMES:

				self.listener.waitForTransform(BASE_FRAME, 
					'tracker/user_{}/{}'.format(self.user, frame), rospy.Time(), 
					rospy.Duration(10.0))
				
				(trans, rot) = self.listener.lookupTransform(BASE_FRAME,
					'tracker/user_{}/{}'.format(self.user, frame), LAST)

				(linear, angular) = self.listener.lookupTwist(
				'tracker/user_{}/{}'.format(self.user, frame),
				'tracker/user_{}/{}'.format(self.user, REFERENCE_FRAME), t, rospy.Duration(1))
				
				frames.append((trans, rot, linear, angular))

				vtrans = Vector3(trans[0], trans[1], trans[2])
				vlinear= Vector3(linear[0], linear[1], linear[2])
				vangular = Vector3(angular[0], angular[1], angular[2])
	
				setattr(skeleton, '{}_pos'.format(frame), vtrans)
				setattr(skeleton, '{}_linear'.format(frame), vlinear)
				setattr(skeleton, '{}_angular'.format(frame), vangular)

			self.pub.publish(skeleton)
			return skeleton.head_linear

		except (tf.ExtrapolationException):
			pass

