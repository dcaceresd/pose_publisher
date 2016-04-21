#!/usr/bin/env python 


"""
Node for pose recognition
"""


import roslib
roslib.load_manifest('pose_publisher')
import rospy
import tf


BASE_FRAME = 'tracker_depth_frame'
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


if __name__ == '__main__':
	rospy.init_node('pose_recognition_node')
	rospy.loginfo("Initializing Node...")

	listener = tf.TransformListener()

	rospy.loginfo("Waiting for tracking...")	


	r = rospy.Rate(1)
	while not rospy.is_shutdown():
		try:
			listener.waitForTransform(BASE_FRAME, 'tracker/user_1/right_hand', rospy.Time(), rospy.Duration(60.0))

			
			(trans, rot) = listener.lookupTransform(BASE_FRAME, 'tracker/user_1/right_hand', rospy.Time())
			(trans2, rot2) = listener.lookupTransform(BASE_FRAME, 'tracker/user_1/left_hand', rospy.Time())

        	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):

            		continue
		
		"""
		Logic to determine which hand is higher to the ground
		"""
		if trans[1] > (trans2[1] + 0.05):
			rospy.loginfo("RIGHT HAND")
		else:
			if trans2[1] > (trans[1] + 0.05):
				rospy.loginfo("LEFT HAND")
			else:
				rospy.loginfo("HANDS AT THE SAME HEIGHT")
		
		r.sleep()
