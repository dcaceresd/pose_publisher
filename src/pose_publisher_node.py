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
LAST = rospy.Duration()

if __name__ == '__main__':
	rospy.init_node('pose_recognition_node')
	rospy.loginfo("Initializing Node...")

	listener = tf.TransformListener()
	rospy.loginfo("Waiting for transform")	
	listener.waitForTransform(BASE_FRAME, 'tracker/user_1/right_hand', rospy.Time(), rospy.Duration(40.0))
	rospy.loginfo("Checked!")

	r = rospy.Rate(1)
	while not rospy.is_shutdown():
		try:
			listener.waitForTransform(BASE_FRAME, 'tracker/user_1/right_hand', rospy.Time(), rospy.Duration(4.0))			
			trans, rot = listener.lookupTransform(BASE_FRAME, 'tracker/user_1/right_hand', rospy.Time())

        	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):

            		continue
		
		x = trans[0]
		y = trans[1]
		z = trans[2]


		
		rospy.loginfo(str(x) +", "+str(y))
		r.sleep()
