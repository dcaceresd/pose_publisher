#!/usr/bin/env python 


"""
Node for pose publishing
"""


import roslib
roslib.load_manifest('pose_publisher')
import rospy
import tf
from std_msgs.msg import String


BASE_FRAME = 'tracker_depth_frame'


def talker():

	pub = rospy.Publisher('chatter', String, queue_size = 10)
	rospy.init_node('talker', anonymous = True)
	rospy.loginfo("Initializing Node...")

	listener = tf.TransformListener()

	rospy.loginfo("Waiting for tracking...")	

	r = rospy.Rate(1)
	while not rospy.is_shutdown():
		
		listener.waitForTransform(BASE_FRAME, 'tracker/user_1/right_hand', rospy.Time(), rospy.Duration(60.0))

			
		(trans, rot) = listener.lookupTransform(BASE_FRAME, 'tracker/user_1/right_hand', rospy.Time())
		
		#(trans2, rot2) = listener.lookupTransform(BASE_FRAME, 'tracker/user_1/left_hand', rospy.Time())

		rospy.loginfo(trans)
		coord = str(trans[0]) + ", " + str(trans[1]) + ", " + str(trans[2])
		pub.publish(coord)

		r.sleep()


if __name__ == '__main__':


		try:
			talker()

        	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):

            		pass
