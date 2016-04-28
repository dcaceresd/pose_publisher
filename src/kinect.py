#!/usr/bin/env python 


"""
Module to connect to a Kinect v2 through ROS + OpenNI2 and access the skeleton postures.
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


class Kinect:

    def __init__(self, name='pose_publisher_node', user=1):
        rospy.init_node(name, anonymous=True)
        self.listener = tf.TransformListener()
        self.user = user

    
    def get_posture(self):
        """Returns a list of frames constituted by a translation matrix
        and a rotation matrix.
        Raises IndexError when a frame can't be found (which happens if
        the requested user is not calibrated).
        """
        try:
            frames = []
            for frame in FRAMES:
                self.listener.waitForTransform(BASE_FRAME, "tracker/user_%d/%s" % (self.user, frame), rospy.Time(), rospy.Duration(4.0)
                (trans, rot) = self.listener.lookupTransform(BASE_FRAME,"tracker/user_%d/%s" % (self.user, frame), LAST)
                frames.append((trans, rot))
            return frames
        except (tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException):
            raise IndexError