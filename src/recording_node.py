#!/usr/bin/env python

import roslib
roslib.load_manifest('pose_publisher')
import rospy, numpy
import roslib, roslib.message

from geometry_msgs.msg import Vector3
from pose_publisher.msg import HandsTwist

matrix = []
last = []

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "Recording %s", data.right_linear)

    current = [data.right_linear.x, data.right_linear.y, data.right_linear.z,
data.left_linear.x, data.left_linear.y, data.left_linear.z]

    global last

    if not (numpy.array_equal(current, last)):
        matrix.append(current)
        last = current

def listener():

    rospy.Subscriber('velocities', HandsTwist, callback)
    rospy.spin()   

if __name__ == '__main__':

    rospy.init_node('pose_recording', anonymous=True)
    listener()
    numpy.savetxt('hands_recording', matrix)
    print matrix
