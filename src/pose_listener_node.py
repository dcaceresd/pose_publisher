#!/usr/bin/env python
import roslib
roslib.load_manifest('pose_publisher')
import rospy
import std_msgs.msg
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(data.data)
    
def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    rospy.loginfo("Pose Listener Node")
    listener()

