#!/usr/bin/env python

import roslib
roslib.load_manifest('pose_publisher')
import rospy, numpy
import roslib, roslib.message
import csv

from geometry_msgs.msg import Vector3
from pose_publisher.msg import SkeletonData

matrix = []

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

"""Atrributes to arff file

@ATTRIBUTE head_pos_x NUMERIC
@ATTRIBUTE head_pos_y NUMERIC
@ATTRIBUTE head_pos_z NUMERIC

@ATTRIBUTE head_linear_x NUMERIC
@ATTRIBUTE head_linear_y NUMERIC
@ATTRIBUTE head_linear_z NUMERIC


@ATTRIBUTE neck_pos_x NUMERIC
@ATTRIBUTE neck_pos_y NUMERIC
@ATTRIBUTE neck_pos_z NUMERIC

@ATTRIBUTE neck_linear_x NUMERIC
@ATTRIBUTE neck_linear_y NUMERIC
@ATTRIBUTE neck_linear_z NUMERIC


@ATTRIBUTE torso_pos_x NUMERIC
@ATTRIBUTE torso_pos_y NUMERIC
@ATTRIBUTE torso_pos_z NUMERIC

@ATTRIBUTE torso_linear_x NUMERIC
@ATTRIBUTE torso_linear_y NUMERIC
@ATTRIBUTE torso_linear_z NUMERIC


@ATTRIBUTE left_shoulder_pos_x NUMERIC
@ATTRIBUTE left_shoulder_pos_y NUMERIC
@ATTRIBUTE left_shoulder_pos_z NUMERIC

@ATTRIBUTE left_shoulder_linear_x NUMERIC
@ATTRIBUTE left_shoulder_linear_y NUMERIC
@ATTRIBUTE left_shoulder_linear_z NUMERIC


@ATTRIBUTE left_elbow_pos_x NUMERIC
@ATTRIBUTE left_elbow_pos_y NUMERIC
@ATTRIBUTE left_elbow_pos_z NUMERIC

@ATTRIBUTE left_elbow_linear_x NUMERIC
@ATTRIBUTE left_elbow_linear_y NUMERIC
@ATTRIBUTE left_elbow_linear_z NUMERIC


@ATTRIBUTE left_hand_pos_x NUMERIC
@ATTRIBUTE left_hand_pos_y NUMERIC
@ATTRIBUTE left_hand_pos_z NUMERIC

@ATTRIBUTE left_hand_linear_x NUMERIC
@ATTRIBUTE left_hand_linear_y NUMERIC
@ATTRIBUTE left_hand_linear_z NUMERIC


@ATTRIBUTE left_hip_pos_x NUMERIC
@ATTRIBUTE left_hip_pos_y NUMERIC
@ATTRIBUTE left_hip_pos_z NUMERIC

@ATTRIBUTE left_hip_linear_x NUMERIC
@ATTRIBUTE left_hip_linear_y NUMERIC
@ATTRIBUTE left_hip_linear_z NUMERIC


@ATTRIBUTE left_knee_pos_x NUMERIC
@ATTRIBUTE left_knee_pos_y NUMERIC
@ATTRIBUTE left_knee_pos_z NUMERIC

@ATTRIBUTE left_knee_linear_x NUMERIC
@ATTRIBUTE left_knee_linear_y NUMERIC
@ATTRIBUTE left_knee_linear_z NUMERIC


@ATTRIBUTE left_foot_pos_x NUMERIC
@ATTRIBUTE left_foot_pos_y NUMERIC
@ATTRIBUTE left_foot_pos_z NUMERIC

@ATTRIBUTE left_foot_linear_x NUMERIC
@ATTRIBUTE left_foot_linear_y NUMERIC
@ATTRIBUTE left_foot_linear_z NUMERIC


@ATTRIBUTE right_shoulder_pos_x NUMERIC
@ATTRIBUTE right_shoulder_pos_y NUMERIC
@ATTRIBUTE right_shoulder_pos_z NUMERIC

@ATTRIBUTE right_shoulder_linear_x NUMERIC
@ATTRIBUTE right_shoulder_linear_y NUMERIC
@ATTRIBUTE right_shoulder_linear_z NUMERIC


@ATTRIBUTE right_elbow_pos_x NUMERIC
@ATTRIBUTE right_elbow_pos_y NUMERIC
@ATTRIBUTE right_elbow_pos_z NUMERIC

@ATTRIBUTE right_elbow_linear_x NUMERIC
@ATTRIBUTE right_elbow_linear_y NUMERIC
@ATTRIBUTE right_elbow_linear_z NUMERIC


@ATTRIBUTE right_hand_pos_x NUMERIC
@ATTRIBUTE right_hand_pos_y NUMERIC
@ATTRIBUTE right_hand_pos_z NUMERIC

@ATTRIBUTE right_hand_linear_x NUMERIC
@ATTRIBUTE right_hand_linear_y NUMERIC
@ATTRIBUTE right_hand_linear_z NUMERIC


@ATTRIBUTE right_hip_pos_x NUMERIC
@ATTRIBUTE right_hip_pos_y NUMERIC
@ATTRIBUTE right_hip_pos_z NUMERIC

@ATTRIBUTE right_hip_linear_x NUMERIC
@ATTRIBUTE right_hip_linear_y NUMERIC
@ATTRIBUTE right_hip_linear_z NUMERIC


@ATTRIBUTE right_knee_pos_x NUMERIC
@ATTRIBUTE right_knee_pos_y NUMERIC
@ATTRIBUTE right_knee_pos_z NUMERIC

@ATTRIBUTE right_knee_linear_x NUMERIC
@ATTRIBUTE right_knee_linear_y NUMERIC
@ATTRIBUTE right_knee_linear_z NUMERIC


@ATTRIBUTE right_foot_pos_x NUMERIC
@ATTRIBUTE right_foot_pos_y NUMERIC
@ATTRIBUTE right_foot_pos_z NUMERIC

@ATTRIBUTE right_foot_linear_x NUMERIC
@ATTRIBUTE right_foot_linear_y NUMERIC
@ATTRIBUTE right_foot_linear_z NUMERIC


@ATTRIBUTE right_foot_pos_x NUMERIC
@ATTRIBUTE right_foot_pos_y NUMERIC
@ATTRIBUTE right_foot_pos_z NUMERIC

@ATTRIBUTE right_foot_linear_x NUMERIC
@ATTRIBUTE right_foot_linear_y NUMERIC
@ATTRIBUTE right_foot_linear_z NUMERIC


"""

last = ()

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "Recording %s", data.right_hand_linear)
	
	current = ()
	global last
	for frame in FRAMES:

		frame_pos = getattr(data, '{}_pos'.format(frame))
		frame_linear = getattr(data, '{}_linear'.format(frame))
		#frame_angular = getattr(data, '{}_angular'.format(frame))
	
		current += frame_pos.x, frame_pos.y, frame_pos.z, frame_linear.x, frame_linear.y, frame_linear.z
	
	if not (current == last):

		csvfile.writerow(current)	
        last = current

def listener():

	rospy.Subscriber('velocities', SkeletonData, callback)
	rospy.spin()   

if __name__ == '__main__':

	f = open('test.csv', 'wt')
	csvfile = csv.writer(f)
	rospy.init_node('pose_recording', anonymous=True)
	listener()
	#rospy.Subscriber('velocities', SkeletonData, callback)
	f.close()
