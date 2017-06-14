#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy 
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time

def callback(joy_data):
	scale = 0.05
	#global finder_arm_pose
	if abs(joy_data.axes[0]) > 0.9:
		finder_arm_pose.position[0] += float(joy_data.axes[0]*scale)#[0]Left/Right Axis stick left

	if abs(joy_data.axes[1]) > 0.9:
		finder_arm_pose.position[1] += float(joy_data.axes[1]*scale)#[1]Up/Down Axis stick left

	if abs(joy_data.axes[3]) > 0.9:
		finder_arm_pose.position[2] += float(joy_data.axes[3]*scale)#[2]Left/Right Axis stick right

	if abs(joy_data.axes[4]) > 0.9:
		finder_arm_pose.position[3] += float(joy_data.axes[4]*scale)#[3]Up/Down Axis stick right

	if abs(joy_data.axes[6]) > 0.9:
		finder_arm_pose.position[4] += float(joy_data.axes[6]*scale)#[6]cross key left/right

	if abs(joy_data.axes[7]) > 0.9:
		finder_arm_pose.position[5] += float(joy_data.axes[7]*scale)#[7]cross key up/down


	if abs(joy_data.axes[2]) > 0.9:
		finder_arm_pose.position[6] += float(joy_data.axes[2]*scale)#LT-gatillo izquierdo

	if abs(joy_data.axes[5]) > 0.9:
		finder_arm_pose.position[7] -= float(joy_data.axes[5]*scale)#RT-gatillo derecho

	finder_arm_pose.position[7] = 0.0;
	finder_arm_pose.position[8] = 0.0;
	finder_arm_pose.position[9] = 0.0;
	finder_arm_pose.position[10] = 0.0;


def start():
	rospy.Subscriber("joy",Joy,callback)
	pub = rospy.Publisher("joint_states",JointState,queue_size = 10)
	rospy.init_node("joint_state_publisher")
	rate = rospy.Rate(10)#10hz
	global finder_arm_pose
	finder_arm_pose = JointState()
	finder_arm_pose.header.frame_id = "/base_link"
	finder_arm_pose.header = Header()
	finder_arm_pose.header.stamp = rospy.Time.now()
	finder_arm_pose.name = ["right_front_arm_joint",
							"left_front_arm_joint",
							"right_back_arm_joint",
							"left_back_arm_joint",
							"base_rotation", 
							"shoulder_rotation", 
							"elbow_rotation", 
							"roll_rotation", 
							"pitch_rotation", 
							"yaw_rotation", 
							"gripper_rotation"]
	finder_arm_pose.position = [0]*11
	finder_arm_pose.velocity = []
	finder_arm_pose.effort = []
	pub.publish(finder_arm_pose)
	time.sleep(10)
	while not rospy.is_shutdown():
		finder_arm_pose.header.stamp = rospy.Time.now()
		pub.publish(finder_arm_pose)
		rate.sleep()
if __name__ == '__main__':
		start()