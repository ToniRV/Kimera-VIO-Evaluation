import rosbag
import re
from cv_bridge import CvBridge
import cv2
import numpy as np
import pandas as pd
import tf2_msgs.msg
import geometry_msgs.msg
import rospy
from nav_msgs.msg import Odometry

def gt_odom_msg_from_csv(path_to_csv, frame_id, child_frame_id):
    trajectory_df = pd.read_csv(path_to_csv)
    trajectory = []
    for index, odom_packet in trajectory_df.iterrows():
	odom = Odometry()
	odom.header.stamp = rospy.Time(odom_packet['#timestamp']/1e9)
	odom.header.frame_id = frame_id
	odom.child_frame_id = child_frame_id
	odom.pose.pose.position.x = odom_packet['x']
	odom.pose.pose.position.y = odom_packet['y']
	odom.pose.pose.position.z = odom_packet['z']
	odom.pose.pose.orientation.w = odom_packet['qw']
	odom.pose.pose.orientation.x = odom_packet['qx']
	odom.pose.pose.orientation.y = odom_packet['qy']
	odom.pose.pose.orientation.z = odom_packet['qz']
	odom.twist.twist.linear.x = odom_packet['vx']
	odom.twist.twist.linear.y = odom_packet['vy']
	odom.twist.twist.linear.z = odom_packet['vz']
	trajectory.append(odom)
    return trajectory


gt_world_frame = 'odom'
gt_body_frame = 'base_link'
gt_topic = 'gt_odom_oriented_with_velocity'
prev_t = rospy.Time(0)
gt_arr_index = 0
with rosbag.Bag(r'/home/andrew/Documents/SPARK/JackRabbot/rosbags/TressiderWithGTIMUNoisyAndGTVel.bag', 'w') as outbag:
    gt_arr = gt_odom_msg_from_csv(r'../csv/traj_gt_oriented_with_velocity.csv', gt_world_frame, gt_body_frame)
    for topic, msg, t in rosbag.Bag(r'/home/andrew/Documents/SPARK/JackRabbot/rosbags/TressiderWithGTIMUNoisy.bag').read_messages():
	while(gt_arr_index < len(gt_arr) and prev_t <= gt_arr[gt_arr_index].header.stamp and gt_arr[gt_arr_index].header.stamp <= t):
	    outbag.write(gt_topic, gt_arr[gt_arr_index], gt_arr[gt_arr_index].header.stamp)
            # print "Interjected IMU message with t: {}".format(gt_arr[gt_arr_index].header.stamp)
	    gt_arr_index = gt_arr_index + 1
	outbag.write(topic, msg, t)
        # print "Wrote message with t: {}".format(t)
    while(gt_arr_index < len(gt_arr)):
	outbag.write(gt_topic, gt_arr[gt_arr_index], gt_arr[gt_arr_index].header.stamp)
	gt_arr_index = gt_arr_index + 1
