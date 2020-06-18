import rosbag
import re
from cv_bridge import CvBridge
import cv2
import numpy as np
import pandas as pd
import tf2_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import rospy

def imu_msgs_from_csv(path_to_csv, frame_id):
    imu_df = pd.read_csv(path_to_csv)
    sequence = 0
    imu_arr = []
    for index, imu_packet in imu_df.iterrows():
	imu_msg = sensor_msgs.msg.Imu()
	imu_msg.header.stamp = rospy.Time(imu_packet['timestamp']/1e9)
	imu_msg.header.frame_id = frame_id
	imu_msg.header.seq = sequence
	sequence = sequence + 1
	imu_msg.orientation.w = imu_packet['qw']
	imu_msg.orientation.x = imu_packet['qx']
	imu_msg.orientation.y = imu_packet['qy']
	imu_msg.orientation.z = imu_packet['qz']
	imu_msg.angular_velocity.x = imu_packet['rvx']
	imu_msg.angular_velocity.y = imu_packet['rvy']
	imu_msg.angular_velocity.z = imu_packet['rvz']
	imu_msg.linear_acceleration.x = imu_packet['ax']
	imu_msg.linear_acceleration.y = imu_packet['ay']
	imu_msg.linear_acceleration.z = imu_packet['az']
	imu_arr.append(imu_msg)
    return imu_arr


imu_frame = 'ext_imu_frame'
imu_topic = 'imu/data_from_gt'
prev_t = rospy.Time(0)
imu_arr_index = 0
with rosbag.Bag(r'/home/andrew/Documents/SPARK/JackRabbot/rosbags/TressiderWithGTIMU.bag', 'w') as outbag:
    imu_arr = imu_msgs_from_csv('../csv/tressidor_imu_from_gyro.csv', imu_frame)
    for topic, msg, t in rosbag.Bag(r'/home/andrew/Documents/SPARK/JackRabbot/rosbags/TressiderRotatedOccam01.bag').read_messages():
	while(imu_arr_index < len(imu_arr) and prev_t <= imu_arr[imu_arr_index].header.stamp and imu_arr[imu_arr_index].header.stamp <= t):
	    outbag.write(imu_topic, imu_arr[imu_arr_index], imu_arr[imu_arr_index].header.stamp)
            # print "Interjected IMU message with t: {}".format(imu_arr[imu_arr_index].header.stamp)
	    imu_arr_index = imu_arr_index + 1
	outbag.write(topic, msg, t)
        # print "Wrote message with t: {}".format(t)
    while(imu_arr_index < len(imu_arr)):
	outbag.write(imu_topic, imu_arr[imu_arr_index], imu_arr[imu_arr_index].header.stamp)
	imu_arr_index = imu_arr_index + 1
