import rosbag
import re
from cv_bridge import CvBridge
import cv2
import numpy as np
from kvh_msgs.msg import Kvh

def interpolate(from_thing, to_thing, interp_factor):
    return from_thing + interp_factor*(to_thing - from_thing)

def normalize_quat(quaternion):
    magnitude = quaternion.x*quaternion.x + quaternion.y*quaternion.y + quaternion.z*quaternion.z + quaternion.w*quaternion.w
    magnitude = np.sqrt(magnitude)
    normalized = quaternion
    normalized.x = quaternion.x / magnitude
    normalized.y = quaternion.y / magnitude
    normalized.z = quaternion.z / magnitude
    normalized.w = quaternion.w / magnitude
    return normalized

def interpolate_quat(from_quat, to_quat, interp_factor):
    interpolated = from_quat
    interpolated.x = interpolate(from_quat.x, to_quat.x, interp_factor)
    interpolated.y = interpolate(from_quat.y, to_quat.y, interp_factor)
    interpolated.z = interpolate(from_quat.z, to_quat.z, interp_factor)
    interpolated.w = interpolate(from_quat.w, to_quat.w, interp_factor)
    return normalize_quat(interpolated)

def interpolate_vector3(from_vec, to_vec, interp_factor):
    interpolated = from_vec
    interpolated.x = interpolate(from_vec.x, to_vec.x, interp_factor)
    interpolated.y = interpolate(from_vec.y, to_vec.y, interp_factor)
    interpolated.z = interpolate(from_vec.z, to_vec.z, interp_factor)
    return interpolated

imu_multiplier = 10
prev_imu_msg = None
prev_t = None
with rosbag.Bag('TressiderRotatedOccam01WithInterIMU.bag', 'w') as outbag:
    for topic, msg, t in rosbag.Bag('TressiderRotatedOccam01.bag').read_messages():
        # This also replaces tf timestamps under the assumption 
        # that all transforms in the message share the same timestamp
        if re.search("imu/data", topic) != None: 
	    if prev_imu_msg != None:
                for i in range(imu_multiplier):
		    if(i == 0):
			continue
		    interp_factor = i / float(imu_multiplier)
		    interp_t = prev_t + interp_factor * (t - prev_t)
		    interp_msg = prev_imu_msg
		    interp_msg.header.stamp = interpolate(\
			prev_imu_msg.header.stamp, msg.header.stamp, interp_factor)
		    interp_msg.orientation = interpolate_quat(\
			prev_imu_msg.orientation, msg.orientation, interp_factor)
		    interp_msg.angular_velocity = interpolate_vector3(\
			prev_imu_msg.angular_velocity, msg.angular_velocity, interp_factor)
		    interp_msg.linear_acceleration = interpolate_vector3(\
			prev_imu_msg.linear_acceleration, msg.linear_acceleration, interp_factor)
		    outbag.write(topic, interp_msg, interp_t)
	    prev_imu_msg = msg
	    prev_t = t
	outbag.write(topic, msg, t)
