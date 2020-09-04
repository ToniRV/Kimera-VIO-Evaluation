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

def transform_msg_from_csv(path_to_csv, frame_id, child_frame_id):
    tfs = pd.read_csv(path_to_csv)
    tf_array = []
    for index, tf in tfs.iterrows():
	tf_msg = tf2_msgs.msg.TFMessage()
	tf_stamped = geometry_msgs.msg.TransformStamped()
	tf_stamped.header.frame_id = frame_id
	tf_stamped.child_frame_id = child_frame_id
	tf_stamped.header.stamp = rospy.Time(tf['#timestamp']*1e-9) # ns to sec
	tf_stamped.transform.translation.x = tf['x']
	tf_stamped.transform.translation.y = tf['y']
	tf_stamped.transform.translation.z = tf['z']
	tf_stamped.transform.rotation.x = tf['qx']
	tf_stamped.transform.rotation.y = tf['qy']
	tf_stamped.transform.rotation.z = tf['qz']
	tf_stamped.transform.rotation.w = tf['qw']
	tf_msg.transforms.append(tf_stamped)
	tf_array.append(tf_msg)
    return tf_array


tf_world_frame = 'world'
tf_body_frame = 'left_cam'
tf_topic = '/tf'
tf_arr_index = 0
tf_arr = transform_msg_from_csv(r'/home/andrew/catkin_ws/src/Kimera-VIO-ROS/output_logs/PianoOrganExperimental/traj_vio.csv', tf_world_frame, tf_body_frame)
with rosbag.Bag(r'/home/andrew/Documents/SPARK/RealSense/Church/PianoOrganNonOptimized.bag', 'w') as outbag:
    for topic, msg, t in rosbag.Bag(r'/home/andrew/Documents/SPARK/RealSense/Church/PianoOrganSynchronized.bag').read_messages():
	while(tf_arr_index < len(tf_arr) and tf_arr[tf_arr_index].transforms[0].header.stamp <= t):
	    outbag.write(tf_topic, tf_arr[tf_arr_index], tf_arr[tf_arr_index].transforms[0].header.stamp)
            # print "Interjected IMU message with t: {}".format(tf_arr[tf_arr_index].header.stamp)
	    tf_arr_index = tf_arr_index + 1
	outbag.write(topic, msg, t)
        # print "Wrote message with t: {}".format(t)
    while(tf_arr_index < len(tf_arr)):
	outbag.write(tf_topic, tf_arr[tf_arr_index], tf_arr[tf_arr_index].transforms[0].header.stamp)
	tf_arr_index = tf_arr_index + 1
