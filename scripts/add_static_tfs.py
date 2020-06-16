import rosbag
import re
from cv_bridge import CvBridge
import cv2
import numpy as np
import pandas as pd
import tf2_msgs.msg
import geometry_msgs.msg
import rospy

def static_transform_msg_from_csv(path_to_csv):
    static_tfs = pd.read_csv(path_to_csv)
    tf_array = tf2_msgs.msg.TFMessage()
    for index, static_tf in static_tfs.iterrows():
	tf_stamped = geometry_msgs.msg.TransformStamped()
	tf_stamped.header.frame_id = static_tf['frame_id']
	tf_stamped.child_frame_id = static_tf['child_frame_id']
	tf_stamped.header.stamp = t
	tf_stamped.transform.translation.x = static_tf['x']
	tf_stamped.transform.translation.y = static_tf['y']
	tf_stamped.transform.translation.z = static_tf['z']
	tf_stamped.transform.rotation.x = static_tf['qx']
	tf_stamped.transform.rotation.y = static_tf['qy']
	tf_stamped.transform.rotation.z = static_tf['qz']
	tf_stamped.transform.rotation.w = static_tf['qw']
	tf_array.transforms.append(tf_stamped)
    return tf_array

tf_stamped_period = rospy.Duration.from_sec(2)
tf_stamped_prev_publish = rospy.Time.from_sec(0)
with rosbag.Bag('../rosbags/TressiderWithStaticTfs.bag', 'w') as outbag:
    for topic, msg, t in rosbag.Bag('../rosbags/tressider-2019-04-26_0.bag').read_messages():
	if(t >= tf_stamped_prev_publish + tf_stamped_period and hasattr(msg, "header")):
	    tf_array = static_transform_msg_from_csv(../csv/'relevant_static_tfs.csv')
	    outbag.write('/tf', tf_array, t)
	    tf_stamped_prev_publish = t
	outbag.write(topic, msg, t)
