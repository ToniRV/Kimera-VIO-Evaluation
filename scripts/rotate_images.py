import rosbag
import re
from cv_bridge import CvBridge
import cv2
import numpy as np
bridge = CvBridge()

with rosbag.Bag('TressiderRotatedOccam01.bag', 'w') as outbag:
    for topic, msg, t in rosbag.Bag('tressider-2019-04-26_0.bag').read_messages():
        # This also replaces tf timestamps under the assumption 
        # that all transforms in the message share the same timestamp
        if re.search("ros_indigosdk_node/image0/compressed", topic) != None: 
	    rotated_topic = "ros_indigosdk_node/image0_rotated"
	    np_arr = np.fromstring(msg.data, np.uint8)
	    cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
	    rotated_img = cv2.rotate(cv_img, cv2.ROTATE_90_COUNTERCLOCKWISE)
	    rotated_msg = bridge.cv2_to_imgmsg(rotated_img, encoding="bgr8")
	    rotated_msg.header = msg.header
            outbag.write(rotated_topic, rotated_msg, t)
        elif re.search("ros_indigosdk_node/image1/compressed", topic) != None: 
	    rotated_topic = "ros_indigosdk_node/image1_rotated"
	    np_arr = np.fromstring(msg.data, np.uint8)
	    cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
	    rotated_img = cv2.rotate(cv_img, cv2.ROTATE_90_COUNTERCLOCKWISE)
	    rotated_msg = bridge.cv2_to_imgmsg(rotated_img, encoding="bgr8")
	    rotated_msg.header = msg.header
            outbag.write(rotated_topic, rotated_msg, t)
	outbag.write(topic, msg, t)
