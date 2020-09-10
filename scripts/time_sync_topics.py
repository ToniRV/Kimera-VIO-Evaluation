import rosbag
import re
from cv_bridge import CvBridge
import cv2
import numpy as np
bridge = CvBridge()


topics_to_sync = ["/depth_camera/color/camera_info", \
		  "/depth_camera/color/image_raw", \
		  "/depth_camera/depth/camera_info", \
		  "/depth_camera/depth/image_rect_raw", \
		  "/depth_camera/aligned_depth_to_color/camera_info", \
		  "/depth_camera/aligned_depth_to_color/image_raw"]
timer_topic = "/vio_camera/infra1/camera_info"
input_bag_path = "/home/andrew/DemoForNathan.bag"
output_bag_path = "/home/andrew/DemoForNathanSynchronized.bag"

# find timestamp of first message of each topic
print "Reading messages!"
first_msg_stamp_dict = {}
all_msgs = []
for topic, msg, t in rosbag.Bag(input_bag_path).read_messages():
    for to_sync in topics_to_sync:
	if re.search(to_sync, topic) != None and not topic in first_msg_stamp_dict: 
	    first_msg_stamp_dict[topic] = msg.header.stamp
    if re.search(timer_topic, topic) != None and not topic in first_msg_stamp_dict:
        first_msg_stamp_dict[timer_topic] = msg.header.stamp
    # align bag time to header time
    if topic == "/tf" and msg.transforms:
	all_msgs.append((topic, msg, msg.transforms[0].header.stamp))
    elif hasattr(msg, "header"):
	all_msgs.append((topic, msg, msg.header.stamp))
    else:
	all_msgs.append((topic, msg, t))

print "Synchronizing messages!"
for i in range(len(all_msgs)):
    old_msg = all_msgs[i]
    found_match = False
    for to_sync in topics_to_sync:
	if not found_match and re.search(to_sync, old_msg[0]) != None:
	    old_stamp = old_msg[1].header.stamp
	    delta_stamp = first_msg_stamp_dict[timer_topic] - first_msg_stamp_dict[to_sync]
	    new_stamp = old_stamp + delta_stamp
	    new_ros_msg = old_msg[1]
	    new_ros_msg.header.stamp = new_stamp
	    new_msg = (old_msg[0], new_ros_msg, new_stamp)
	    all_msgs[i] = new_msg
	    found_match = True

print "Sorting messages!"
all_msgs.sort(key=lambda x: x[1].header.stamp.to_nsec())

print "Writing messages!"
with rosbag.Bag(output_bag_path, 'w') as outbag:
    for topic, msg, t in all_msgs:
	outbag.write(topic, msg, t)
