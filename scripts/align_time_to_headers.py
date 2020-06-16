import rosbag
import re

with rosbag.Bag('TressiderRotatedOccam01Aligned.bag', 'w') as outbag:
    for topic, msg, t in rosbag.Bag('TressiderRotatedOccam01.bag').read_messages():
        # This also replaces tf timestamps under the assumption 
        # that all transforms in the message share the same timestamp
        if topic == "/tf" and msg.transforms:
            outbag.write(topic, msg, msg.transforms[0].header.stamp)
        elif hasattr(msg, "header"):
            outbag.write(topic, msg, msg.header.stamp)
	else:
	    outbag.write(topic, msg, t)
