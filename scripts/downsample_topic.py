import rosbag
import re
from cv_bridge import CvBridge
import cv2
import numpy as np
bridge = CvBridge()


topics_to_downsample = ["/vio_camera/infra1/camera_info", \
			"/vio_camera/infra1/image_rect_raw", \
			"/vio_camera/infra2/camera_info", \
			"/vio_camera/infra2/image_rect_raw"]
downsample_rate = 100
input_bag_path = "../Calibration/VioChessboardCalibrationManyImgs.bag"
output_bag_path = "../Calibration/VioChessboardCalibrationDownSampled.bag"

# set encounter count to zero for all topics
count_dict = {}
for topic in topics_to_downsample:
    count_dict[topic] = 0

with rosbag.Bag(output_bag_path, 'w') as outbag:
    for topic, msg, t in rosbag.Bag(input_bag_path).read_messages():
	# iterate through the downsample topics until we find a match
        found_match = False
        for to_downsample in topics_to_downsample:
	    if not found_match and re.search(to_downsample, topic) != None: 
		# write out every nth message on the downsampled topic
		if count_dict[to_downsample] % downsample_rate == 0:
		    outbag.write(topic, msg, t)
		count_dict[to_downsample] = count_dict[to_downsample] + 1
		found_match = True
	# if we do not find a match, add as usual
	if not found_match:
	    outbag.write(topic, msg, t)
