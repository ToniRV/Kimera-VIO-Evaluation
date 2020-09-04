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

input_bag_path = "../Calibration/VioChessboardCalibrationManyImgs.bag"
output_bag_path = "../Calibration/VioChessboardCalibrationDownSampled.bag"

with rosbag.Bag(output_bag_path, 'w') as outbag:
    for topic, msg, t in rosbag.Bag(input_bag_path).read_messages():
	# iterate through the downsample topics until we find a match
        found_match = False
        for to_downsample in topics_to_downsample:
	    if not found_match and re.search(to_downsample, topic) != None: 
		outbag.write(topic, msg, t)
		found_match = True
