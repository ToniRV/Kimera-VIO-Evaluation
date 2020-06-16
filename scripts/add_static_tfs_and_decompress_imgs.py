import rosbag
import re
from cv_bridge import CvBridge
import cv2
import numpy as np
import pandas as pd
import tf2_msgs.msg
import geometry_msgs.msg
import struct
import os

def write_out_compressed_depth(msg):
    print "Decoding depth..."
    # 'msg' as type CompressedImage
    depth_fmt, compr_type = msg.format.split(';')
    print "Depth format: {}".format(depth_fmt)
    # remove white space
    depth_fmt = depth_fmt.strip()
    compr_type = compr_type.strip()
    if compr_type != "compressedDepth":
        raise Exception("Compression type is not 'compressedDepth'."
                    "You probably subscribed to the wrong topic.")

    # remove header from raw data
    depth_header_size = 12
    raw_data = msg.data[depth_header_size:]

    depth_img_raw = cv2.imdecode(np.fromstring(raw_data, np.uint8), cv2.IMREAD_UNCHANGED)
    if depth_img_raw is None:
	# probably wrong header size
	raise Exception("Could not decode compressed depth image."
                        "You may need to change 'depth_header_size'!")

    if depth_fmt == "16UC1":
	# write raw image data
	cv2.imwrite(os.path.join(path_depth, "depth_" + str(msg.header.stamp) + ".png"), depth_img_raw)
    elif depth_fmt == "32FC1":
	raw_header = msg.data[:depth_header_size]
	# header: int, float, float
	[compfmt, depthQuantA, depthQuantB] = struct.unpack('iff', raw_header)
        print [compfmt, depthQuantA, depthQuantB]
	depth_img_scaled = depthQuantA / (depth_img_raw.astype(np.float32)-depthQuantB)
	# filter max values
	depth_img_scaled[depth_img_raw==0] = 0

	# depth_img_scaled provides distance in meters as f32
	# for storing it as png, we need to convert it to 16UC1 again (depth in mm)
	depth_img_mm = (depth_img_scaled*1000).astype(np.uint16)
	cv2.imwrite(os.path.join(path_depth, "depth_" + str(msg.header.stamp) + ".png"), depth_img_mm)
    else:
	raise Exception("Decoding of '" + depth_fmt + "' is not implemented!")

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

def decompress_depth_msg(msg):
    print "stub"

path_depth = ""
bridge = CvBridge()
first_message = True

with rosbag.Bag('TressiderWithStaticTfsAndDecompressedImgs.bag', 'w') as outbag:
    for topic, msg, t in rosbag.Bag('tressider-2019-04-26_0.bag').read_messages():
	if(first_message):
	    tf_array = static_transform_msg_from_csv('relevant_static_tfs.csv')
	    outbag.write('/tf', tf_array, t)
	    first_message = False
        if re.search("camera/rgb/image_rect_color/compressed", topic) != None: 
	    print "Found a compressed color image!"
	    decompressed_topic = "camera/rgb/image_rect_color"
	    np_arr = np.fromstring(msg.data, np.uint8)
	    cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
	    decompressed_msg = bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
	    decompressed_msg.header = msg.header
            outbag.write(decompressed_topic, decompressed_msg, t)
        elif re.search("camera/depth/image/compressedDepth", topic) != None: 
	    print "Found a compressed depth image!"
	    decompressed_topic = "camera/depth/image"
	    write_out_compressed_depth(msg)
	    depth_header_size = 12
	    raw_data = msg.data[depth_header_size:]
	    np_arr = np.fromstring(raw_data, np.uint8)
	    cv_img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
	    decompressed_msg = bridge.cv2_to_imgmsg(cv_img, encoding="mono8")
	    decompressed_msg.header = msg.header
	    decompressed_msg.format = msg.format
            outbag.write(decompressed_topic, decompressed_msg, t)
	    break
	else:
	    outbag.write(topic, msg, t)
