import rosbag
import re

def print_single_transform(transform):
    print '{},{},{},{},{},{},{},{},{},{}'.format(transform.header.stamp, transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w, transform.header.frame_id, transform.child_frame_id)

print '#timestamp,x,y,z,qx,qy,qz,qw,frame_id,child_frame_id'
for topic, msg, t in rosbag.Bag('sample_jackrabbot_bagfile.bag').read_messages():
    if re.search("tf", topic):
	for transform in msg.transforms:
	    if(transform.header.frame_id=="world" and transform.child_frame_id=="odom"):
		print_single_transform(transform)
	    if(transform.header.frame_id=="base_link" and transform.child_frame_id=="base_chassis_link"):
		print_single_transform(transform)
	    if(transform.header.frame_id=="base_chassis_link" and transform.child_frame_id=="ext_imu_frame"):
		print_single_transform(transform)
	    if(transform.header.frame_id=="base_chassis_link" and transform.child_frame_id=="lower_velodyne_frame"):
		print_single_transform(transform)
	    if(transform.header.frame_id=="base_chassis_link" and transform.child_frame_id=="upper_velodyne_frame"):
		print_single_transform(transform)
	    if(transform.header.frame_id=="tilt_link" and transform.child_frame_id=="camera_link"):
		print_single_transform(transform)
	    if(transform.header.frame_id=="camera_link" and transform.child_frame_id=="left_camera_frame"):
		print_single_transform(transform)
	    if(transform.header.frame_id=="camera_link" and transform.child_frame_id=="camera_depth_frame"):
		print_single_transform(transform)
		
