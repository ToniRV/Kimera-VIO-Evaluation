import rosbag
import re
print '#timestamp,x,y,z,qw,qx,qy,qz,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz'
for topic, msg, t in rosbag.Bag('tressider-2019-04-26_0.bag').read_messages():
    if re.search("tf", topic):
	for transform in msg.transforms:
	    if(transform.header.frame_id=="odom" and transform.child_frame_id=="base_link"):
		print '{}, {}, {}, {}, {}, {}, {}, {},,,,,,,,,'.format(transform.header.stamp, transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z, transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z)
