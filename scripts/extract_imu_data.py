import rosbag
import re

rosbag_path = r'/home/andrew/Documents/SPARK/Experimentation/V1_01_easy.bag'
print 'timestamp,ax,ay,az,rvx,rvy,rvz,qw,qx,qy,qz'
for topic, msg, t in rosbag.Bag(rosbag_path).read_messages():
    if re.search("imu0", topic):
	print '{},{},{},{},{},{},{},{},{},{},{}'.format(msg.header.stamp, msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z, msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z, msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
