import rosbag
import re
from scipy.spatial.transform import Rotation

rosbag_path = r'/home/andrew/Documents/SPARK/JackRabbot/rosbags/tressider-2019-04-26_0.bag'
print 'timestamp,ax,ay,az,rvx,rvy,rvz,qw,qx,qy,qz'
for topic, msg, t in rosbag.Bag(rosbag_path).read_messages():
    if re.search("dsp3000", topic):
	orientation = Rotation.from_euler('XYZ', [0,0,msg.yaw_angle_rad]).as_quat()
	print '{},{},{},{},{},{},{},{},{},{},{}'.format(msg.header.stamp, 0, 0, 0, 0, 0, 0, msg.yaw_rate_rps, orientation[3], orientation[0], orientation[1], orientation[0])
