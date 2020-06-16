## Scripts
Here are various utility scripts for manipulating data within rosbags, especially useful for preparing Kimera datasets. Below is a list of each script and roughly what they do, then a list of the data formats each script uses. The `extract_xx` and `add_xx` scripts are the most generally useful. For JackRabbot, the rotate_images.py script is necessary to transform the vertical stereo to Kimera-compatible horizontal stereo.

`add_imu_data.py`: Copies a rosbag, inserting all of the packets from an Imu-style csv at appropriate times in the bag.

`add_static_tfs.py`: Copies a rosbag, inserting transformations from a Tf style csv at a desired frequency in the bag.

`align_time_to_headers.py`: Copies a rosbag, except messages play back at the time they were created (determined by header timestamp)

`extract_imu_data.py`: Given an IMU topic, prints the relevant IMU information in the Imu csv style. To capture in a file, use file redirection as in python extract_imu_data.py > output_imu.csv

`extract_tf.py`: Given a parent and child frame id, prints any direct transforms between them in the Tf csv style. Note that this will *not* yield indirect transforms-- if the tf tree has a->b->c, you must capture a->b and b->c, not a->c. Using rqt's tf tree visualization is useful to determine what transforms are available. To capture in a file, use file redirection as in `python extract_imu_data.py > output_imu.csv`

`find_relative_pose.py`: Given left and right camera intrinsics and a rosbag of left/right stereo pairs, use opencv and/or opengv to determine the relative pose between the cameras using ORB feature points. This did not work as well as the others and likely has subtle bugs.

`generate_imu_data.py`: Generates and visualizes an artificial IMU dataset. The dataset can be generated from a) deriving ground truth absolute pose in the Trajectory csv format or b) using linear or cubic interpolation on existing data in the Imu csv format.

`interpolate_imu.py`: Deprecated. Copies a rosbag, except with a linear interpolation between imu messages. Likely has subtle bugs. Use `extract_imu_data.py`, `generate_imu_data.py`, and `add_imu_data.py` instead.

`rotate_images.py`: Copies a rosbag, but with the addition of decompressed and rotated images.

`rotation_mat_calc.py`: Deprecated. Assorted rotation calculations and conversions for determining stereo extrinsics in the base reference frame using scikit's Rotation library. None of the rotations this version produced provided good extrinsics, so take this with a grain of salt.

## Data Formats
These scripts use comma-separated value files alongside the Pandas library to store and retrieve data. The data format can be modified by changing the order of the columns or by adding new columns without breaking backwards compatibility, but the existing column names cannot change.

#### Tf
A csv capturing transformations between reference frames with the columns #timestamp, x, y, z, qx, qy, qz, qw, frame_id, child_frame_id
`#timestamp` is the static tf timestamp, in nanoseconds
`x,y,z` are the translation from child_frame_id to frame_id in m
`qx, qy, qz, qw` are the quaternion values of the rotation from child_frame_id to frame_id

#### Trajectory
A csv capturing absolute poses, velocities, gravity reference, and acceleration following the Euroc ground truth dataset format. When inserted into a rosbag, Kimera can use this to initialize from ground truth and calculate pose error. Most of the scripts only populate the timestamp and absolute pose columns.
`#timestamp` is the static tf timestamp, in nanoseconds
`x,y,z` are the translation of the absolute pose in m
`qx, qy, qz, qw` are the quaternion values of the rotation of the absolute pose.
`vx, vy, vz` are the absolute linear velocity, usually unused.
`bgx, bgy, bgz` are the gravity direction, usually unused.
`bax, bay, baz` are the linear acceleration, usually unused. See Imu for acceleration data.

#### Imu
A csv capturing acceleration, angular velocity, and orientation estimates consistent with [ROS Imu messages](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html). 
`timestamp` is the static tf timestamp, in nanoseconds
`ax, ay, az` are the linear acceleration components in m/s2
`rvx, rvy, rvz` are the angular velocity components in rad/s
`qx, qy, qz, qw` are the quaternion values of the orientation estimate

