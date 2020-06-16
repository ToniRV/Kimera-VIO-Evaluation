import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import scipy
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation
from scipy.signal import savgol_filter

def main():
    imu_old_df = pd.read_csv(r'../csv/tressidor_imu.csv')
    #imu_new_df = compute_new_imu_df()
    imu_new_df = pd.read_csv(r'../csv/tressidor_imu_from_gt.csv')
    plot_accelerations(imu_old_df, imu_new_df)
    plot_angular_velocities(imu_old_df, imu_new_df)
    plot_orientations(imu_old_df, imu_new_df)    

def compute_new_imu_df():
    imu_old_df = pd.read_csv(r'../csv/tressidor_imu.csv')
    trajectory = pd.read_csv(r'../csv/traj_gt.csv')
    smoothed_trajectory = smooth(trajectory)
    imu_new_df = imu_from_traj_finite_differences(smoothed_trajectory)
    imu_new_df.to_csv(r'../csv/tressidor_imu_from_gt_smoothed.csv', index=False)
    return imu_new_df

def smooth(trajectory):
    smoothed_trajectory = trajectory.copy()
    smoothed_trajectory.loc[:, 'x'] = savgol_filter(trajectory['x'], 51, 3)
    smoothed_trajectory.loc[:, 'y'] = savgol_filter(trajectory['y'], 51, 3)
    smoothed_trajectory.loc[:, 'z'] = savgol_filter(trajectory['z'], 51, 3)
    # smooth quaternion using FGA
    quat_columns = ['qx','qy','qz','qw']
    unsmoothed_rot = Rotation.from_quat(trajectory[quat_columns])
    unsmoothed_euler = unsmoothed_rot.as_euler('XYZ')
    smoothed_euler = unsmoothed_euler.copy()
    smoothed_euler[:,0] = savgol_filter(unsmoothed_euler[:,0], 51, 3)
    smoothed_euler[:,1] = savgol_filter(unsmoothed_euler[:,1], 51, 3)
    smoothed_euler[:,2] = savgol_filter(unsmoothed_euler[:,2], 51, 3)
    smoothed_rot = Rotation.from_euler(trajectory[quat_columns])
    smoothed_trajectory.loc[:, quat_columns] = smoothed_rot.as_quat()
    return smoothed_trajectory

def imu_from_traj_finite_differences(trajectory_df):
    imu_columns = ['timestamp','ax','ay','az','rvx','rvy','rvz','qw','qx','qy','qz']
    imu_df = pd.DataFrame(columns=imu_columns,index=range(len(trajectory_df.index)-2))
    relevant_indices = range(1,len(trajectory_df.index)-1)
    quat_columns = ['qx','qy','qz','qw']
    imu_df[quat_columns] = trajectory_df[quat_columns].iloc[relevant_indices]
    imu_df['timestamp'] = trajectory_df['#timestamp'].iloc[relevant_indices]
    for i in (relevant_indices):
	h = (trajectory_df.loc[i+1,'#timestamp'] - trajectory_df.loc[i-1,'#timestamp'])/2 * 1e-9 # convert to secs
	# compute acceleration: (f(x-1) - 2f(x) + f(x+1))/h^2
	imu_df.loc[i-1,'ax'] = (trajectory_df.loc[i-1,'x'] - 2*trajectory_df.loc[i,'x'] + trajectory_df['x'].loc[i+1])/h**2
	imu_df.loc[i-1,'ay'] = (trajectory_df.loc[i-1,'y'] - 2*trajectory_df.loc[i,'y'] + trajectory_df.loc[i+1,'y'])/h**2
	# add gravity
	azi = (trajectory_df.loc[i-1,'z'] - 2*trajectory_df.loc[i,'z'] + trajectory_df.loc[i+1,'z'])/h**2
	imu_df.loc[i-1,'az'] = azi + 9.81
	# compute angular velocity: (f(x+1) - f(x-1))/2h
	rot_prev = Rotation.from_quat(trajectory_df[quat_columns].loc[i-1])
	rot_next = Rotation.from_quat(trajectory_df[quat_columns].loc[i+1])
	rot_delta = rot_next*rot_prev.inv()
	angular_vel = rot_delta.as_euler('XYZ')/2/h
	# fill df
	imu_df.loc[i-1,'rvx'] = angular_vel[0]
	imu_df.loc[i-1,'rvy'] = angular_vel[1]
	imu_df.loc[i-1,'rvz'] = angular_vel[2]
    return imu_df

def generate_timestamps(ts_old, new_frequency):
    t_start = min(ts_old)
    t_end = max(ts_old)
    num_points = (t_end - t_start)*new_frequency + 1
    ts_new = np.linspace(t_start, t_end, num=num_points, endpoint=True)
    return ts_new

def interpolate_df_cubic(imu_old_df, new_timestamps):
    imu_new_df = pd.DataFrame(columns=imu_old_df.columns)
    imu_new_df['timestamp'] = new_timestamps
    # linear acceleration
    rvx_cubic_interp = interp1d(imu_old_df['timestamp'], imu_old_df['ax'], kind='cubic')
    imu_new_df['ax'] = rvx_cubic_interp(new_timestamps)
    rvx_cubic_interp = interp1d(imu_old_df['timestamp'], imu_old_df['ay'], kind='cubic')
    imu_new_df['ay'] = rvx_cubic_interp(new_timestamps)
    rvx_cubic_interp = interp1d(imu_old_df['timestamp'], imu_old_df['az'], kind='cubic')
    imu_new_df['az'] = rvx_cubic_interp(new_timestamps)
    # angular velocity
    rvx_cubic_interp = interp1d(imu_old_df['timestamp'], imu_old_df['rvx'], kind='cubic')
    imu_new_df['rvx'] = rvx_cubic_interp(new_timestamps)
    rvx_cubic_interp = interp1d(imu_old_df['timestamp'], imu_old_df['rvy'], kind='cubic')
    imu_new_df['rvy'] = rvx_cubic_interp(new_timestamps)
    rvx_cubic_interp = interp1d(imu_old_df['timestamp'], imu_old_df['rvz'], kind='cubic')
    imu_new_df['rvz'] = rvx_cubic_interp(new_timestamps)
    # quaternion orientation-- note, RotationSpline is unavailable in this version of scikit
    # using Slerp instead
    old_rotations = Rotation.from_quat(imu_old_df[['qx','qy','qz','qw']])
    orientation_cubic_interp = Slerp(imu_old_df['timestamp'], old_rotations)
    new_rotations = orientation_cubic_interp(new_timestamps)
    new_rotation_quats = new_rotations.as_quat()
    imu_new_df['qx'] = new_rotation_quats[:,0]
    imu_new_df['qy'] = new_rotation_quats[:,1]
    imu_new_df['qz'] = new_rotation_quats[:,2]
    imu_new_df['qw'] = new_rotation_quats[:,3]
    return imu_new_df
    
def plot_accelerations(imu_old_df, imu_new_df):
    plt.plot(imu_old_df['timestamp'], imu_old_df['ax'])
    plt.plot(imu_new_df['timestamp'], imu_new_df['ax'])
    plt.plot(imu_old_df['timestamp'], imu_old_df['ay'])
    plt.plot(imu_new_df['timestamp'], imu_new_df['ay'])
    plt.plot(imu_old_df['timestamp'], imu_old_df['az'])
    plt.plot(imu_new_df['timestamp'], imu_new_df['az'])
    plt.legend(['ax_old','ax_new','ay_old','ay_new','az_old','az_new'])
    plt.show()

def plot_angular_velocities(imu_old_df, imu_new_df):
    plt.plot(imu_old_df['timestamp'], imu_old_df['rvx'])
    plt.plot(imu_new_df['timestamp'], imu_new_df['rvx'])
    plt.plot(imu_old_df['timestamp'], imu_old_df['rvy'])
    plt.plot(imu_new_df['timestamp'], imu_new_df['rvy'])
    plt.plot(imu_old_df['timestamp'], imu_old_df['rvz'])
    plt.plot(imu_new_df['timestamp'], imu_new_df['rvz'])
    plt.legend(['rvx_old','rvx_new','rvy_old','rvy_new','rvz_old','rvz_new'])
    plt.show()

def plot_orientations(imu_old_df, imu_new_df):
    #plt.plot(imu_old_df['timestamp'], imu_old_df['qx'])
    #plt.plot(imu_new_df['timestamp'], imu_new_df['qx'])
    #plt.plot(imu_old_df['timestamp'], imu_old_df['qy'])
    #plt.plot(imu_new_df['timestamp'], imu_new_df['qy'])
    plt.plot(imu_old_df['timestamp'], imu_old_df['qz'])
    plt.plot(imu_new_df['timestamp'], imu_new_df['qz'])
    plt.plot(imu_old_df['timestamp'], imu_old_df['qw'])
    plt.plot(imu_new_df['timestamp'], imu_new_df['qw'])
    #plt.legend(['qx_old','qx_new','qy_old','qy_new','qz_old','qz_new','qw_old','qw_new'])
    plt.legend(['qz_old','qz_new','qw_old','qw_new'])
    plt.show()

main()