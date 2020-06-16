import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import scipy
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation, Slerp

print scipy.__version__

def main():
    imu_old_df = pd.read_csv(r'../csv/tressidor_imu.csv')
    new_frequency = 200e-9 # nHz, originally 20e-9
    ts_new = generate_timestamps(imu_old_df['timestamp'], new_frequency)
    imu_new_df = interpolate_df_cubic(imu_old_df, ts_new)
    plot_accelerations(imu_old_df, imu_new_df)
    # plot_angular_velocities(imu_old_df, imu_new_df)
    # plot_orientations(imu_old_df, imu_new_df)
    # imu_new_df.to_csv(r'../csv/tressidor_imu_interpolated.csv', index=False)

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
    plt.plot(imu_old_df['timestamp'], imu_old_df['qx'])
    plt.plot(imu_new_df['timestamp'], imu_new_df['qx'])
    plt.plot(imu_old_df['timestamp'], imu_old_df['qy'])
    plt.plot(imu_new_df['timestamp'], imu_new_df['qy'])
    plt.plot(imu_old_df['timestamp'], imu_old_df['qz'])
    plt.plot(imu_new_df['timestamp'], imu_new_df['qz'])
    plt.plot(imu_old_df['timestamp'], imu_old_df['qw'])
    plt.plot(imu_new_df['timestamp'], imu_new_df['qw'])
    plt.legend(['qx_old','qx_new','qy_old','qy_new','qz_old','qz_new','qw_old','qw_new'])
    plt.show()

main()
