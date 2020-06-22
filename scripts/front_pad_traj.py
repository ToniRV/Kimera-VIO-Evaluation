import numpy as np
import pandas as pd

path_to_traj = r'../csv/traj_gt.csv'
path_to_padded_traj = r'../csv/traj_gt_padded.csv'
num_points_to_pad = 100

traj_df = pd.read_csv(path_to_traj)
num_points = len(traj_df.index)
delta_t = (traj_df.loc[num_points-1, '#timestamp'] - traj_df.loc[0, '#timestamp'])/num_points
row_template = traj_df.iloc[0].copy()
padding = []
for i in range(num_points_to_pad):
    new_timestamp = int(traj_df.loc[0, '#timestamp'] + (-num_points_to_pad + i) * delta_t)
    row_template['#timestamp'] = new_timestamp
    padding.append(row_template.copy())
padding_df = pd.DataFrame(padding, columns=traj_df.columns)
padded_traj = pd.concat([padding_df, traj_df], ignore_index=True)
padded_traj.to_csv(path_to_padded_traj)

