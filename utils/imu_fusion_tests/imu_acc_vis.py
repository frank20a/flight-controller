import pandas as pd, open3d as o3d, numpy as np
from imu_plotter import plot_imu

ra_filter_lens = [8, 3, 5]

data = pd.read_csv('./utils/data/imu_100hz_15s.csv')

