import pandas as pd, open3d as o3d, numpy as np, time
from imu_plotter import plot_imu
from copy import deepcopy

ra_filter_lens = [8, 3, 5]

data = pd.read_csv('./utils/data/imu_100hz_15s.csv')[['gx', 'gy', 'gz']]
print(data.head())

r, p, y = 0, 0, 0
dt = 10e-3

print ("Creating visualizer...")
vis = o3d.visualization.Visualizer()
vis.create_window(width=800, height=600, window_name='IMU Gyro Visualization')
mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
vis.add_geometry(mesh)
print ("Starting visualization...")

for i, row in data.iterrows():    
    # r += row.loc['gx'] * dt
    # p += row.loc['gy'] * dt
    # y += row.loc['gz'] * dt
    
    R = mesh.get_rotation_matrix_from_xyz((
        row.loc['gx'] * dt, 
        row.loc['gy'] * dt, 
        row.loc['gz'] * dt
    ))
    mesh.rotate(R, center=(0, 0, 0))

    vis.update_geometry(mesh)
    if vis.poll_events():
        vis.update_renderer()
    time.sleep(dt)
    