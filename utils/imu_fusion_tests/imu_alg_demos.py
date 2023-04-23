import serial, struct, pandas as pd, numpy as np, open3d as o3d, quaternion
from matplotlib import pyplot as plt
from copy import deepcopy


def q2eul(q):
    return (
        float(np.arctan2(2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x*q.x + q.y*q.y))),
        float(-np.pi/2 + 2*np.arctan2(np.sqrt(1 + 2*(q.w*q.y - q.x*q.z)), np.sqrt(1 - 2*(q.w*q.y - q.x*q.z)))),
        float(np.arctan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z)))
    )
    
def eul2q(r, p, y):
    return np.quaternion(
        np.cos(r/2)*np.cos(p/2)*np.cos(y/2) + np.sin(r/2)*np.sin(p/2)*np.sin(y/2),
        np.sin(r/2)*np.cos(p/2)*np.cos(y/2) - np.cos(r/2)*np.sin(p/2)*np.sin(y/2),
        np.cos(r/2)*np.sin(p/2)*np.cos(y/2) + np.sin(r/2)*np.cos(p/2)*np.sin(y/2),
        np.cos(r/2)*np.cos(p/2)*np.sin(y/2) - np.sin(r/2)*np.sin(p/2)*np.cos(y/2)
    )

def q_conj(q):
    return np.quaternion(q.w, -q.x, -q.y, -q.z)


dt = 10e-3
acc_rpy = (0, 0, 0)
gyr_rpy = (0, 0, 0)
mahony_q = np.quaternion(1, 0, 0, 0)
mahony_rpy = (0, 0, 0)
madgwick_q = np.quaternion(1, 0, 0, 0)
madgwick_rpy = (0, 0, 0)

mahony_kp = 1e-9
mahony_ki = 1e-5

madgwick_k = 0.5

Ea = np.quaternion(0, 0, 0, 9.800655)
Em = np.quaternion(0, 26.6068, 2.3557, 46.3964)

print ("Creating visualizer...")
acc_vis = o3d.visualization.Visualizer()
acc_vis.create_window(width=945, height=460, left = 10, top=40, window_name='IMU Accelerometer Visualization')

gyr_vis = o3d.visualization.Visualizer()
gyr_vis.create_window(width=945, height=460, left=965, top=40, window_name='IMU Gyroscope Visualization')

mahony_vis = o3d.visualization.Visualizer()
mahony_vis.create_window(width=945, height=460, left=10, top=540, window_name='Mahony Visualization')

madgwick_vis = o3d.visualization.Visualizer()
madgwick_vis.create_window(width=945, height=460, left = 965, top=540, window_name='Madgwick Visualization')


mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
acc_mesh = deepcopy(mesh)
gyr_mesh = deepcopy(mesh)
madgwick_mesh = deepcopy(mesh)
mahony_mesh = deepcopy(mesh)

acc_vis.add_geometry(acc_mesh)
gyr_vis.add_geometry(gyr_mesh)
madgwick_vis.add_geometry(madgwick_mesh)
mahony_vis.add_geometry(mahony_mesh)

print("Openning Serial port...")
with serial.Serial('COM9', 115200, timeout=1) as ser:
    data = np.array([0] * 9)
    
    ser.flushInput()
    ser.flush()
    
    iter = 0
    while True:
        raw = ser.read(4 * 9)
        if len(raw) != 4 * 9:
            print ('Error: read {} bytes'.format(len(raw)))
            continue
        
        ax, ay, az, mx, my, mz, gx, gy, gz = struct.unpack('f'*9, raw)
        sa = np.array([ax, ay, az])
        sm = np.array([mx, my, mz])
        sw = np.array([gx, gy, -gz])
        
        # =================== Accelerometer calculations ===================
        r = np.arctan2(ay, az)
        p = np.arctan2(-ax, (ay*ay + az*az)**0.5)
        # y = np.arctan2(-my, mx)
        y = 0
        
        R = acc_mesh.get_rotation_matrix_from_xyz((r - acc_rpy[0], p - acc_rpy[1], y - acc_rpy[2]))
        acc_mesh.rotate(R, center=(0, 0, 0))
        
        acc_rpy = (r, p, y)
        # print(ax, ay, az, r, p, y)
        
        if iter == 0:
            mahony_rpy = acc_rpy
            mahoyn_q = eul2q(r, p, y)
        
        # =================== Gyro calculations ===================
        gyr_dx = gx * dt
        gyr_dy = gy * dt
        gyr_dz = gz * dt
        # gyr_rpy = (gyr_rpy[0] + gyr_dx, gyr_rpy[1] + gyr_dy, gyr_rpy[2] + gyr_dz)
        gyr_rpy = (gyr_rpy[0] + gyr_dx, gyr_rpy[1] + gyr_dy, 0)
        
        # R = gyr_mesh.get_rotation_matrix_from_xyz((gyr_dx, gyr_dy, gyr_dz))
        R = gyr_mesh.get_rotation_matrix_from_xyz((gyr_dx, gyr_dy, 0))
        gyr_mesh.rotate(R, center=(0, 0, 0))
        
        # =================== Mahony calculations ===================
        
        sa = np.quaternion(0, ax, ay, az)
        sm = np.quaternion(0, mx, my, -mz)
        sw = np.quaternion(0, gx, gy, gz)
        
        sa_ = q_conj(mahony_q) * Ea * mahony_q
        sa_ /= (sa_.x**2 + sa_.y**2 + sa_.z**2 + sa_.w**2)**0.5
        sm_ = q_conj(mahony_q) * Em * mahony_q
        sm_ /= (sm_.x**2 + sm_.y**2 + sm_.z**2 + sm_.w**2)**0.5
        
        sw_mest = np.cross(sa.imag, sa_.imag / np.linalg.norm(sa_.imag)) + np.cross(sm.imag, sm_.imag / np.linalg.norm(sm_.imag))
        sw_ = -mahony_ki * sw_mest * dt
        sw_ /= np.linalg.norm(sw_)
        sw_rqt = sw - np.quaternion(0, sw_[0], sw_[1], sw_[2]) + \
            np.quaternion(0, mahony_kp * sw_mest[0], mahony_kp * sw_mest[1], mahony_kp * sw_mest[2])
        sw_rqt /= (sw_rqt.x**2 + sw_rqt.y**2 + sw_rqt.z**2 + sw_rqt.w**2)**0.5
            
        mahony_q = mahony_q + 0.5 * mahony_q * sw_rqt * dt
        mahony_q /= (mahony_q.x**2 + mahony_q.y**2 + mahony_q.z**2 + mahony_q.w**2)**0.5
        
        r, p, y = q2eul(mahony_q)
        R = mahony_mesh.get_rotation_matrix_from_xyz((r - mahony_rpy[0], p - mahony_rpy[1], y - mahony_rpy[2]))
        mahony_mesh.rotate(R, center=(0, 0, 0))
        mahony_rpy = (r, p, y)
        
        # =================== Madgwick calculations ===================
        
        # =================== Update Visuals ===================
        acc_vis.update_geometry(acc_mesh)
        if acc_vis.poll_events():
            acc_vis.update_renderer()
        
        gyr_vis.update_geometry(gyr_mesh)
        if gyr_vis.poll_events():
            gyr_vis.update_renderer()
            
        mahony_vis.update_geometry(mahony_mesh)
        if mahony_vis.poll_events():
            mahony_vis.update_renderer()
            
        madgwick_vis.update_geometry(madgwick_mesh)
        if madgwick_vis.poll_events():
            madgwick_vis.update_renderer()
            
        iter += 1
