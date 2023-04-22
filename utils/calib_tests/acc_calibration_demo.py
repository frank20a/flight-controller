import serial, struct, numpy as np, open3d as o3d, pickle
from copy import deepcopy


dt = 10e-3
raw_rpy = (0, 0, 0)
calib_rpy = (0, 0, 0)


with open('./utils/data/accel_calib.pkl', 'rb') as f:
    Ka, Ta, ba = pickle.load(f)
print(Ka, Ta, ba)


print ("Creating visualizer...")
raw_vis = o3d.visualization.Visualizer()
raw_vis.create_window(width=945, height=460, left = 10, top=40, window_name='RAW Accelerometer Visualization')

calib_vis = o3d.visualization.Visualizer()
calib_vis.create_window(width=945, height=460, left=965, top=40, window_name='CALIBRATED Gyroscope Visualization')


mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
raw_mesh = deepcopy(mesh)
calib_mesh = deepcopy(mesh)

raw_vis.add_geometry(raw_mesh)
calib_vis.add_geometry(calib_mesh)

print("Openning Serial port...")
with serial.Serial('COM9', 576000, timeout=1) as ser:
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
        mz *= -1
        
        # print('Iter: {}, ax: {}, ay: {}, az: {}'.format(iter, ax, ay, az))
        
        # =================== Accelerometer calculations ===================
        r = np.arctan2(ay, az)
        p = np.arctan2(-ax, (ay*ay + az*az)**0.5)
        # y = np.arctan2(-my, mx)
        y = 0
        
        R = raw_mesh.get_rotation_matrix_from_xyz((r - raw_rpy[0], p - raw_rpy[1], y - raw_rpy[2]))
        raw_mesh.rotate(R, center=(0, 0, 0))
        
        raw_rpy = (r, p, y)
        
        # =================== Gyro calculations ===================
        cax, cay, caz = Ta @ Ka @ (np.array([deepcopy(ax), deepcopy(ay), deepcopy(az)]) - ba)
        
        r = np.arctan2(cay, caz)
        p = np.arctan2(-cax, (cay*cay + caz*caz)**0.5)
        # y = np.arctan2(-my, mx)
        y = 0
        
        R = calib_mesh.get_rotation_matrix_from_xyz((r - calib_rpy[0], p - calib_rpy[1], y - calib_rpy[2]))
        calib_mesh.rotate(R, center=(0, 0, 0))
        
        calib_rpy = (r, p, y)
        
        # =================== Update Visuals ===================
        raw_vis.update_geometry(raw_mesh)
        if raw_vis.poll_events():
            raw_vis.update_renderer()
        
        calib_vis.update_geometry(calib_mesh)
        if calib_vis.poll_events():
            calib_vis.update_renderer()
        
        iter += 1
