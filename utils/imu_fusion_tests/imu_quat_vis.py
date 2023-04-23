import serial, struct, numpy as np, open3d as o3d, quaternion
from copy import deepcopy


def q2eul(q):
    return (
        float(np.arctan2(2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x*q.x + q.y*q.y))),
        float(-np.pi/2 + 2*np.arctan2(np.sqrt(1 + 2*(q.w*q.y - q.x*q.z)), np.sqrt(1 - 2*(q.w*q.y - q.x*q.z)))),
        float(np.arctan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z)))
    )


print ("Creating visualizer...")
vis = o3d.visualization.Visualizer()
vis.create_window(width=945, height=460, left = 10, top=40, window_name='IMU Accelerometer Visualization')

mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])

vis.add_geometry(mesh)

print("Openning Serial port...")
with serial.Serial('COM9', 576000) as ser:
    data = np.array([0] * 9)
    
    ser.flushInput()
    ser.flush()
    
    rpy_old = (0, 0, 0)
    while True:
        raw = ser.read(4 * 4)
    
        # print(raw)
        if len(raw) != 4 * 4:
            print ('Error: read {} bytes'.format(len(raw)))
            continue
        
        x, y, z, w = struct.unpack('f'*4, raw)
        print(f'w: {w: <5.3f}, x: {x: <5.3f}, y: {y: <5.3f}, z: {z: <5.3f}')
        
        # =================== Accelerometer calculations ===================
        
        rpy = q2eul(np.quaternion(w, x, y, z))
        R = mesh.get_rotation_matrix_from_xyz([rpy[0] - rpy_old[0], rpy[1] - rpy_old[1], rpy[2] - rpy_old[2]])
        mesh.rotate(R, center=(0, 0, 0))
        rpy_old = rpy
        
        # =================== Update Visuals ===================
        vis.update_geometry(mesh)
        if vis.poll_events():
            vis.update_renderer()
