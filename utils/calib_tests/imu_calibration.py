import serial, struct, numpy as np, time, pickle
from functools import partial
from scipy.optimize import minimize

dt = 10e-3

def measure(dur: int, port: str = 'COM9', baud: int = 576000):
    global dt
    
    with serial.Serial(port, baud) as ser:
        data = np.array([0] * 9)
        
        ser.flush()
        ser.flushInput()
        for i in range(int(dur / dt)):
            raw = ser.read(4 * 9)
            if len(raw) != 4 * 9:
                print ('Error: read {} bytes'.format(len(raw)))
                continue
            ax, ay, az, mx, my, mz, gx, gy, gz = struct.unpack('f'*9, raw)
            if i < 10:
                print(f'ax: {ax: <8.3}, ay: {ay: <8.3}, az: {az: <8.3}, mx: {mx: <8.3}, my: {my: <8.3}, mz: {mz: <8.3}, gx: {gx: <8.3}, gy: {gy: <8.3}, gz: {gz: <8.3}')
            
            data = np.vstack((data, [ax, ay, az, mx, my, mz, gx, gy, gz]))
        
        return data[1:]
    return False
            

def countdown(dur: int):
    for i in range(dur, 0, -1):
        print (f'Starting in {i}...')
        time.sleep(1)
    print ('Start!')
    return True
            
            
def accel_cost(x, data):
    Ta = np.array([[1, x[2], -x[1]], [-x[2], 1, x[0]], [x[1], -x[0], 1]])
    Ka = np.array([[x[3], 0, 0], [0, x[4], 0], [0, 0, x[5]]])
    ba = np.array([x[6], x[7], x[8]])
    
    res = 0
    for row in data:
        res += (np.linalg.norm(Ta @ Ka @ (row + ba)) - 9.81)**2
        
    return res


print('Starting Accelerometer Calibration')
print('Place the IMU on a flat surface, move it and keep it still in different orientations')
print('Recording data for 20 seconds...')
countdown(5)
data = measure(20)

print('Data Collected! Calibrating...')
res = minimize(
    partial(accel_cost, data=data[:, :3]),
    (0, 0, 0, 1, 1, 1, 1e-5, 1e-5, 1e-5),
    method = 'SLSQP',
    options={
        'disp': True,
        'maxiter': 5000,
    },
    bounds = (
        (-0.1, 0.1),
        (-0.1, 0.1),
        (-0.1, 0.1),
        (0.9, 1.1),
        (0.9, 1.1),
        (0.9, 1.1),
        (-2, 2),
        (-2, 2),
        (-2, 2),            
    )
)
Ta = np.array([[1, res.x[2], -res.x[1]], [-res.x[2], 1, res.x[0]], [res.x[1], -res.x[0], 1]])
Ka = np.array([[res.x[3], 0, 0], [0, res.x[4], 0], [0, 0, res.x[5]]])
ba = np.array([res.x[6], res.x[7], res.x[8]])

with open('./utils/data/accel_calib.pkl', 'wb') as f:
    pickle.dump((Ta, Ka, ba), f)