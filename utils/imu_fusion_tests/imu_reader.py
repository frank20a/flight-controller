import serial, struct, pandas as pd, numpy as np
from matplotlib import pyplot as plt

with serial.Serial('/dev/ttyUSB0', 576000, timeout=1) as ser:
    data = np.array([0] * 9)
    
    ser.flushInput()
    ser.flush()
    for i in range(1500):
        raw = ser.read(4 * 9)
        if len(raw) != 4 * 9:
            print ('Error: read {} bytes'.format(len(raw)))
            continue
        ax, ay, az, mx, my, mz, gx, gy, gz = struct.unpack('f'*9, raw)
        print(f'Acceleration (m/s^2): ({ax: <8.3f}, {ay: <8.3f}, {az: <8.3f})'.rjust(65))
        print(f'Magnetic field (uT): ({mx: <8.3f}, {my: <8.3f}, {mz: <8.3f})'.rjust(65))
        print(f'Angular velocity (rad/s): ({gx: <8.3f}, {gy: <8.3f}, {gz: <8.3f})'.rjust(65))
        
        data = np.vstack((data, [ax, ay, az, mx, my, mz, gx, gy, gz]))

df = pd.DataFrame(data[1:], columns=['ax', 'ay', 'az', 'mx', 'my', 'mz', 'gx', 'gy', 'gz'])
df.to_csv('./utils/data/imu_100hz_15s.csv', index=False)
        