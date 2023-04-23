import serial, struct, numpy as np, time, pickle

dt = 10e-3


def test(port: str = 'COM9', baud: int = 576000):
    with serial.Serial(port, baud) as ser:
        while True:
            ser.flush()
            ser.flushInput()
            for j in range(5):
                raw = ser.read(4 * 9)
                if len(raw) != 4 * 9:
                    print ('Error: read {} bytes'.format(len(raw)))
                    continue
                ax, ay, az, mx, my, mz, gx, gy, gz = struct.unpack('f'*9, raw)
                
                print("Accel X: {: >6.2f}, Accel Y: {: >6.2f}, Accel Z: {: >6.2f}, Mag X: {: >6.2f}, Mag Y: {: >6.2f}, Mag Z: {: >6.2f}, Gyro X: {: >6.2f}, Gyro Y: {: >6.2f}, Gyro Z: {: >6.2f}".format(ax, ay, az, mx, my, mz, gx, gy, gz))
            
            cmd = input("Press enter to retest, q to quit...")
            if cmd == 'q':
                break

def measure_static(dur: int, port: str = 'COM9', baud: int = 576000):    
    with serial.Serial(port, baud) as ser:
        data = []
        
        while True:
            meas = []
            
            ser.flush()
            ser.flushInput()
            for j in range(dur):
                raw = ser.read(4 * 9)
                if len(raw) != 4 * 9:
                    print ('Error: read {} bytes'.format(len(raw)))
                    continue
                ax, ay, az, mx, my, mz, gx, gy, gz = struct.unpack('f'*9, raw)
                
                if abs(ax) > 80 or abs(ay) > 80 or abs(az) > 80 or abs(mx) > 200 or abs(my) > 200 or abs(mz) > 200 or abs(gx) > 20 or abs(gy) > 20 or abs(gz) > 20:
                    ser.flush()
                    ser.flushInput()
                    continue
                
                meas += [[ax, ay, az, mx, my, mz, gx, gy, gz]]
            
            entry = np.mean(meas, axis=0)
            data += [entry]
            print(f'Accel: X={entry[0]: >6.2f}, Y={entry[1]: >6.2f}, Z={entry[2]: >6.2f}, Mag: X={entry[3]: >6.2f}, Y={entry[4]: >6.2f}, Z={entry[5]: >6.2f}, Gyro: X={entry[6]: >6.2f}, Y={entry[7]: >6.2f}, Z={entry[8]: >6.2f}')
            
            cmd = input('Press enter to continue, q to quit...')
            if cmd == 'q':
                break
    
    res = np.array(data)
    with open('./utils/data/calib_data.pkl', 'wb') as f:
        pickle.dump(res, f)
    return res
             
def measure_moving(dur: int, port: str = 'COM9', baud: int = 576000):    
    with serial.Serial(port, baud) as ser:
        data = []
        
        ser.flush()
        ser.flushInput()
        for j in range(int(dur / dt)):
            raw = ser.read(4 * 9)
            if len(raw) != 4 * 9:
                print ('Error: read {} bytes'.format(len(raw)))
                continue
            ax, ay, az, mx, my, mz, gx, gy, gz = struct.unpack('f'*9, raw)
            
            if abs(ax) > 80 or abs(ay) > 80 or abs(az) > 80 or abs(mx) > 200 or abs(my) > 200 or abs(mz) > 200 or abs(gx) > 20 or abs(gy) > 20 or abs(gz) > 20:
                ser.flush()
                ser.flushInput()
                continue
            
            data += [[ax, ay, az, mx, my, mz, gx, gy, gz]]
            
    return np.array(data)
                          
def countdown(dur: int):
    for i in range(dur, 0, -1):
        print (f'Starting in {i}...')
        time.sleep(1)
    print ('Start!')
    return True


test()
print('Starting Measurements')
countdown(3)
data = measure_moving(30)

with open('./utils/data/calib_data_3.pkl', 'wb') as f:
    pickle.dump(data, f)