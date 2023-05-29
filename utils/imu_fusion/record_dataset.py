import serial, struct, numpy as np, time, pandas as pd, sys

dt = 10e-3

sensor = sys.argv[1]


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


def measure(dur: int, port: str = 'COM9', baud: int = 576000):    
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
            
            data += [[ax, ay, az, gx, gy, gz, mx, my, mz]]
            
    return np.array(data)


def countdown(dur: int):
    for i in range(dur, 0, -1):
        print (f'Starting in {i}...')
        time.sleep(1)
    print ('Start!')
    return True


test('COM10')
print('Starting Measurements')
countdown(3)
data = measure(60, 'COM10')

with open(f'./utils/datasets/recording_{sys.argv[1]}.csv', 'wb') as f:
    pd.DataFrame(np.hstack((np.zeros((data.shape[0], 5)), data)), columns=['', 'W', 'X', 'Y', 'Z', 'X', 'Y', 'Z', 'X', 'Y', 'Z', 'X', 'Y', 'Z']).to_csv(f, index=False, sep=';')
