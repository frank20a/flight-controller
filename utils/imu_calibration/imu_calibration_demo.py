import matplotlib.pyplot as plt, numpy as np, pickle


# ACCELEROMETER CALIBRATION

with open('./utils/data/calib_data_1.pkl', 'rb') as f:
    data = pickle.load(f)

with open('./utils/data/accel_calib.pkl', 'rb') as f:
    Ta, Ka, ba = pickle.load(f)

print("Accelerometer Calibration Parameters:")
print("Aa =\n")
for row in Ta @ Ka:
    print(f'{row[0]:.15f}, {row[1]:.15f}, {row[2]:.15f}')
print("\nba =\n")
print(f'{ba[0]:.15f}, {ba[1]:.15f}, {ba[2]:.15f}\n\n\n')
    

calibrated_accel = []
for row in data:
    calibrated_accel += [Ta@Ka@(row[:3] - ba)]
calibrated_accel = np.array(calibrated_accel)

fig = plt.figure(figsize=(11, 9))
fig.tight_layout()
fig.suptitle('Accelerometer Calibration')
fig.subplots_adjust(left=0.11, bottom=0.055, right=0.93, top=0.91, wspace=0.3, hspace=0.24)

ax1 = fig.add_subplot(2, 2, 1, projection='3d')
ax1.scatter(data[:, 0], data[:, 1], data[:, 2], color='red', marker='x', label='Raw Data')
ax1.scatter(calibrated_accel[:, 0], calibrated_accel[:, 1], calibrated_accel[:, 2], color='blue', marker='x', label='Calibrated Data')
ax1.set_xlabel('Accel X (m/s^2)')
ax1.set_ylabel('Accel Y (m/s^2)')
ax1.set_zlabel('Accel Z (m/s^2)')
ax1.legend()
ax1.set_title('Accelerometer 3D Scatter')


ax2 = fig.add_subplot(2, 2, 2)
ax2.plot(9.81*np.cos(np.linspace(0, 2*np.pi, 100)), 9.81*np.sin(np.linspace(0, 2*np.pi, 100)), color='grey', label='g=9.81')
ax2.scatter(data[:, 0], data[:, 1], color='red', marker='x', label='Raw Data')
ax2.scatter(calibrated_accel[:, 0], calibrated_accel[:, 1], color='blue', marker='x', label='Calibrated Data')
ax2.set_xlabel('Accel X (m/s^2)')
ax2.set_ylabel('Accel Y (m/s^2)')
ax2.legend()
ax2.set_title('Accelerometer XY Scatter')


ax3 = fig.add_subplot(2, 2, 3)
ax3.plot(9.81*np.cos(np.linspace(0, 2*np.pi, 100)), 9.81*np.sin(np.linspace(0, 2*np.pi, 100)), color='grey', label='g=9.81')
ax3.scatter(data[:, 1], data[:, 2], color='red', marker='x', label='Raw Data')
ax3.scatter(calibrated_accel[:, 1], calibrated_accel[:, 2], color='blue', marker='x', label='Calibrated Data')
ax3.set_xlabel('Accel Y (m/s^2)')
ax3.set_ylabel('Accel Z (m/s^2)')
ax3.legend()
ax3.set_title('Accelerometer YZ Scatter')


ax4 = fig.add_subplot(2, 2, 4)
ax4.plot(9.81*np.cos(np.linspace(0, 2*np.pi, 100)), 9.81*np.sin(np.linspace(0, 2*np.pi, 100)), color='grey', label='g=9.81')
ax4.scatter(data[:, 0], data[:, 2], color='red', marker='x', label='Raw Data')
ax4.scatter(calibrated_accel[:, 0], calibrated_accel[:, 2], color='blue', marker='x', label='Calibrated Data')
ax4.set_xlabel('Accel X (m/s^2)')
ax4.set_ylabel('Accel Z (m/s^2)')
ax4.legend()
ax4.set_title('Accelerometer XZ Scatter')


# MAGNETOMETER CALIBRATION

with open('./utils/data/calib_data_2.pkl', 'rb') as f:
    data = pickle.load(f)

with open('./utils/data/mag_calib.pkl', 'rb') as f:
    Am, bm = pickle.load(f)
print("Magnetometer Calibration Parameters:")
print("Am =\n")
for row in Am:
    print(f'{row[0]:.15f}, {row[1]:.15f}, {row[2]:.15f}')
print("\nbm =\n")
print(f'{bm[0]:.15f}, {bm[1]:.15f}, {bm[2]:.15f}\n\n\n')

calibrated_mag = []
for row in data:
    calibrated_mag += [Am@(row[3:6] - bm)]
calibrated_mag = np.array(calibrated_mag)

fig = plt.figure(figsize=(11, 9))
fig.tight_layout()
fig.suptitle('Magnetometer Calibration')
fig.subplots_adjust(left=0.11, bottom=0.055, right=0.93, top=0.91, wspace=0.3, hspace=0.24)

ax1 = fig.add_subplot(2, 2, 1, projection='3d')
ax1.scatter(data[:, 3], data[:, 4], data[:, 5], color='red', marker='x', label='Raw Data')
ax1.scatter(calibrated_mag[:, 0], calibrated_mag[:, 1], calibrated_mag[:, 2], color='blue', marker='x', label='Calibrated Data')
ax1.set_xlabel('Mag X (uT)')
ax1.set_ylabel('Mag Y (uT)')
ax1.set_zlabel('Mag Z (uT)')
ax1.legend()
ax1.set_title('Magnetometer 3D Scatter')


ax2 = fig.add_subplot(2, 2, 2)
ax2.plot(46.397*np.cos(np.linspace(0, 2*np.pi, 100)), 46.397*np.sin(np.linspace(0, 2*np.pi, 100)), color='grey', label='M=46.4')
ax2.scatter(data[:, 3], data[:, 4], color='red', marker='x', label='Raw Data')
ax2.scatter(calibrated_mag[:, 0], calibrated_mag[:, 1], color='blue', marker='x', label='Calibrated Data')
ax2.set_xlabel('Mag X (uT)')
ax2.set_ylabel('Mag Y (uT)')
ax2.set_xlim(-90, 105)
ax2.set_ylim(-130, 55)
ax2.legend()
ax2.set_title('Magnetometer XY Scatter')


ax3 = fig.add_subplot(2, 2, 3)
ax3.plot(46.397*np.cos(np.linspace(0, 2*np.pi, 100)), 46.397*np.sin(np.linspace(0, 2*np.pi, 100)), color='grey', label='M=46.4')
ax3.scatter(data[:, 4], data[:, 5], color='red', marker='x', label='Raw Data')
ax3.scatter(calibrated_mag[:, 1], calibrated_mag[:, 2], color='blue', marker='x', label='Calibrated Data')
ax3.set_xlabel('Mag X (uT)')
ax3.set_ylabel('Mag Y (uT)')
ax3.set_xlim(-130, 55)
ax3.set_ylim(-95, 90)
ax3.legend()
ax3.set_title('Magnetometer YZ Scatter')


ax4 = fig.add_subplot(2, 2, 4)
ax4.plot(46.397*np.cos(np.linspace(0, 2*np.pi, 100)), 46.397*np.sin(np.linspace(0, 2*np.pi, 100)), color='grey', label='M=46.4')
ax4.scatter(data[:, 3], data[:, 5], color='red', marker='x', label='Raw Data')
ax4.scatter(calibrated_mag[:, 0], calibrated_mag[:, 2], color='blue', marker='x', label='Calibrated Data')
ax4.set_xlabel('Mag X (uT)')
ax4.set_ylabel('Mag Y (uT)')
ax4.legend()
ax4.set_title('Magnetometer XZ Scatter')


# GYROSCOPE CALIBRATION

with open('./utils/data/calib_data_3.pkl', 'rb') as f:
    data = pickle.load(f)
print(data)

with open('./utils/data/gyro_calib.pkl', 'rb') as f:
    bg = pickle.load(f)
print("Gyroscope Calibration Parameters:")
print("bg =\n")
print(f'{bg[0]:.15f}, {bg[1]:.15f}, {bg[2]:.15f}\n\n\n')

calibrated_gyro = []
for row in data:
    calibrated_gyro += [row[6:9] - bg]
calibrated_gyro = np.array(calibrated_gyro)

fig = plt.figure(figsize=(11, 9))
fig.tight_layout()
fig.suptitle('Gyroscope Bias Calibration')
fig.subplots_adjust(left=0.11, bottom=0.055, right=0.93, top=0.91, wspace=0.3, hspace=0.24)
    
ax1 = fig.add_subplot(2, 1, 1)
ax1.plot(data[:, 6], color='red', label="X Axis")
ax1.plot(data[:, 7], color='green', label="Y Axis")
ax1.plot(data[:, 8], color='blue', label="Z Axis")
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Angular Rate (rad/s)')
ax1.set_xticks(np.linspace(0, data.shape[0], 7), np.arange(0, 31, 5))
ax1.legend()
ax1.set_title('Uncalibrated Gyroscope Data')


ax2 = fig.add_subplot(2, 1, 2)
ax2.plot(calibrated_gyro[:, 0], color='red', label="X Axis")
ax2.plot(calibrated_gyro[:, 1], color='green', label="Y Axis")
ax2.plot(calibrated_gyro[:, 2], color='blue', label="Z Axis")
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Angular Rate (rad/s)')
ax2.set_xticks(np.linspace(0, data.shape[0], 7), np.arange(0, 31, 5))
ax2.legend()
ax2.set_title('Calibrated Gyroscope Data')

plt.show()