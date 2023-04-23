import matplotlib.pyplot as plt, numpy as np, pickle

with open('./utils/data/accel_calib.pkl', 'rb') as f:
    Ta, Ka, ba = pickle.load(f)
    
with open('./utils/data/calib_data.pkl', 'rb') as f:
    data = pickle.load(f)
    
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
ax4.scatter(data[:, 0], data[:, 1], color='red', marker='x', label='Raw Data')
ax4.scatter(calibrated_accel[:, 0], calibrated_accel[:, 1], color='blue', marker='x', label='Calibrated Data')
ax4.set_xlabel('Accel X (m/s^2)')
ax4.set_ylabel('Accel Z (m/s^2)')
ax4.legend()
ax4.set_title('Accelerometer XZ Scatter')

plt.show()