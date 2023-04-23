import matplotlib.pyplot as plt, numpy as np, pickle, pandas as pd
from imu_plotter import plot_imu


def euler_to_quaternion(r, p, y):
    cy = np.cos(y * 0.5)
    sy = np.sin(y * 0.5)
    cp = np.cos(p * 0.5)
    sp = np.sin(p * 0.5)
    cr = np.cos(r * 0.5)
    sr = np.sin(r * 0.5)

    q = np.array([
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        sy * cp * sr + cy * sp * cr,
        sy * cp * cr - cy * sp * sr
    ])
    return q


dt = 10e-3
raw_rpy = (0, 0, 0)
calib_rpy = (0, 0, 0)

with open('./utils/data/accel_calib.pkl', 'rb') as f:
    Ta, Ka, ba = pickle.load(f)
print(Ta, Ka, ba)

data = pd.read_csv('./utils/data/imu_100hz_15s.csv')

fig, ((ax1, ax4), (ax2, ax5), (ax3, ax6)) = plt.subplots(3, 2)
fig.tight_layout()

plot_imu(data, (ax1, ax2, ax3), lims=((-15, 15), None, (-5, 7)))

# Rolling average filter
for i, row in data.iterrows():
    tmp = np.array([row['ax'], row['ay'], row['az']])
    tmp = Ta @ Ka @ (tmp + ba)
    data.iloc[i, 0:3] = tmp

plot_imu(data, (ax4, ax5, ax6), title_apnds=[' - Calibrated', ]*3, lims=((-15, 15), None, (-5, 7)))

plt.show()