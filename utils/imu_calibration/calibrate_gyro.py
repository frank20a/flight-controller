import numpy as np, pickle
from functools import partial
from scipy.optimize import minimize


with open('./utils/data/calib_data_3.pkl', 'rb') as f:
    data = pickle.load(f)

print('Data Collected! Calibrating...')
with open('./utils/data/gyro_calib.pkl', 'wb') as f:
    pickle.dump(np.mean(data[:, 6:9], axis=0), f)