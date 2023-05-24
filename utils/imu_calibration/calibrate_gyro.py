import numpy as np, json, pickle
from functools import partial
from scipy.optimize import minimize


with open('./utils/data/gyro_calib_data.pkl', 'rb') as f:
    data = pickle.load(f)

print('Data Collected! Calibrating...')
with open('./utils/data/gyro_calib.json', 'w') as f:
    json.dump({
        'A': np.zeros((3, 3)).tolist(),
        'b': np.mean(data[:, 6:9], axis=0).tolist()
    }, f)
