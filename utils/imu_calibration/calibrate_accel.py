import numpy as np, pickle
from functools import partial
from scipy.optimize import minimize


def accel_cost(x, data):
    Ta = np.array([[1, x[2], -x[1]], [-x[2], 1, x[0]], [x[1], -x[0], 1]])
    Ka = np.array([[x[3], 0, 0], [0, x[4], 0], [0, 0, x[5]]])
    ba = np.array([x[6], x[7], x[8]])
    
    res = 0
    for row in data:
        tmp = Ta @ Ka @ (row - ba)
        res += (np.linalg.norm(tmp) - 9.81)**2
        
    return res


with open('./utils/data/calib_data_1.pkl', 'rb') as f:
    data = pickle.load(f)

print('Data Collected! Calibrating...')
res = minimize(
    partial(accel_cost, data=data[:, :3]),
    (0, 0, 0, 1, 1, 1, 1e-5, 1e-5, 1e-5),
    method = 'SLSQP',
    options={
        'disp': True,
    },
)
Ta = np.array([[1, res.x[2], -res.x[1]], [-res.x[2], 1, res.x[0]], [res.x[1], -res.x[0], 1]])
Ka = np.array([[res.x[3], 0, 0], [0, res.x[4], 0], [0, 0, res.x[5]]])
ba = np.array([res.x[6], res.x[7], res.x[8]])

with open('./utils/data/accel_calib.pkl', 'wb') as f:
    pickle.dump((Ta, Ka, ba), f)