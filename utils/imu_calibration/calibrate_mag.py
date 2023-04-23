import numpy as np, pickle
from functools import partial
from scipy.optimize import minimize


def mag_cost(x, data):
    A = np.array([[x[0], x[1], x[2]], [x[3], x[4], x[5]], [x[6], x[7], x[8]]])
    b = np.array([x[9], x[10], x[11]])
    
    res = 0
    for row in data:
        tmp = A @ (row - b)
        res += (np.linalg.norm(tmp) - 46.4)**2
        
    return res


with open('./utils/data/calib_data_2.pkl', 'rb') as f:
    data = pickle.load(f)

print('Data Collected! Calibrating...')
res = minimize(
    partial(mag_cost, data=data[:, 3:6]),
    (1, 0, 0, 0, 1, 0, 0, 0, 1, 15, -70, -5),
    method = 'SLSQP',
    options={
        'disp': True,
        'maxiter': 5000,
    },
    bounds = [
        (0.7, 1.3), (-0.15, 0.15), (-0.15, 0.15),
        (-0.15, 0.15), (0.7, 1.3), (-0.15, 0.15),
        (-0.15, 0.15), (-0.15, 0.15), (0.7, 1.3),
        
        (-100, 100), (-100, 100), (-100, 100),
    ]
)
A = np.array([[res.x[0], res.x[1], res.x[2]], [res.x[3], res.x[4], res.x[5]], [res.x[6], res.x[7], res.x[8]]])
b = np.array([res.x[9], res.x[10], res.x[11]])


A = np.array([
    [ 1.030310, -0.062524,  0.016296],
    [-0.062524,  1.008574, -0.066112],
    [ 0.016296, -0.066112,  1.167872]
])
b = np.array([16.565142, -74.437627, -3.953151])
with open('./utils/data/mag_calib.pkl', 'wb') as f:
    pickle.dump((A, b), f)