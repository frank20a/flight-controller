import matplotlib.pyplot as plt, pandas as pd, sys, os, numpy as np


def load(filename):
    data = pd.read_csv(os.path.join(os.getcwd(), 'utils/datasets', filename), header=[0, 1], delimiter=';')

    truth = data.iloc[:, 1:5]
    accel = data.iloc[:, 5:8]
    gyro = data.iloc[:, 8:11]
    mag = data.iloc[:, 11:14]
    
    return truth, accel, gyro, mag


def plot_agm(axis, data):
    titles = ('Accelerometer', 'Gyroscope', 'Magnetometer')
    
    for ax, title, data in zip(axis, titles, data):
        ax.plot(data)
        ax.set_title(title)
        ax.grid(True)
        ax.legend(('x', 'y', 'z'))

    
def plot_ahrs_quat(axis, data, title='Estimation'):
    titles = ('Ground Truth', title)
    
    for ax, title, data in zip(axis, titles, data):
        ax.plot(data)
        ax.set_title(title)
        ax.grid(True)
        ax.legend(('$q_w$', '$q_x$', '$q_y$', '$q_z$'))
        

def plot_ahrs_rpy(axis, data, title='Estimation'):
    err = 0
    for i in range(0, len(data[0])):
        err += np.linalg.norm(data[0][i] - data[1][i])**2
    err /= len(data[0])
        
    titles = ('Ground Truth', f'{title} (MSE: {err:.2f})')
    
    for i, (ax, title, d) in enumerate(zip(axis, titles, data)):
        try:
            ax.plot(d)
            # if i == 1:
            #     ax.plot(data[0])
            ax.set_title(title)
            ax.grid(True)
            ax.legend(('$\\phi$', '$\\theta$', '$\\psi$'))
        except ValueError:
            print(title, data)
        
        
def rpy2quat(rpy):
    '''
    Convert roll, pitch, yaw angles to quaternion.
    Input: (roll, pitch, yaw) in radians
    Output: (w, x, y, z)
    '''
    roll, pitch, yaw = rpy
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return np.array([w, x, y, z])


def quat2rpy(q):
    '''
    Convert quaternion to roll, pitch, yaw angles.
    Input: (w, x, y, z)
    Output: (roll, pitch, yaw) in radians
    '''
    w, x, y, z = q
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
    pitch = 2 * np.arctan2(np.sqrt((1 + 2 * (w*y - x*z))), np.sqrt(1 - 2 * (w*y - x*z))) - np.pi/2
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    
    return np.array([roll, pitch, yaw])


def qmul(q1, q2):
    '''
    Multiply two quaternions.
    Input: q1: (w, x, y, z), q2: (w, x, y, z)
    Output: (w, x, y, z)
    '''
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    
    return np.array([w, x, y, z])


def qinv(q):
    '''
    Invert quaternion.
    Input: (w, x, y, z)
    Output: (w, x, y, z)
    '''
    w, x, y, z = q
    
    return np.array([w, -x, -y, -z])


if __name__ == '__main__':
    truth, accel, gyro, mag = load(sys.argv[1])
    
    fig = plt.figure(figsize=(12, 8))
    
    plot_agm(fig.subplots(3, 1), (accel, gyro, mag))
    fig.tight_layout()
    
    plt.show()
    