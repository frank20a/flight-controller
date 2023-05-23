import sys, numpy as np
from utils import rpy2quat
from ahrs_ import AHRSEstimator
from copy import deepcopy


def floorceil_angle(angle):
    if abs(angle) > np.pi:
        return angle - np.sign(angle) * 2 * np.pi
    return angle


class AHRSAccel(AHRSEstimator):
    def __init__(self, filename, title, alpha = 0.02):
        super().__init__(filename, f'{title} ($\\alpha$ = {alpha})')
        self.alpha = alpha
    
    def calculate(self) -> np.ndarray:
        # Calculate Accelerometer Estimation
        q = np.empty((self.accel.shape[0], 4))
        rpy = np.empty((self.accel.shape[0], 3))
        for (i, a), (_, m), (_, g) in zip(self.accel.iterrows(), self.mag.iterrows(), self.gyro.iterrows()):
            x, y, z = a.iloc
            xmag, ymag, zmag = m.iloc
            
            
            xmag_roll = np.arctan2(y, z)
            xmag_pitch = np.arcsin(-x / np.sqrt(x**2 + z**2 + y**2))
            if i == 0:
                init_yaw = np.arctan2(
                    -ymag * np.cos(xmag_roll) + zmag * np.sin(xmag_roll),
                    xmag * np.cos(xmag_pitch) + ymag * np.sin(xmag_pitch) * np.sin(xmag_roll) + zmag * np.sin(xmag_pitch) * np.cos(xmag_roll)
                )
            xmag_yaw = np.arctan2(
                -ymag * np.cos(xmag_roll) + zmag * np.sin(xmag_roll),
                xmag * np.cos(xmag_pitch) + ymag * np.sin(xmag_pitch) * np.sin(xmag_roll) + zmag * np.sin(xmag_pitch) * np.cos(xmag_roll)
            ) - init_yaw
        
            x, y, z = g.iloc
            
            if i == 0:
                prev = (xmag_roll, xmag_pitch, xmag_yaw)
            else:
                prev = rpy[i - 1]
                
            gyro_roll = floorceil_angle(prev[0] + x * 1e-2)
            gyro_pitch = floorceil_angle(prev[1] + y * 1e-2)
            gyro_yaw = floorceil_angle(prev[2] + z * 1e-2)
            
            rpy[i] = [
                self.alpha * xmag_roll + (1 - self.alpha) * gyro_roll,
                self.alpha * xmag_pitch + (1 - self.alpha) * gyro_pitch,
                self.alpha * xmag_yaw + (1 - self.alpha) * gyro_yaw
            ]
            q[i] = rpy2quat(rpy[i])
            
        # Set calculated values
        self.q_calc = q
        self.rpy_calc = rpy
 

if __name__ == '__main__':
    ahrs = AHRSAccel(sys.argv[1], "Complementary Filter Estimation", alpha = float(sys.argv[2]))
    ahrs.plot()