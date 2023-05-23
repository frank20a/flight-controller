import sys, numpy as np
from utils import rpy2quat
from ahrs_ import AHRSEstimator


def floorceil_angle(angle):
    if abs(angle) > np.pi:
        return angle - np.sign(angle) * 2 * np.pi
    return angle


class AHRSAccel(AHRSEstimator):
    def calculate(self) -> np.ndarray:
        q = np.empty((self.accel.shape[0], 4))
        rpy = np.empty((self.accel.shape[0], 3))
        prev = None
        
        for i, g in self.gyro.iterrows():
            x, y, z = g.iloc
            
            if i > 0:
                roll = floorceil_angle(prev[0] + x * 1e-2)
                pitch = floorceil_angle(prev[1] + y * 1e-2)
                yaw = floorceil_angle(prev[2] + z * 1e-2)
            else:
                roll = 0
                pitch = 0
                yaw = 0
            
            prev = (roll, pitch, yaw)
            
            q[i] = rpy2quat((roll, pitch, yaw))
            rpy[i] = (roll, pitch, yaw)
            
        self.q_calc = q
        self.rpy_calc = rpy
 

if __name__ == '__main__':
    ahrs = AHRSAccel(sys.argv[1], "Gyro Estimation")
    ahrs.plot()