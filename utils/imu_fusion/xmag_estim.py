import sys, numpy as np
from utils import rpy2quat
from ahrs_ import AHRSEstimator

class AHRSAccel(AHRSEstimator):
    def calculate(self) -> np.ndarray:
        q = np.empty((self.accel.shape[0], 4))
        rpy = np.empty((self.accel.shape[0], 3))
        
        
        init_yaw = 0
        for (i, a), (j, m) in zip(self.accel.iterrows(), self.mag.iterrows()):
            x, y, z = a.iloc
            xmag, ymag, zmag = m.iloc
            
            roll = np.arctan2(y, z)
            pitch = np.arcsin(-x / np.sqrt(x**2 + z**2 + y**2))
            if i == 0:
                init_yaw = np.arctan2(
                    -ymag * np.cos(roll) + zmag * np.sin(roll),
                    xmag * np.cos(pitch) + ymag * np.sin(pitch) * np.sin(roll) + zmag * np.sin(pitch) * np.cos(roll)
                )
            yaw = np.arctan2(
                -ymag * np.cos(roll) + zmag * np.sin(roll),
                xmag * np.cos(pitch) + ymag * np.sin(pitch) * np.sin(roll) + zmag * np.sin(pitch) * np.cos(roll)
            ) - init_yaw
            
            q[i] = rpy2quat((roll, pitch, yaw))
            rpy[i] = (roll, pitch, yaw)
            
        self.q_calc = q
        self.rpy_calc = rpy


if __name__ == '__main__':
    ahrs = AHRSAccel(sys.argv[1], "Accelerometer/Magnetometer Estimation")
    ahrs.plot()