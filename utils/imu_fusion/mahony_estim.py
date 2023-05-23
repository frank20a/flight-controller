import sys, numpy as np
from utils import qinv, qmul, quat2rpy
from ahrs_ import AHRSEstimator


BETA = np.sqrt(3/4) * 0.0068
ZETA = np.sqrt(3/4) * 0.00003


def floorceil_angle(angle):
    if abs(angle) > np.pi:
        return angle - np.sign(angle) * 2 * np.pi
    return angle


def gen_F(q, d, s) -> np.ndarray:
    return np.array([
        2 * d[0] * (0.5 - q[2]**2 - q[3]**2) + 2 * d[1] * (q[0] * q[3] + q[1] * q[2]) + 2 * d[2] * (q[1] * q[3] - q[0] * q[2]) - s[0],
        2 * d[0] * (q[1] * q[2] - q[0] * q[3]) + 2 * d[1] * (0.5 - q[1]**2 - q[3]**2) + 2 * d[2] * (q[0] * q[1] + q[2] * q[3]) - s[1],
        2 * d[0] * (q[0] * q[2] + q[1] * q[3]) + 2 * d[1] * (q[2] * q[3] - q[0] * q[1]) + 2 * d[2] * (0.5 - q[1]**2 - q[2]**2) - s[2]
    ])

  
def gen_J(q, d) -> np.ndarray:
    return np.array([
        [2*d[1]*q[3] - 2*d[2]*q[2], 2*d[1]*q[2] + 2*d[2]*q[3], -4*d[0]*q[2] + 2*d[1]*q[1] - 2*d[2]*q[0], -4*d[0]*q[3] + 2*d[1]*q[0] + 2*d[2]*q[1]],
        [-2*d[0]*q[3] + 2*d[2]*q[1], 2*d[0]*q[2] - 4*d[1]*q[1] + 2*d[2]*q[0], 2*d[0]*q[1] + 2*d[2]*q[3], -2*d[0]*q[0] - 4*d[1]*q[3] + 2*d[2]*q[2]],
        [2*d[0]*q[2] - 2*d[1]*q[1], 2*d[0]*q[3] - 2*d[1]*q[0] - 4*d[2]*q[1], 2*d[0]*q[0] + 2*d[1]*q[3] - 4*d[2]*q[2], 2*d[0]*q[1] + 2*d[1]*q[2]]
    ])


class AHRSMahony(AHRSEstimator):
    def __init__(self, filename, title, kp=0.1, ki=0.02):
        super().__init__(filename, f"{title} ($K_p$: {kp:.2f}, $K_i$: {ki:.2f})")
        self.kp = kp
        self.ki = ki

    def calculate(self):
        # Calculate Accelerometer Estimation
        q = np.array([1, 0, 0, 0], dtype=np.float64)
        q_ = np.empty((self.accel.shape[0], 4))
        bias = np.array([0, 0, 0], dtype=np.float64)
        rpy = np.empty((self.accel.shape[0], 3))
        for (i, a), (_, m), (_, g) in zip(
            self.accel.iterrows(), self.mag.iterrows(), self.gyro.iterrows()
        ):
            ax, ay, az = a.iloc
            ax /= np.sqrt(ax**2 + ay**2 + az**2)
            ay /= np.sqrt(ax**2 + ay**2 + az**2)
            az /= np.sqrt(ax**2 + ay**2 + az**2)

            mx, my, mz = m.iloc
            mx /= np.sqrt(mx**2 + my**2 + mz**2)
            my /= np.sqrt(mx**2 + my**2 + mz**2)
            mz /= np.sqrt(mx**2 + my**2 + mz**2)

            gx, gy, gz = g.iloc
            
            sa = qmul(qinv(q), qmul(np.array([0, 0, 0, 1]), q))
            h = qmul(q, qmul([0, mx, my, mz], qinv(q)))
            bm = [0, 0, np.sqrt(h[1]**2 + h[2]**2), h[3]]
            sm = qmul(qinv(q), qmul(bm, q))
            
            sw_mes = np.cross([ax, ay, az], [sa[1], sa[2], sa[3]]) + np.cross([mx, my, mz], [sm[1], sm[2], sm[3]])
            sw_b_dot = -self.ki * sw_mes
            bias += sw_b_dot * self.dt
            sw_rqt = np.array([0, gx, gy, gz]) - np.array([0, bias[0], bias[1], bias[2]]) + self.kp * np.array([0, sw_mes[0], sw_mes[1], sw_mes[2]])
            
            q_dot = 0.5 * qmul(q, sw_rqt)
            q += q_dot * self.dt
            
            q_[i] = q
            rpy[i] = quat2rpy(q)

        # Set calculated values
        self.q_calc = q_
        self.rpy_calc = rpy


if __name__ == "__main__":
    try:
        kp = float(sys.argv[2])
        ki = float(sys.argv[3])
    except IndexError:
        kp = 0.1
        ki = 0.02
    
    ahrs = AHRSMahony(sys.argv[1], "Mahony Filter Estimation", kp, ki)
    ahrs.plot()
