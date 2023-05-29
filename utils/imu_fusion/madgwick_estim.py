import sys, numpy as np
from utils import qinv, qmul, quat2rpy
from ahrs_ import AHRSEstimator


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
        [2*d[1]*q[3] - 2*d[2]*q[2], 2*d[1]*q[2] + 2*d[2]*q[3], -4*d[0]*q[2] + 2*d[1]*q[1] - 2*d[2]*q[0], -4*d[0]*q[3] + 2*d[1]*q[0] +2*d[2]*q[1]],
        [-2*d[0]*q[3] + 2*d[2]*q[1], 2*d[0]*q[2] - 4*d[1]*q[1] + 2*d[2]*q[0], 2*d[0]*q[1] + 2*d[2]*q[3], -2*d[0]*q[0] - 4*d[1]*q[3] + 2*d[2]*q[2]],
        [2*d[0]*q[2] - 2*d[1]*q[1], 2*d[0]*q[3] - 2*d[1]*q[0] - 4*d[2]*q[1], 2*d[0]*q[0] + 2*d[1]*q[3] - 4*d[2]*q[2], 2*d[0]*q[1] + 2*d[1]*q[2]]
    ])


class AHRSMadgwick(AHRSEstimator):
    def __init__(self, filename, title, beta=0.0068, zeta=0.00003):
        
        self.BETA = np.sqrt(3/4) * beta
        self.ZETA = np.sqrt(3/4) * zeta
        super().__init__(filename, f"{title} ($\\beta$= {self.BETA:.2e}, $\\zeta$= {self.ZETA:.2e})")

    def calculate(self):
        # Calculate Accelerometer Estimation
        q = np.array([1, 0, 0, 0], dtype=np.float64)
        q_ = np.empty((self.accel.shape[0], 4))
        w_b = np.array([0, 0, 0, 0], dtype=np.float64)
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

            F_g = np.array([
                2 * (q[1] * q[3] - q[0] * q[2]) - ax,
                2 * (q[0] * q[1] + q[2] * q[3]) - ay,
                2 * (0.5 - q[1]**2 - q[2]**2) - az
            ])
            J_g = np.array([
                [-2 * q[2], 2 * q[3], -2 * q[0], 2 * q[1]],
                [2 * q[1], 2 * q[0], 2 * q[3], 2 * q[2]],
                [0, -4 * q[1], -4 * q[2], 0]
            ])  # type: ignore

            h = qmul(q, qmul([0, mx, my, mz], qinv(q)))
            bm = [np.sqrt(h[1]**2 + h[2]**2), 0, h[3]]
            # F_m = gen_F(q, bm, [mx, my, mz])
            # J_m = gen_J(q, bm)
            F_m = np.array([
                2*bm[0]*(0.5 - q[2]**2 - q[3]**2) + 2*bm[2]*(q[1]*q[3] - q[0]*q[2]) - mx,
                2*bm[0]*(q[1]*q[2] - q[0]*q[3]) + 2*bm[2]*(q[0]*q[1] + q[2]*q[3]) - my,
                2*bm[0]*(q[0]*q[2] + q[1]*q[3]) + 2*bm[2]*(0.5 - q[1]**2 - q[2]**2) - mz
            ])
            J_m = gen_J(q, bm)

            F = np.concatenate((F_g, F_m))
            J = np.vstack((J_g, J_m))

            grad = J.T @ F
            grad /= np.linalg.norm(grad)

            w_e = 2 * qmul(qinv(q), grad)
            w_b += w_e * self.dt

            q_w = np.array([0, gx, gy, gz], dtype=np.float64) - self.ZETA * w_b
            q_w_dot = 0.5 * qmul(q, q_w)

            q_dot = q_w_dot - self.BETA * grad.T
            q += q_dot * self.dt
            q /= np.linalg.norm(q)

            q_[i] = q
            rpy[i] = quat2rpy(q)

        # Set calculated values
        self.q_calc = q_
        self.rpy_calc = rpy


if __name__ == "__main__":
    if len(sys.argv) > 2:
        beta = float(sys.argv[2])
        zeta = float(sys.argv[3])
    else:
        beta = 0.0068
        zeta = 0.00003
        
    ahrs = AHRSMadgwick(sys.argv[1], "Madgwick Filter Estimation", beta, zeta)
    ahrs.plot()
