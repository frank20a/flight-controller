import matplotlib.pyplot as plt
import numpy as np
from utils import load, plot_agm, plot_ahrs_quat, plot_ahrs_rpy, quat2rpy


class AHRSEstimator:
    def __init__(self, filename, title):
        # Load data
        self.title = f'{title} ({filename.split(".")[0]})'
        self.truth, self.accel, self.gyro, self.mag = load(filename)

        fS = 100  # Sampling rate.
        fL = 2  # Cutoff frequency.
        N = 50
        h = np.sinc(2 * fL / fS * (np.arange(N) - (N - 1) / 2))
        h *= np.blackman(N)
        h /= np.sum(h)
        
        self.dt = 1 / fS

        self.accel.iloc[:, 0] = np.convolve(self.accel.iloc[:, 0], h, mode="same")
        self.accel.iloc[:, 1] = np.convolve(self.accel.iloc[:, 1], h, mode="same")
        self.accel.iloc[:, 2] = np.convolve(self.accel.iloc[:, 2], h, mode="same")

        self.mag.iloc[:, 0] = np.convolve(self.mag.iloc[:, 0], h, mode="same")
        self.mag.iloc[:, 1] = np.convolve(self.mag.iloc[:, 1], h, mode="same")
        self.mag.iloc[:, 2] = np.convolve(self.mag.iloc[:, 2], h, mode="same")

        self.gyro.iloc[:, 0] = np.convolve(self.gyro.iloc[:, 0], h, mode="same")
        self.gyro.iloc[:, 1] = np.convolve(self.gyro.iloc[:, 1], h, mode="same")
        self.gyro.iloc[:, 2] = np.convolve(self.gyro.iloc[:, 2], h, mode="same")

        self.rpy_calc = self.q_calc = None

    def calculate(self) -> np.ndarray:
        raise NotImplementedError()

    def plot(self):
        if self.q_calc is None or self.rpy_calc is None:
            self.calculate()

        fig = plt.figure(figsize=(12, 8))
        fig.suptitle(self.title)
        gs = fig.add_gridspec(6, 3)

        truth_rpy = [quat2rpy(r.iloc) for i, r in self.truth.iterrows()]

        plot_agm(
            (
                fig.add_subplot(gs[0:2, 0]),
                fig.add_subplot(gs[2:4, 0]),
                fig.add_subplot(gs[4:6, 0]),
            ),
            (self.accel, self.gyro, self.mag),
        )
        plot_ahrs_quat(
            (
                fig.add_subplot(gs[0:3, 1]),
                fig.add_subplot(gs[3:6, 1]),
            ),
            (self.truth, self.q_calc),
        )
        plot_ahrs_rpy(
            (
                fig.add_subplot(gs[0:3, 2]),
                fig.add_subplot(gs[3:6, 2]),
            ),
            (np.array(truth_rpy), self.rpy_calc),
        )

        fig.tight_layout()
        plt.show()
