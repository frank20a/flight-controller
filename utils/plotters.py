import serial, struct
import matplotlib.pyplot as plt
import matplotlib.style as style
import matplotlib.animation as animation
import numpy as np


colors = (
    'red',
    'green',
    'blue',
    'gray',
    'yellow',
    'cyan',
    'magenta',
    'black'
)


def quat2eul(q: list) -> list:
    x, y, z, w = q
    phi = np.arctan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2))
    theta = np.arcsin(2*(w*y - z*x))
    psi = np.arctan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
    
    return [phi, theta, psi]


class Plotter:
    def __init__(self, dt, port="COM10", baud=576000, subplts=[1, 1], lins=[1], readvals=None, lbls=['X'], xlabel='Time (s)', ylabel=['Value'], datalim=100):
        self.ser = serial.Serial(port, baud)
        self.dt = dt
        self.data = np.array([])
        
        self.subplts = subplts
        self.lins = lins
        self.datalim = datalim
        self.readvals = sum(lins) if readvals is None else readvals
        
        self.lbls = lbls
        self.xlabel = xlabel
        self.ylabel = ylabel
        
        self.fig = plt.figure()
        self.axs = self.fig.subplots(*self.subplts)
        self.fig.suptitle('Serial Stream')
        self.fig.tight_layout()
        
        self.anim = animation.FuncAnimation(self.fig, self.animate, interval=self.dt, save_count=0)
        
    def read(self) -> list:
        raw = self.ser.read(4 * self.readvals)
        if len(raw) != 4 * self.readvals:
            print('Error: read {} bytes'.format(len(raw)))
            return
        
        return struct.unpack('f' * self.readvals, raw)
    
    def animate(self, *args):
        vals = self.read()
        
        self.data = np.vstack((self.data, vals)) if self.data.size else np.array([vals])
        self.data = self.data[-self.datalim:]
        
        v = 0
        for i in range(self.subplts[0] * self.subplts[1]):
            self.axs[i].cla()
            self.axs[i].set_xlabel(self.xlabel)
            self.axs[i].set_ylabel(self.ylabel[i])
            self.axs[i].set_xlim(0, self.datalim)
            if i == 0:
                self.axs[i].set_ylim(-1, 1)
            elif i == 1:
                self.axs[i].set_ylim(-np.pi, np.pi)
            
            for j in range(self.lins[i]):
                self.axs[i].plot(self.data[:, v], label=self.lbls[v], color=colors[j])
                v += 1
            
            self.axs[i].legend()

    def mainloop(self):
        self.ser.flush()
        self.ser.flushInput()
        plt.show()
        self.ser.close()


class PrePlotter(Plotter):
    def __init__(self, samples, dt, port="COM10", baud=576000, subplts=[1, 1], lins=[1], readvals=None, lbls=['X'], xlabel='Time (s)', ylabel=['Value'], datalim=100):
        super().__init__(dt, port, baud, subplts, lins, readvals, lbls, xlabel, ylabel, datalim)
        self.samples = samples
        
        self.i = 0
        self.data_ = []
        for _ in range(self.samples):
            raw = self.ser.read(4 * self.readvals)
            if len(raw) != 4 * self.readvals:
                print('Error: read {} bytes'.format(len(raw)))
                return

            vals = struct.unpack('f' * self.readvals, raw)
            print(vals)
            self.data_.append(list(vals))
            
                    
    def read(self) -> list:
        self.i += 1
        if self.i % self.samples == 0:
            print("LOOP")
        return self.data_[(self.i - 1) % self.samples]


class AnglePlotter(Plotter):
    def __init__(self, dt, port="COM10", baud=576000, datalim=100):
        super().__init__(dt, port, baud, subplts=[2, 1], lins=[4, 3], readvals=4, lbls=['Qx', 'Qy', 'Qz', 'Qw', '$\\phi$', '$\\theta$', '$\\psi$'], xlabel='Time (s)', ylabel=['', 'Angle (rad)'], datalim=datalim)

    def read(self) -> list:
        vals = super().read()
        vals = list(vals) + quat2eul(vals)
        
        return vals


class RawPlotter(Plotter):
    def __init__(self, dt, port="COM10", baud=576000, datalim=100):
        super().__init__(dt, port, baud, subplts=[3, 1], lins=[3, 3, 3], lbls=['X', 'Y', 'Z']*3, xlabel='Time (s)', ylabel=['Linear Acceleration (m/s2)', 'Magnetic Field ($\\mu T$)', 'Angular Velocity (rad/s)'], datalim=datalim)


class RawPrePlotter(PrePlotter):
    def __init__(self, samples, dt, port="COM10", baud=576000, datalim=100):
        super().__init__(samples, dt, port, baud, subplts=[3, 1], lins=[3, 3, 3], lbls=['X', 'Y', 'Z']*3, xlabel='Time (s)', ylabel=['Linear Acceleration (m/s2)', 'Magnetic Field ($\\mu T$)', 'Angular Velocity (rad/s)'], datalim=datalim)


if __name__ == '__main__':
    plt.close('all')
    # p = RawPrePlotter(1000, 10)
    p = AnglePlotter(50)
    p.mainloop()
