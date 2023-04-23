# IMU Calibration

## Accelerometer

### Process

We consider a model of the form: 

$ a = T_a \cdot K_a \cdot (a_m + b_a)$, where $a_m$ is the measured acceleration, $a$ is the calibrated acceleration, $b_a$ is a bias vector, $T_a = \begin{bmatrix}  1 & \Delta\psi_a & -\Delta\theta_a \\ -\Delta\psi_a & 1 & \Delta\phi_a \\ \Delta\theta_a & -\Delta\phi_a & 1 \end{bmatrix}$ denotes a tilt introduced during imperfect placement of the sensors, and $K_a = \begin{bmatrix} s_{ax} & 0 & 0 \\ 0 & s_{ay} & 0 \\ 0 & 0 & s_{az} \end{bmatrix}$ is a scaling matrix.

To calibrate this model, multiple measurements of the acceleration are taken while the sensor is rotated a full circle around each of its axis. On every measurement, the sensor is held stationary and multiple measurements are averaged to reduce noise. The measurements are then used to solve the following optimization problem:

$\Theta^*_a = \argmin_{\Theta_a \in \mathbb{R}^9} \sum_{k=1}^M (\left\| T_a \cdot K_a \cdot (a_m + b_a) \right\|- g)^2$
Where $\Theta_a = [\Delta\phi_a, \Delta\theta_a, \Delta\psi_a, s_{ax}, s_{ay}, s_{az}, b_{ax}, b_{ay}, b_{az}]$

### Results

![Accelerometer Calibration](img/Accelerometer%20Calibration.png)