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

## Magnetometer

### Process

We consider a model of the form:

$\omega = A_g \cdot (\omega_m + b_g)$, where $\omega_m$ is the measured angular velocity, $\omega$ is the calibrated angular velocity, $b_g$ is a bias vector, $A_m$ is a dense matrix that contains both the tilt introduced during imperfect placement of the sensors and scaling errors. This is a simplified version of the model used in the accelerometer calibration.

To calibrate this model, multiple measurements of the angular velocity are taken while the sensor is rotated in a 3D 8 shape. The sensor is continuously moved aiming to cover the full spherical range of the sensor. 

$\Theta^*_a = \argmin_{\Theta_a \in \mathbb{R}^9} \sum_{k=1}^M (\left\| A_m \cdot (a_m + b_a) \right\|- M)^2$
Where $\Theta_a = [Am_{11}, Am_{12}, Am_{13}, Am_{21}, Am_{22}, Am_{23}, Am_{31}, Am_{32}, Am_{33}, b_{ax}, b_{ay}, b_{az}]$ and M is the expected magnetic field that can be calculated [here](https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm).

### Results

![Magnetometer Calibration](img/Magnetometer%20Calibration.png)

## Note

Python's scipy optimization algorithms are pretty bad. Their results are not always very accurate, they require very good initialization and they are very slow. For this reason, [this](https://www.navsparkforum.com.tw/viewtopic.php?t=1673) software was used to calculate the $A_m$ matrix for the magnetometer calibration.