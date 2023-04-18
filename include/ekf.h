#pragma once 

#include <cmath>
#include <Adafruit_Sensor.h>

class IMU_EKF {
    public:
        IMU_EKF();
        void update(sensors_vec_t mag);
        void predict(sensors_vec_t accel, sensors_vec_t gyro);

    private:
        float x[4];
        float P[4][4];
        float Q[4][4];
        float R[4][4];
};