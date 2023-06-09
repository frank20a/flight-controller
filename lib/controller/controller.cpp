#include "controller.h"

namespace CONTROLLER {
    float PID::update(float error) {
        integral += error * dt;
        derivative = (error - prev_error) / dt;
        prev_error = error;
        return kp * error + ki * integral + kd * derivative;
    }

    void PID::reset() {
        integral = 0;
        derivative = 0;
        prev_error = 0;
    }

    MotorController::MotorController(
        Eigen::Vector3f *g_,
        Eigen::Vector3f *rpy_,
        SemaphoreHandle_t *fused_mutex_,
        SemaphoreHandle_t *gyro_mutex_
    ) {
        g = g_;
        rpy = rpy_;
        fused_mutex = fused_mutex_;
        gyro_mutex = gyro_mutex_;

        pid_r = PID(PID_KP, PID_KI, PID_KD, PID_DT);
    }
    
    bool MotorController::begin() {
        xTaskCreatePinnedToCore(task_wrapper, "AHRS", 512, this, 1, NULL, 1);
        return true;
    }



};