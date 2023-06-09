#pragma once

#include <ArduinoEigen.h>

namespace CONTROLLER {
    class PID {
        public:
            PID(float kp, float ki, float kd, float dt) { this->kp = kp; this->ki = ki; this->kd = kd; this->dt = dt; this->reset(); }
            float update(float error);
            void reset();
        private:
            float kp, ki, kd, dt;
            float integral, derivative, prev_error;
    };

    class MotorController {
        public:
            MotorController(
                Eigen::Vector3f *g_,
                Eigen::Vector3f *rpy_,
                SemaphoreHandle_t *fused_mutex_,
                SemaphoreHandle_t *gyro_mutex_
            );
            bool begin();
        
        protected:
            void update();
            static void task_wrapper(void *pvParam) {
                static_cast<MotorController*>(pvParam)->update();
            }

            Eigen::Vector3f *g, *rpy;
            SemaphoreHandle_t *fused_mutex, *gyro_mutex;
            PID pid_r, pid_p, pid_y, pid_z;

    };
}