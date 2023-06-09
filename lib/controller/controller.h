#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ArduinoEigen.h>
#include <SPI.h>

#include "main_attr.h"

namespace Controller {
    class PID {
        public:
            PID(float kp, float ki, float kd, float dt) { this->kp = kp; this->ki = ki; this->kd = kd; this->dt = dt; this->reset(); }
            float update(float error);
            void reset();
        private:
            float kp, ki, kd, dt;
            float integral, derivative, prev_error;
    };

    class MotorController : public Service {
        public:
            MotorController(MainAttr *attr_) : Service(attr_) {
                pid_r = PID(PID_KP, PID_KI, PID_KD, PID_DT);
                pid_p = PID(PID_KP, PID_KI, PID_KD, PID_DT);
                pid_y = PID(PID_KP, PID_KI, PID_KD, PID_DT);
                pid_z = PID(PID_KP, PID_KI, PID_KD, PID_DT);
            };
            bool begin();
        
        protected:
            void update();
            static void task_wrapper(void *pvParam) {
                static_cast<MotorController*>(pvParam)->update();
            }

            PID pid_r, pid_p, pid_y, pid_z;

    };
}