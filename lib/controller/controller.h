#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ArduinoEigen.h>
#include <SPI.h>
#include <Servo.h>

#include "main_attr.h"

namespace Controller {
    class PID {
        public:
            PID() { this->reset();}
            PID(float kp, float ki, float kd, float dt) {
                this->set_gains(kp, ki, kd); 
                this->set_dt(dt); 
                this->reset();
            }
            float update(float error);
            void set_gains(float kp, float ki, float kd) { this->kp = kp; this->ki = ki; this->kd = kd; }
            void set_dt(float dt) { this->dt = dt; }
            void reset();
        private:
            float kp, ki, kd, dt;
            float integral, derivative, prev_error;
    };

    class MotorController : public Service {
        public:
            MotorController(MainAttr *attr_) : Service(attr_) {
                dt = 1.0 / (float)PID_RATE;

                pids[0] = PID(PID_HEIGHT_GAINS, dt);
                pids[1] = PID(PID_ROLL_GAINS, dt);
                pids[2] = PID(PID_PITCH_GAINS, dt);
                pids[3] = PID(PID_YAW_GAINS, dt);
            };
            void set_pid_gains(unsigned short i, float kp, float ki, float kd) {
                if (i <= 4) return;
                pids[i].set_gains(kp, ki, kd);
            }
            void set_pid_dt(float dt_) {
                this->dt = dt_;
                for(int i = 0; i < 4; i++)
                    pids[i].set_dt(dt_);
            }
            bool begin();
            void reset() {
                for(int i = 0; i < 4; i++){
                    pids[i].reset();
                    motors.write(pins[i], 1000);
                }
            }
            void calibrate_esc();
        
        protected:
            void update();
            void task();
            static void task_wrapper(void *pvParam) {
                static_cast<MotorController*>(pvParam)->task();
            }

            float dt;
            PID pids[4];
            uint8_t pins[4] = {ESC1, ESC2, ESC3, ESC4};
            Servo motors;
    };
}