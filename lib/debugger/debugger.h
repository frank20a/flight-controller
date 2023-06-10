#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ArduinoEigen.h>
#include <SPI.h>

#include "main_attr.h"

class Debugger : public Service {
    public:
        Debugger(MainAttr *attr_) : Service(attr_) {
            this->config(DEBUG_FLAGS, DEBUG_PERIOD);
        }
        bool begin() override;
        void config(char flags, unsigned short dt_);

    private:
        static void task_wrapper(void *pvParam) {
            static_cast<Debugger*>(pvParam)->debug_msg();
        }
        void debug_msg();
        void imu_debugger();
        void ahrs_debugger();
        void radio_debugger();
        void control_debugger();

        bool debug_io_mode;
        char debug_type;
        unsigned short dt;
};