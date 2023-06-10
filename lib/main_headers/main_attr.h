#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ArduinoEigen.h>
#include <SPI.h>

struct ControllerLevels {
    uint16_t ch00, ch01, ch02, ch03;                // Range 0-1023     Analog High-Res
    uint8_t  ch04, ch05, ch06, ch07;                // Range 0-255      Analog Low-Res
    uint8_t  ch08, ch09, ch10, ch11, ch12, ch13;    // Range 0-3        Quad-State Switch
    bool     ch14, ch15, ch16, ch17, ch18, ch19;    // Range 0-1        Binary Switch
    uint8_t  ch20, ch21;                            // Range 0-7        Oc
};

struct MainAttr {
    // Communication Mutexes
    SemaphoreHandle_t spi2_mutex, uart0_mutex;

    // Shared memory
    Eigen::Quaternion<float> imu_fused;
    Eigen::Vector3f acc_raw, mag_raw, gyr_raw, rpy_fused;
    float motor_level[4];
    ControllerLevels controller_level;
    SemaphoreHandle_t acc_raw_mutex, mag_raw_mutex, gyr_raw_mutex, imu_fused_mutex, motor_level_mutex, controller_level_mutex;

    // Communication
    SPIClass spi2;
};

class Service {
    public:
        Service () {}
        Service (MainAttr *attr_) { this->attr = attr_; }
        virtual bool begin() = 0;

    protected:
        MainAttr *attr = NULL;
};