#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ArduinoEigen.h>
#include <SPI.h>

struct MainAttr {
    // Communication Mutexes
    SemaphoreHandle_t spi2_mutex, uart0_mutex;

    // Shared memory
    Eigen::Quaternion<float> imu_fused;
    Eigen::Vector3f acc_raw, mag_raw, gyr_raw, rpy_fused;
    SemaphoreHandle_t acc_raw_mutex, mag_raw_mutex, gyr_raw_mutex, imu_fused_mutex;

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