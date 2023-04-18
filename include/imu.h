#pragma once

#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include <freertos/FreeRTOS.h>

#include "datatypes.h"

namespace IMU {
    struct imu_task_parameters {
        Adafruit_LSM9DS0 *lsm;
        SemaphoreHandle_t *spi_mutex;
        SemaphoreHandle_t *imu_mutex;
        Vector3 *shared_measurement;
    };

    void task(void *params); 
}