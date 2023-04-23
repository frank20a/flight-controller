#pragma once

#include <freertos/FreeRTOS.h>
#include <lsm9ds0.h>
#include <ArduinoEigen.h>

#include "SensorFusion.h"

namespace AHRS {

    struct ahrs_task_parameters {
        LSM9DS0::LSM9DS0 *lsm;
        SemaphoreHandle_t *spi_mutex, *data_mutex;
        Eigen::Vector3f *shared_data;
        int rate;
        byte type;
    };

    void measure_task(void *params_);

    struct madwick_task_parameters {
        Eigen::Quaternion<float> *fused;
        SemaphoreHandle_t *fused_mutex;
        Eigen::Vector3f *acc_raw, *mag_raw, *gyro_raw;
        SemaphoreHandle_t *acc_raw_mutex, *mag_raw_mutex, *gyro_raw_mutex;
        float beta;
        int rate;
    };

    void madwick_task(void *params_);
}