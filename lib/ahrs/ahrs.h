#pragma once

#include <freertos/FreeRTOS.h>
#include <lsm9ds0.h>
#include <ArduinoEigen.h>

namespace AHRS {
    struct ahrs_task_parameters {
        LSM9DS0::LSM9DS0 *lsm;
        SemaphoreHandle_t *spi_mutex, *data_mutex;
        Eigen::Vector3d *shared_data;
        int rate;
        byte type;
    };

    void measure_task(void *params_);
}