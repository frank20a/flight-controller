#include "ahrs.h"

namespace AHRS {
    Eigen::Matrix3f Aa {
        { 0.972959607701079, -0.000000110451098, -0.000001623424595},
        { 0.000000112019011,  0.959341241072106, -0.000000670934736},
        { 0.000001550036871,  0.000000631638348,  1.019025151197725}
    };
    Eigen::Vector3f ba {-0.608390088442559, 0.194751778040356, -0.228696615444335};

    Eigen::Matrix3f Am {
        { 1.030310000000000, -0.062524000000000,  0.016296000000000},
        {-0.062524000000000,  1.008574000000000, -0.066112000000000},
        { 0.016296000000000, -0.066112000000000,  1.167872000000000}
    };
    Eigen::Vector3f bm {16.565142000000002, -74.437627000000006, -3.953151000000000};

    Eigen::Vector3f bg {0.100986880159907, 0.089347646711130, -0.045182741880317};


    void measure_task(void *params_) {
        ahrs_task_parameters *params = (ahrs_task_parameters *)params_;
        Eigen::Vector3f tmp;

        while(true) {
            if(xSemaphoreTake(*(params->spi_mutex), portMAX_DELAY)){
                if (params->type == 0) {
                    tmp = params->lsm->getAccel();
                    tmp = Aa * (tmp - ba);
                } else if (params->type == 1) {
                    tmp = params->lsm->getMag();
                    tmp = Am * (tmp - bm);
                } else if (params->type == 2) {
                    tmp = params->lsm->getGyro();
                    tmp = tmp - bg;
                } else 
                    return;

                xSemaphoreGive(*(params->spi_mutex));
            }
            

            if(xSemaphoreTake(*(params->data_mutex), 0)){
                *(params->shared_data) = tmp;
                xSemaphoreGive(*(params->data_mutex));
            }

            vTaskDelay(pdMS_TO_TICKS(1000 / params->rate));
        }
    }

    void madwick_task(void *params_) {
        madwick_task_parameters *params = (madwick_task_parameters *)params_;
        Eigen::Vector3f acc_raw, mag_raw, gyro_raw;

        SF fusion;

        while(true) {
            if(xSemaphoreTake(*(params->acc_raw_mutex), portMAX_DELAY)){
                acc_raw = *(params->acc_raw);
                xSemaphoreGive(*(params->acc_raw_mutex));
            }
            if(xSemaphoreTake(*(params->mag_raw_mutex), portMAX_DELAY)){
                mag_raw = *(params->mag_raw);
                xSemaphoreGive(*(params->mag_raw_mutex));
            }
            if(xSemaphoreTake(*(params->gyro_raw_mutex), portMAX_DELAY)){
                gyro_raw = *(params->gyro_raw);
                xSemaphoreGive(*(params->gyro_raw_mutex));
            }

            // implement madwick filter here
            // float dt = fusion.deltatUpdate();
            fusion.MadgwickUpdate(gyro_raw(0), gyro_raw(1), gyro_raw(2), acc_raw(0), acc_raw(1), acc_raw(2), mag_raw(0), mag_raw(1), mag_raw(2), 1.0 / params->rate);

            if(xSemaphoreTake(*(params->fused_mutex), 0)){
                float *tmp = fusion.getQuat();
                *(params->fused) = Eigen::Quaternion<float>(tmp[0], tmp[1], tmp[2], tmp[3]);
                xSemaphoreGive(*(params->fused_mutex));
            }

            vTaskDelay(pdMS_TO_TICKS(1000 / params->rate));
        }
    }

}
