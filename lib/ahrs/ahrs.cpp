#include "ahrs.h"

namespace AHRS {
    void measure_task(void *params_) {
        ahrs_task_parameters *params = (ahrs_task_parameters *)params_;
        Eigen::Vector3f tmp;

        while(true) {
            if(xSemaphoreTake(*(params->spi_mutex), portMAX_DELAY)){
                if (params->type == 0) tmp = params->lsm->getAccel();
                else if (params->type == 1) tmp = params->lsm->getMag();
                else if (params->type == 2) tmp = params->lsm->getGyro();
                else return;

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
            float dt = fusion.deltatUpdate();
            fusion.MadgwickUpdate(gyro_raw(0), gyro_raw(1), gyro_raw(2), acc_raw(0), acc_raw(1), acc_raw(2), mag_raw(0), mag_raw(1), mag_raw(2), dt);

            if(xSemaphoreTake(*(params->fused_mutex), 0)){
                float *tmp = fusion.getQuat();
                *(params->fused) = Eigen::Quaternion<float>(tmp[0], tmp[1], tmp[2], tmp[3]);
                xSemaphoreGive(*(params->fused_mutex));
            }

            vTaskDelay(pdMS_TO_TICKS(1000 / params->rate));
        }
    }

}
