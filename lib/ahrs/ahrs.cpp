#include "ahrs.h"

namespace AHRS {
    void measure_task(void *params_) {
        ahrs_task_parameters *params = (ahrs_task_parameters *)params_;
        Vector3 tmp;

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

}
