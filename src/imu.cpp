#include "imu.h"

namespace IMU {
    void task(void *params_) {
        // Get Parameters
        imu_task_parameters params = *((imu_task_parameters *)params_);
        Adafruit_LSM9DS0 *lsm = params.lsm;
        SemaphoreHandle_t *spi_mutex = params.spi_mutex;
        SemaphoreHandle_t *imu_mutex = params.imu_mutex;
        Vector3 *shared_measurement = params.shared_measurement;

        // Setup Sensors
        lsm->setupAccel(lsm->LSM9DS0_ACCELRANGE_8G);
        lsm->setupMag(lsm->LSM9DS0_MAGGAIN_2GAUSS);
        lsm->setupGyro(lsm->LSM9DS0_GYROSCALE_500DPS);

        // Begin read loop
        sensors_event_t accel, mag, gyro, temp;
        while(true) {        
            // Read Sensor Data
            xSemaphoreTake(*spi_mutex, portMAX_DELAY);
            lsm->getEvent(&accel, &mag, &gyro, &temp);
            xSemaphoreGive(*spi_mutex);

            // Fuse Sensor Data
            Vector3 fused = {
                .x = accel.acceleration.x,
                .y = accel.acceleration.y,
                .z = accel.acceleration.z
            };

            // Write to Shared Memory
              xSemaphoreTake(*imu_mutex, portMAX_DELAY);
              *shared_measurement = fused;
              xSemaphoreGive(*imu_mutex);

            Serial.println("IMU: " + String(accel.acceleration.x) + ", " + String(accel.acceleration.y) + ", " + String(accel.acceleration.z));

            // Wait for next cycle
            vTaskDelay(pdMS_TO_TICKS(1000 / 100));
        }
    }
}
