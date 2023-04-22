#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <vector>
#include <ArduinoEigen.h>

#include "ahrs.h"
#include "lsm9ds0.h"

// IMU Sensor Types
#define IMU_SENSORTYPE_ACCEL 0 
#define IMU_SENSORTYPE_MAG 1
#define IMU_SENSORTYPE_GYRO 2

// FreeRTOS Config
#define CONFIG_FREERTOS_HZ 5000

// Communication Mutexes
SemaphoreHandle_t spi2_mutex = xSemaphoreCreateMutex();
SemaphoreHandle_t uart0_mutex = xSemaphoreCreateMutex();

// Shared memory
Eigen::Quaternion<float> imu_fused;
SemaphoreHandle_t imu_fused_mutex = xSemaphoreCreateMutex();
Eigen::Vector3d acc_raw, mag_raw, gyro_raw;
SemaphoreHandle_t acc_raw_mutex = xSemaphoreCreateMutex();
SemaphoreHandle_t mag_raw_mutex = xSemaphoreCreateMutex();
SemaphoreHandle_t gyro_raw_mutex = xSemaphoreCreateMutex();

// Communication
SPIClass SPI2 = SPIClass(HSPI);

// Sensors
LSM9DS0::LSM9DS0 lsm = LSM9DS0::LSM9DS0(&SPI2);

// Task Parameters
AHRS::ahrs_task_parameters params_a, params_m, params_g;