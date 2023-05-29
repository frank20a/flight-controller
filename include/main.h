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
Eigen::Vector3f rpy_fused;
SemaphoreHandle_t imu_fused_mutex = xSemaphoreCreateMutex();
Eigen::Vector3f acc_raw, mag_raw, gyro_raw;
SemaphoreHandle_t acc_raw_mutex = xSemaphoreCreateMutex();
SemaphoreHandle_t mag_raw_mutex = xSemaphoreCreateMutex();
SemaphoreHandle_t gyro_raw_mutex = xSemaphoreCreateMutex();

// Communication
SPIClass SPI2 = SPIClass(HSPI);

// Sensors
LSM9DS0::LSM9DS0 lsm = LSM9DS0::LSM9DS0(&SPI2);

// Task Parameters
AHRS::meas_task_parameters params_a = {
    .lsm = &lsm,
    .spi_mutex = &spi2_mutex,
    .data_mutex = &acc_raw_mutex,
    .shared_data = &acc_raw,
    .rate = 400,
    .type = IMU_SENSORTYPE_ACCEL,
    .calib = USE_CALIBRATE,
};
AHRS::meas_task_parameters params_m = {
    .lsm = &lsm,
    .spi_mutex = &spi2_mutex,
    .data_mutex = &mag_raw_mutex,
    .shared_data = &mag_raw,
    .rate = 100,
    .type = IMU_SENSORTYPE_MAG,
    .calib = USE_CALIBRATE,
};
AHRS::meas_task_parameters params_g = {
    .lsm = &lsm,
    .spi_mutex = &spi2_mutex,
    .data_mutex = &gyro_raw_mutex,
    .shared_data = &gyro_raw,
    .rate = 190,
    .type = IMU_SENSORTYPE_GYRO,
    .calib = USE_CALIBRATE,
};
AHRS::ahrs_init_parameters params_filter = {
    .a = &acc_raw,
    .m = &mag_raw,
    .g = &gyro_raw,
    .rpy = &rpy_fused,
    .q = &imu_fused,
    .a_mutex = &acc_raw_mutex,
    .m_mutex = &mag_raw_mutex,
    .g_mutex = &gyro_raw_mutex,
    .fused_mutex = &imu_fused_mutex,
    .dt = 1.0 / AHRS_RATE,
};

// AHRS Filters
#if defined(AHRS_GYRO)
    AHRS::Gyro *ahrs_filter = new AHRS::Gyro(&params_filter); 
#elif defined(AHRS_XMAG)
    AHRS::XMag *ahrs_filter = new AHRS::XMag(&params_filter);
#elif defined(AHRS_COMPLEMENTARY)
    #ifndef AHRS_COMPLEMENTARY_ALPHA
        #error "AHRS_COMPLEMENTARY_BETA not defined"
    #endif
    AHRS::Complementary *ahrs_filter = new AHRS::Complementary(&params_filter, AHRS_COMPLEMENTARY_ALPHA);
#elif defined(AHRS_MAHONY)
    #if !defined(AHRS_MAHONY_KI) || !defined(AHRS_MAHONY_KP)
        #error "AHRS_MAHONY_KI and/or AHRS_MAHONY_KP not defined"
    #endif
#elif defined(AHRS_MADGWICK)
    #if !defined(AHRS_MADGWICK_BETA) || !defined(AHRS_MADGWICK_ZETA)
        #error "AHRS_MADGWICK_BETA and/or AHRS_MADGWICK_ZETA not defined"
    #endif
    AHRS::Madgwick *ahrs_filter = new AHRS::Madgwick(&params_filter, AHRS_MADGWICK_BETA, AHRS_MADGWICK_ZETA);
#elif defined(AHRS_MADGWICK_OPTIMIZED)
    #if !defined(AHRS_MADGWICK_BETA) || !defined(AHRS_MADGWICK_ZETA)
        #error "AHRS_MADGWICK_BETA and/or AHRS_MADGWICK_ZETA not defined"
    #endif
    AHRS::MadgwickOptimized *ahrs_filter = new AHRS::MadgwickOptimized(&params_filter, AHRS_MADGWICK_BETA);
#endif

// Utility Functions
void write_imu_fused();
void print_imu_fused();
void write_imu_raw();
void print_imu_raw();