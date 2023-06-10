#pragma once

#include "main_srvs.h"

// IMU Sensor Types
#define IMU_SENSORTYPE_ACCEL 0 
#define IMU_SENSORTYPE_MAG 1
#define IMU_SENSORTYPE_GYRO 2

// FreeRTOS Config
#define CONFIG_FREERTOS_HZ 5000


MainAttr main_attr = {
    .spi2_mutex = xSemaphoreCreateMutex(),
    .uart0_mutex = xSemaphoreCreateMutex(),
    .imu_fused = Eigen::Quaternion<float>(1, 0, 0, 0),
    .acc_raw = Eigen::Vector3f(0, 0, 0),
    .mag_raw = Eigen::Vector3f(0, 0, 0),
    .gyr_raw = Eigen::Vector3f(0, 0, 0),
    .rpy_fused = Eigen::Vector3f(0, 0, 0),
    .motor_level = {0, 0, 0, 0},
    .controller_level = {0, 0, 0, 0},
    .acc_raw_mutex = xSemaphoreCreateMutex(),
    .mag_raw_mutex = xSemaphoreCreateMutex(),
    .gyr_raw_mutex = xSemaphoreCreateMutex(),
    .imu_fused_mutex = xSemaphoreCreateMutex(),
    .motor_level_mutex = xSemaphoreCreateMutex(),
    .controller_level_mutex = xSemaphoreCreateMutex(),
    .spi2 = SPIClass(HSPI),
};

MainSrvs main_srvs = {
    .lsm = LSM9DS0(&main_attr),
    .ahrs_filter = 
        #if defined(AHRS_GYRO)
            AHRS::Gyro(&main_attr),
        #elif defined(AHRS_XMAG)
            AHRS::XMag(&main_attr),
        #elif defined(AHRS_COMPLEMENTARY)
            AHRS::Complementary(&main_attr),
        #elif defined(AHRS_MAHONNY)
            #error "Mahonny not implemented yet"
        #elif defined(AHRS_MADGWICK)
            AHRS::Madgwick(&main_attr),
        #elif defined(AHRS_MADGWICK_OPTIMIZED)
            AHRS::MadgwickOptimized(&main_attr),
        #endif
    #ifdef DEBUG
    .debugger = Debugger(&main_attr),
    #endif
    .controller = Controller::MotorController(&main_attr),
    .radio = Radio(&main_attr),
};