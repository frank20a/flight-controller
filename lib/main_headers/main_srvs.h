#pragma once

#include "ahrs.h"
#include "lsm9ds0.h"
#include "controller.h"
#include "radio.h"
// #include "cmd_handler.h"
#ifdef DEBUG
    #include "debugger.h"
#endif

struct MainSrvs {
    // Sensors
    LSM9DS0 lsm;

    // AHRS Filters
    #if defined(AHRS_GYRO)
        AHRS::Gyro ahrs_filter;
    #elif defined(AHRS_XMAG)
        AHRS::XMag ahrs_filter;
    #elif defined(AHRS_COMPLEMENTARY)
        AHRS::Complementary ahrs_filter;
    #elif defined(AHRS_MAHONNY)
        #error "Mahonny not implemented yet"
    #elif defined(AHRS_MADGWICK)
        AHRS::Madgwick ahrs_filter;
    #elif defined(AHRS_MADGWICK_OPTIMIZED)
        AHRS::MadgwickOptimized ahrs_filter;
    #endif

    // Debugger
    #ifdef DEBUG
    Debugger debugger;
    #endif

    // // Controller
    Controller::MotorController controller;

    // // Radio
    Radio radio;
    
};
