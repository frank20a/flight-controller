#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ArduinoEigen.h>
#include <SPI.h>
#include <cmath>

#include "main_attr.h"

namespace AHRS {

    class AHRS : public Service {
        public:
            AHRS(MainAttr *attr_) {
                this->a = &(attr_->acc_raw);
                this->m = &(attr_->mag_raw);
                this->g = &(attr_->gyr_raw);
                this->q = &(attr_->imu_fused);
                this->rpy = &(attr_->rpy_fused);

                this->a_mutex = &(attr_->acc_raw_mutex);
                this->m_mutex = &(attr_->mag_raw_mutex);
                this->g_mutex = &(attr_->gyr_raw_mutex);
                this->fused_mutex = &(attr_->imu_fused_mutex);

                this->q->setIdentity();
                this->rpy->setZero();

                this->set_dt(1.0 / AHRS_RATE);
            }
            bool begin() override;
            void set_dt(float dt_) { this->dt = dt_; };
            virtual void set_gains(float gain_a, float gain_b) {};

        protected:
            virtual void update() = 0;
            void task();
            static void task_wrapper(void *pvParam) {
                static_cast<AHRS*>(pvParam)->task();
            }
            
            float dt;
            Eigen::Vector3f *a, *m, *g, *rpy;
            Eigen::Quaternion<float> *q;
            SemaphoreHandle_t *a_mutex, *m_mutex, *g_mutex, *fused_mutex;
    };

    class Gyro : public AHRS {
        public:
            Gyro(MainAttr *attr) : AHRS(attr) { rpy_.setZero(); };
            void update() override;

        protected:
            Eigen::Vector3f rpy_;
    };

    class XMag : public AHRS {
        public:
            XMag(MainAttr *attr) : AHRS(attr) { rpy_.setZero(); };
            void update() override;

        protected:
            Eigen::Vector3f rpy_;
    };

    class Complementary : public AHRS {
        public:
            Complementary(MainAttr *attr) : AHRS(attr) { 
                rpy_xm.setZero();
                rpy_g.setZero();
                this->set_gains(AHRS_GAIN_A, 0);
            };
            void update() override;
            void set_gains(float gain_a, float gain_b) override { alpha = gain_a; };

        protected:
            Eigen::Vector3f rpy_xm, rpy_g;
            float alpha = 0.02;
    };

    class Madgwick : public AHRS {
        public:
            Madgwick(MainAttr *attr) : AHRS(attr) { 
                q_.setIdentity();
                w_b.setIdentity();

                this->set_gains(AHRS_GAIN_A, AHRS_GAIN_B);
            };
            void update() override;
            void set_gains(float gain_a, float gain_b) override { beta = sqrt(3.0/4.0) * gain_a; zeta = sqrt(3.0/4.0) * gain_b; };

        protected:
            float beta, zeta;
        
        private:
            Eigen::Quaternion<float> q_, w_b;
    };

    class MadgwickOptimized : public Madgwick {
        public:
            MadgwickOptimized(MainAttr *attr) : Madgwick(attr) {};
            void update() override;

        private:
            float b_x = 1, b_z = 0;
            float SEq_1 = 1, SEq_2 = 0, SEq_3 = 0, SEq_4 = 0;
            float w_bx = 0, w_by = 0, w_bz = 0;
    };

}