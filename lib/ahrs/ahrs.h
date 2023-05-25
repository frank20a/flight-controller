#pragma once

// #ifndef AHRS_ALG
//     #error AHRS_ALG must be defined
// #endif

#include <freertos/FreeRTOS.h>
#include <lsm9ds0.h>
#include <ArduinoEigen.h>
#include <cmath>

namespace AHRS {
    struct meas_task_parameters {
        LSM9DS0::LSM9DS0 *lsm;
        SemaphoreHandle_t *spi_mutex, *data_mutex;
        Eigen::Vector3f *shared_data;
        int rate;
        byte type;
        bool calib;
    };

    void measure_task(void *params_);

    struct ahrs_init_parameters {
        Eigen::Vector3f *a, *m, *g, *rpy;
        Eigen::Quaternion<float> *q;
        SemaphoreHandle_t *a_mutex, *m_mutex, *g_mutex, *fused_mutex;
        float dt;
    };

    class AHRS {
        public:
            AHRS(void *params_){
                ahrs_init_parameters *params = (ahrs_init_parameters *) params_;

                this->a = params->a;
                this->m = params->m;
                this->g = params->g;
                this->q = params->q;
                this->rpy = params->rpy;

                this->a_mutex = params->a_mutex;
                this->m_mutex = params->m_mutex;
                this->g_mutex = params->g_mutex;
                this->fused_mutex = params->fused_mutex;

                this->dt = params->dt;

                this->q->setIdentity();
                this->rpy->setZero();
            }
            void task();
            void start_task();
            static void task_wrapper(void *pvParam) {
                static_cast<AHRS*>(pvParam)->task();
            }
            virtual void update() = 0;

            float dt;

        protected:
            Eigen::Vector3f *a, *m, *g, *rpy;
            Eigen::Quaternion<float> *q;
            SemaphoreHandle_t *a_mutex, *m_mutex, *g_mutex, *fused_mutex;
    };

    class Gyro : public AHRS {
        public:
            Gyro(void *params_) : AHRS(params_) { rpy_.setZero(); };
            void update() override;

        private:
            Eigen::Vector3f rpy_;
    };

    class XMag : public AHRS {
        public:
            XMag(void *params_) : AHRS(params_) { rpy_.setZero(); };
            void update() override;

        private:
            Eigen::Vector3f rpy_;
    };

    class Complementary : public AHRS {
        public:
            Complementary(void *params_, float gain) : AHRS(params_) { 
                rpy_xm.setZero();
                rpy_g.setZero();
                alpha = gain;
            };
            void update() override;

        private:
            Eigen::Vector3f rpy_xm, rpy_g;
            float alpha = 0.02;
    };

    class Madgwick : public AHRS {
        public:
            Madgwick(void *params_, float beta, float zeta) : AHRS(params_) { 
                this->beta = sqrt(3.0/4.0) * beta;
                this->zeta = sqrt(3.0/4.0) * zeta;
                q_.setIdentity();
                w_b.setIdentity();
            };
            void update() override;

        private:
            Eigen::Quaternion<float> q_, w_b;
            float beta, zeta;
    };

    void run_ahrs(void *ahrs);
}