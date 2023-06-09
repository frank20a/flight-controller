#pragma once

#include <freertos/FreeRTOS.h>
#include <ArduinoEigen.h>
#include <cmath>

namespace AHRS {
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
            bool begin();

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
            Gyro(void *params_) : AHRS(params_) { rpy_.setZero(); };
            void update() override;

        protected:
            Eigen::Vector3f rpy_;
    };

    class XMag : public AHRS {
        public:
            XMag(void *params_) : AHRS(params_) { rpy_.setZero(); };
            void update() override;

        protected:
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

        protected:
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

        protected:
            float beta, zeta;
        
        private:
            Eigen::Quaternion<float> q_, w_b;
    };

    class MadgwickOptimized : public Madgwick {
        public:
            MadgwickOptimized(void *params_, float beta) : Madgwick(params_, beta, 0.0f) {};
            void update() override;

        private:
            float b_x = 1, b_z = 0;
            float SEq_1 = 1, SEq_2 = 0, SEq_3 = 0, SEq_4 = 0;
            float w_bx = 0, w_by = 0, w_bz = 0;
    };

}