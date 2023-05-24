#include "ahrs.h"

namespace AHRS {
    Eigen::Matrix3f Aa {
        { 0.973209,  0.006248, -0.000107},
        { 0.006248,  0.959535,  0.000811},
        {-0.000107,  0.000811,  1.019086}
    };
    Eigen::Vector3f ba {-15.850987, 46.848109, -15.884498};

    Eigen::Matrix3f Am {
        { 1.024384, -0.067449,  0.005591},
        {-0.067449,  0.984567, -0.059303},
        { 0.005591, -0.059303,  1.093657}
    };
    Eigen::Vector3f bm {16.565142000000002, -74.437627000000006, -3.953151000000000};

    Eigen::Vector3f bg {0.08786329216688561, 0.011035891374165658, 0.20763187215725581};
    
    void measure_task(void *params_) {
        meas_task_parameters *params = (meas_task_parameters *)params_;
        Eigen::Vector3f tmp;

        while(true) {
            if(xSemaphoreTake(*(params->spi_mutex), portMAX_DELAY)){
                if (params->type == 0) {
                    tmp = params->lsm->getAccel();
                    if(params->calib) tmp = Aa * (tmp - ba);
                } else if (params->type == 1) {
                    tmp = params->lsm->getMag();
                    if(params->calib) tmp = Am * (tmp - bm);
                } else if (params->type == 2) {
                    tmp = params->lsm->getGyro();
                    if(params->calib) tmp = tmp - bg;
                } else 
                    return;

                xSemaphoreGive(*(params->spi_mutex));
            }
            

            if(xSemaphoreTake(*(params->data_mutex), 0)){
                *(params->shared_data) = tmp;
                xSemaphoreGive(*(params->data_mutex));
            }

            vTaskDelay(pdMS_TO_TICKS(1000 / params->rate));
        }
    }

    void run_ahrs(void *ahrs_) {
        AHRS *ahrs = (AHRS *)ahrs_;

        while(true) {
            ahrs->update();
            vTaskDelay(pdMS_TO_TICKS(ahrs->dt * 1000));
        }
    }

    void Gyro::update() {
        Eigen::Vector3f tmp;

        if(xSemaphoreTake(*g_mutex, portMAX_DELAY)){
            tmp = *g;
            xSemaphoreGive(*g_mutex);
        }

        rpy_ += tmp * dt;

        if(xSemaphoreTake(*fused_mutex, 0)){
            *rpy = rpy_;
            *q = Eigen::AngleAxisf(rpy_(2), Eigen::Vector3f::UnitZ()) *
                 Eigen::AngleAxisf(rpy_(1), Eigen::Vector3f::UnitY()) *
                 Eigen::AngleAxisf(rpy_(0), Eigen::Vector3f::UnitX());
            q->normalize();

            xSemaphoreGive(*fused_mutex);
        }
    }

    void XMag::update() {
        Eigen::Vector3f a_, m_;

        if(xSemaphoreTake(*a_mutex, portMAX_DELAY)){
            a_ = *a;
            xSemaphoreGive(*a_mutex);
        }

        if(xSemaphoreTake(*m_mutex, portMAX_DELAY)){
            m_ = *m;
            xSemaphoreGive(*m_mutex);
        }
        
        float roll = atan2(a_.y(), a_.z());
        float pitch = asin(-a_.x() / a_.norm());
        rpy_ = Eigen::Vector3f(
            roll,
            pitch,
            atan2(
                -m_.y() * cos(roll) + m_.z() * sin(roll), 
                m_.x() * cos(pitch) + m_.y() * sin(roll) * sin(pitch) + m_.z() * cos(roll) * sin(pitch)
            )
        );

        if(xSemaphoreTake(*fused_mutex, 0)){
            *rpy = rpy_;
            *q = Eigen::AngleAxisf(rpy_(2), Eigen::Vector3f::UnitZ()) *
                 Eigen::AngleAxisf(rpy_(1), Eigen::Vector3f::UnitY()) *
                 Eigen::AngleAxisf(rpy_(0), Eigen::Vector3f::UnitX());
            q->normalize();

            xSemaphoreGive(*fused_mutex);
        }
    }

    void Complementary::update() {
        
        Eigen::Vector3f a_, m_, g_;

        if(xSemaphoreTake(*a_mutex, portMAX_DELAY)){
            a_ = *a;
            xSemaphoreGive(*a_mutex);
        }

        if(xSemaphoreTake(*m_mutex, portMAX_DELAY)){
            m_ = *m;
            xSemaphoreGive(*m_mutex);
        }

        if(xSemaphoreTake(*g_mutex, portMAX_DELAY)){
            g_ = *g;
            xSemaphoreGive(*g_mutex);
        }
        
        float roll = atan2(a_.y(), a_.z());
        float pitch = asin(-a_.x() / a_.norm());
        rpy_xm = Eigen::Vector3f(
            roll,
            pitch,
            atan2(
                -m_.y() * cos(roll) + m_.z() * sin(roll), 
                m_.x() * cos(pitch) + m_.y() * sin(roll) * sin(pitch) + m_.z() * cos(roll) * sin(pitch)
            )
        );

        rpy_g += *rpy + g_ * dt;

        if(xSemaphoreTake(*fused_mutex, 0)){
            *rpy = rpy_xm * alpha + rpy_g * (1 - alpha);
            *q = Eigen::AngleAxisf(rpy->z(), Eigen::Vector3f::UnitZ()) *
                 Eigen::AngleAxisf(rpy->y(), Eigen::Vector3f::UnitY()) *
                 Eigen::AngleAxisf(rpy->x(), Eigen::Vector3f::UnitX());
            q->normalize();

            xSemaphoreGive(*fused_mutex);
        }
    }

    void Madgwick::update() {
        Eigen::Vector3f a_, m_, g_;

        // Get data
        if(xSemaphoreTake(*a_mutex, portMAX_DELAY)){
            a_ = *a;
            xSemaphoreGive(*a_mutex);
        }

        if(xSemaphoreTake(*m_mutex, portMAX_DELAY)){
            m_ = *m;
            xSemaphoreGive(*m_mutex);
        }

        if(xSemaphoreTake(*g_mutex, portMAX_DELAY)){
            g_ = *g;
            xSemaphoreGive(*g_mutex);
        }

        // Normalize accelerometer and magnetometer data
        a_.normalize();
        m_.normalize();
        
        // Calculate reference direction of magnetic field
        Eigen::Quaternion<float> h = q_ * Eigen::Quaternion<float>(0, m_.x(), m_.y(), m_.z()) * q_.conjugate();
        Eigen::Vector3f b = Eigen::Vector3f(0, sqrt(h.x() * h.x() + h.y() * h.y()), h.z());

        // Generate target function
        Eigen::VectorXf F_gb(6);
        F_gb << 2 * (q_.x() * q_.z() - q_.w() * q_.y()) - a_.x(),
                2 * (q_.w() * q_.x() + q_.y() * q_.z()) - a_.y(),
                2 * (0.5 - q_.x() * q_.x() - q_.y() * q_.y()) - a_.z(),
                2 * b.x() * (0.5 - q_.y() * q_.y() - q_.z() * q_.z()) + 2 * b.z() * (q_.x() * q_.z() - q_.w() * q_.y()) - m_.x(),
                2 * b.x() * (q_.x() * q_.y() - q_.w() * q_.z()) + 2 * b.z() * (q_.w() * q_.x() + q_.y() * q_.z()) - m_.y(),
                2 * b.x() * (q_.w() * q_.y() + q_.x() * q_.z()) + 2 * b.z() * (0.5 - q_.x() * q_.x() - q_.y() * q_.y()) - m_.z();

        // Calculate Jacobian
        Eigen::MatrixXf J_gb(6, 4);
        J_gb << -2 * q_.y(), 2 * q_.z(), -2 * q_.w(), 2 * q_.x(),
                2 * q_.x(), 2 * q_.w(), 2 * q_.z(), 2 * q_.y(),
                0, -4 * q_.x(), -4 * q_.y(), 0,
                -2 * b.z() * q_.y(), 2 * b.z() * q_.z(), -4 * b.x() * q_.y() - 2 * b.z() * q_.w(), -4 * b.x() * q_.z() + 2 * b.z() * q_.x(),
                -2 * b.x() * q_.z() + 2 * b.z() * q_.x(), 2 * b.x() * q_.y() + 2 * b.z() * q_.w(), 2 * b.x() * q_.x() + 2 * b.z() * q_.z(), -2 * b.x() * q_.w() + 2 * b.z() * q_.y(),
                2 * b.x() * q_.y(), 2 * b.x() * q_.z() - 4 * b.z() * q_.x(), 2 * b.x() * q_.w() - 4 * b.z() * q_.y(), 2 * b.x() * q_.x();

        // Calculate gradient
        Eigen::Vector4f grad = J_gb.transpose() * F_gb;
        grad.normalize();

        // Compensate for gyro bias
        Eigen::Quaternion<float> grad_q(grad(0), grad(1), grad(2), grad(3));
        w_b.coeffs() += 2 * (q_.conjugate() * grad_q).coeffs() * dt;

        Eigen::Quaternion<float> q_w(0 - zeta * w_b.w(), g_.x() - zeta * w_b.x(), g_.y() - zeta * w_b.y(), g_.z() - zeta * w_b.z());
        Eigen::Quaternion<float> q_w_dot = q_ * q_w;
        q_w_dot.coeffs() *= 0.5;

        // Apply gradient descent
        Eigen::Quaternion<float> q_dot = q_w_dot;
        q_dot.coeffs() -= beta * grad;

        // Integrate
        q_dot.coeffs() *= dt;
        q_.coeffs() += q_dot.coeffs();
        q_.normalize();

        if(xSemaphoreTake(*fused_mutex, 0)){
            *q = q_;
            q->normalize();

            *rpy = q->toRotationMatrix().eulerAngles(0, 1, 2);

            xSemaphoreGive(*fused_mutex);
        }
    }
}
