#include "ahrs.h"

Eigen::Vector3f quat2eul(Eigen::Quaternion<float> q){
    Eigen::Vector3f rpy;
    rpy(0) = atan2(2*(q.w()*q.x() + q.y()*q.z()), 1 - 2*(q.x()*q.x() + q.y()*q.y()));
    rpy(1) = asin(2*(q.w()*q.y() - q.z()*q.x()));
    rpy(2) = atan2(2*(q.w()*q.z() + q.x()*q.y()), 1 - 2*(q.y()*q.y() + q.z()*q.z()));
    return rpy;
}

Eigen::Quaternion<float> eul2quat(Eigen::Vector3f rpy){
    Eigen::Quaternion<float> q;
    float cy = cos(rpy(2) * 0.5);
    float sy = sin(rpy(2) * 0.5);
    float cp = cos(rpy(1) * 0.5);
    float sp = sin(rpy(1) * 0.5);
    float cr = cos(rpy(0) * 0.5);
    float sr = sin(rpy(0) * 0.5);

    q.w() = cy * cp * cr + sy * sp * sr;
    q.x() = cy * cp * sr - sy * sp * cr;
    q.y() = sy * cp * sr + cy * sp * cr;
    q.z() = sy * cp * cr - cy * sp * sr;

    return q;
}


namespace AHRS {

    float invSqrt(float x) {
        float halfx = 0.5f * x;
        float y = x;
        long i = *(long*)&y;
        i = 0x5f3759df - (i>>1);
        y = *(float*)&i;
        y = y * (1.5f - (halfx * y * y));
        return y;
    }
    
    bool AHRS::begin() {
        xTaskCreatePinnedToCore(task_wrapper, "AHRS", 512, this, 1, NULL, 0);
        return true;
    }

    void AHRS::task() {
        while(true) {
            this->update();
            vTaskDelay(pdMS_TO_TICKS(this->dt * 1000));
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
            *q = eul2quat(rpy_);
            // q->normalize();

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
        
        m->normalize();

        float roll = atan2(a_.y(), a_.z());
        float pitch = asin(-a_.x() / sqrt(a_.x() * a_.x() + a_.y() * a_.y() + a_.z() * a_.z()));
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
            *q = eul2quat(rpy_);

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
        
        // m_.normalize();

        float roll = atan2(a_.y(), a_.z());
        float pitch = asin(-a_.x() / sqrt(a_.x() * a_.x() + a_.y() * a_.y() + a_.z() * a_.z()));
        rpy_xm = Eigen::Vector3f(
            roll,
            pitch,
            atan2(
                -m_.y() * cos(roll) + m_.z() * sin(roll), 
                m_.x() * cos(pitch) + m_.y() * sin(roll) * sin(pitch) + m_.z() * cos(roll) * sin(pitch)
            )
        );

        rpy_g = *rpy + g_ * dt;

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

        Eigen::Quaternion<float> q_w(
            0 - zeta * w_b.w(), 
            g_.x() - zeta * w_b.x(), 
            g_.y() - zeta * w_b.y(), 
            g_.z() - zeta * w_b.z()
        );
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

    void MadgwickOptimized::update() {
        // Initialize measurement variables
        Eigen::Vector3f tmp;
        float a_x, a_y, a_z, m_x, m_y, m_z, w_x, w_y, w_z;

        // Initialize intermediate variables
        float norm;                                                                     // vector norm
        float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4;           // quaternion rate from gyroscopes elements
        float f_1, f_2, f_3, f_4, f_5, f_6;                                             // objective function elements
        float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33,                       // objective function Jacobian elements
            J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64;     //
        float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4;                       // estimated direction of the gyroscope error
        float w_err_x, w_err_y, w_err_z;                                                // estimated direction of the gyroscope error (angular)
        float h_x, h_y, h_z;                                                            // computed flux in the earth frame

        // Get data
        if(xSemaphoreTake(*a_mutex, portMAX_DELAY)){
            tmp = *a;
            xSemaphoreGive(*a_mutex);

            a_x = tmp.x();
            a_y = tmp.y();
            a_z = tmp.z();
        }

        if(xSemaphoreTake(*m_mutex, portMAX_DELAY)){
            tmp = *m;
            xSemaphoreGive(*m_mutex);

            m_x = tmp.x();
            m_y = tmp.y();
            m_z = -tmp.z();
        }

        if(xSemaphoreTake(*g_mutex, portMAX_DELAY)){
            tmp = *g;
            xSemaphoreGive(*g_mutex);

            w_x = tmp.x();
            w_y = tmp.y();
            w_z = tmp.z();
        }


        // axulirary variables to avoid reapeated calcualtions
        float halfSEq_1 = 0.5f * SEq_1;
        float halfSEq_2 = 0.5f * SEq_2;
        float halfSEq_3 = 0.5f * SEq_3;
        float halfSEq_4 = 0.5f * SEq_4;
        float twoSEq_1 = 2.0f * SEq_1;
        float twoSEq_2 = 2.0f * SEq_2;
        float twoSEq_3 = 2.0f * SEq_3;
        float twoSEq_4 = 2.0f * SEq_4;
        float twob_x = 2.0f * b_x;
        float twob_z = 2.0f * b_z;
        float twob_xSEq_1 = 2.0f * b_x * SEq_1;
        float twob_xSEq_2 = 2.0f * b_x * SEq_2;
        float twob_xSEq_3 = 2.0f * b_x * SEq_3;
        float twob_xSEq_4 = 2.0f * b_x * SEq_4;
        float twob_zSEq_1 = 2.0f * b_z * SEq_1;
        float twob_zSEq_2 = 2.0f * b_z * SEq_2;
        float twob_zSEq_3 = 2.0f * b_z * SEq_3;
        float twob_zSEq_4 = 2.0f * b_z * SEq_4;
        float SEq_1SEq_2;
        float SEq_1SEq_3 = SEq_1 * SEq_3;
        float SEq_1SEq_4;
        float SEq_2SEq_3;
        float SEq_2SEq_4 = SEq_2 * SEq_4;
        float SEq_3SEq_4;
        float twom_x = 2.0f * m_x;
        float twom_y = 2.0f * m_y;
        float twom_z = 2.0f * m_z;

        // normalise the accelerometer measurement
        norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
        a_x /= norm;
        a_y /= norm;
        a_z /= norm;

        // normalise the magnetometer measurement
        norm = sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
        m_x /= norm;
        m_y /= norm;
        m_z /= norm;

        // compute the objective function and Jacobian
        f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
        f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
        f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
        f_4 = twob_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - m_x;
        f_5 = twob_x * (SEq_2 * SEq_3 - SEq_1 * SEq_4) + twob_z * (SEq_1 * SEq_2 + SEq_3 * SEq_4) - m_y;
        f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3) - m_z;
        J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
        J_12or23 = 2.0f * SEq_4;
        J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
        J_14or21 = twoSEq_2;
        J_32 = 2.0f * J_14or21; // negated in matrix multiplication
        J_33 = 2.0f * J_11or24; // negated in matrix multiplication
        J_41 = twob_zSEq_3; // negated in matrix multiplication
        J_42 = twob_zSEq_4;
        J_43 = 2.0f * twob_xSEq_3 + twob_zSEq_1; // negated in matrix multiplication
        J_44 = 2.0f * twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
        J_51 = twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
        J_52 = twob_xSEq_3 + twob_zSEq_1;
        J_53 = twob_xSEq_2 + twob_zSEq_4;
        J_54 = twob_xSEq_1 - twob_zSEq_3; // negated in matrix multiplication
        J_61 = twob_xSEq_3;
        J_62 = twob_xSEq_4 - 2.0f * twob_zSEq_2;
        J_63 = twob_xSEq_1 - 2.0f * twob_zSEq_3;
        J_64 = twob_xSEq_2;

        // compute the gradient (matrix multiplication)
        SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
        SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6;
        SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
        SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;

        // normalise the gradient to estimate direction of the gyroscope error
        norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
        SEqHatDot_1 = SEqHatDot_1 / norm;
        SEqHatDot_2 = SEqHatDot_2 / norm;
        SEqHatDot_3 = SEqHatDot_3 / norm;
        SEqHatDot_4 = SEqHatDot_4 / norm;

        // compute angular estimated direction of the gyroscope error
        w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
        w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
        w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;
        
        // compute and remove the gyroscope baises
        w_bx += w_err_x * dt * zeta;
        w_by += w_err_y * dt * zeta;
        w_bz += w_err_z * dt * zeta;
        w_x -= w_bx;
        w_y -= w_by;
        w_z -= w_bz;

        // compute the quaternion rate measured by gyroscopes
        SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
        SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
        SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
        SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;

        // compute then integrate the estimated quaternion rate
        SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * dt;
        SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * dt;
        SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * dt;
        SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * dt;

        // normalise quaternion
        norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
        SEq_1 /= norm;
        SEq_2 /= norm;
        SEq_3 /= norm;
        SEq_4 /= norm;

        // compute flux in the earth frame
        SEq_1SEq_2 = SEq_1 * SEq_2; // recompute axulirary variables
        SEq_1SEq_3 = SEq_1 * SEq_3;
        SEq_1SEq_4 = SEq_1 * SEq_4;
        SEq_3SEq_4 = SEq_3 * SEq_4;
        SEq_2SEq_3 = SEq_2 * SEq_3;
        SEq_2SEq_4 = SEq_2 * SEq_4;
        h_x = twom_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3);
        h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5f - SEq_2 * SEq_2 - SEq_4 * SEq_4) + twom_z * (SEq_3SEq_4 - SEq_1SEq_2);
        h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2) + twom_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3);
        
        // normalise the flux vector to have only components in the x and z
        b_x = sqrt((h_x * h_x) + (h_y * h_y));
        b_z = h_z;

        if(xSemaphoreTake(*fused_mutex, 0)){
            *q = Eigen::Quaternionf(SEq_1, SEq_2, SEq_3, SEq_4);
            q->normalize();

            *rpy = q->toRotationMatrix().eulerAngles(0, 1, 2);

            xSemaphoreGive(*fused_mutex);
        }
    }
}
