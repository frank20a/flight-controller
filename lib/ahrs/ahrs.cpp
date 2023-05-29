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
    Eigen::Matrix3f Aa {
        { 0.973209,  0.006248, -0.000107},
        { 0.006248,  0.959535,  0.000811},
        {-0.000107,  0.000811,  1.019086}
    };
    Eigen::Vector3f ba {-0.611425, 0.195279, -0.229477};

    Eigen::Matrix3f Am {
        { 1.024384, -0.067449,  0.005591},
        {-0.067449,  0.984567, -0.059303},
        { 0.005591, -0.059303,  1.093657}
    };
    Eigen::Vector3f bm {-15.850987, 46.848109, -15.884498};

    Eigen::Vector3f bg {0.08806329216688561, 0.011335891374165658, 0.16633187215725581};

    float invSqrt(float x) {
        float halfx = 0.5f * x;
        float y = x;
        long i = *(long*)&y;
        i = 0x5f3759df - (i>>1);
        y = *(float*)&i;
        y = y * (1.5f - (halfx * y * y));
        return y;
    }
    
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

    void AHRS::start_task() {
        xTaskCreate(task_wrapper, "AHRS", 2048, this, 1, NULL);
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
        float ax, ay, az, mx, my, mz, gx, gy, gz;

        // Initialize intermediate variables
        float recipNorm;
        float s0, s1, s2, s3;
        float qDot1, qDot2, qDot3, qDot4;
        float hx, hy;
        float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;


        // Get data
        if(xSemaphoreTake(*a_mutex, portMAX_DELAY)){
            tmp = *a;
            xSemaphoreGive(*a_mutex);

            ax = tmp.x();
            ay = tmp.y();
            az = tmp.z();
        }

        if(xSemaphoreTake(*m_mutex, portMAX_DELAY)){
            tmp = *m;
            xSemaphoreGive(*m_mutex);

            mx = tmp.x();
            my = tmp.y();
            mz = tmp.z();
        }

        if(xSemaphoreTake(*g_mutex, portMAX_DELAY)){
            tmp = *g;
            xSemaphoreGive(*g_mutex);

            gx = tmp.x();
            gy = tmp.y();
            gz = tmp.z();
        }


        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

            // Normalise accelerometer measurement
            recipNorm = invSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;   

            // Normalise magnetometer measurement
            recipNorm = invSqrt(mx * mx + my * my + mz * mz);
            mx *= recipNorm;
            my *= recipNorm;
            mz *= recipNorm;

            // Auxiliary variables to avoid repeated arithmetic
            _2q0mx = 2.0f * q0 * mx;
            _2q0my = 2.0f * q0 * my;
            _2q0mz = 2.0f * q0 * mz;
            _2q1mx = 2.0f * q1 * mx;
            _2q0 = 2.0f * q0;
            _2q1 = 2.0f * q1;
            _2q2 = 2.0f * q2;
            _2q3 = 2.0f * q3;
            _2q0q2 = 2.0f * q0 * q2;
            _2q2q3 = 2.0f * q2 * q3;
            q0q0 = q0 * q0;
            q0q1 = q0 * q1;
            q0q2 = q0 * q2;
            q0q3 = q0 * q3;
            q1q1 = q1 * q1;
            q1q2 = q1 * q2;
            q1q3 = q1 * q3;
            q2q2 = q2 * q2;
            q2q3 = q2 * q3;
            q3q3 = q3 * q3;

            // Reference direction of Earth's magnetic field
            hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
            hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
            _2bx = sqrt(hx * hx + hy * hy);
            _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;

            // Gradient decent algorithm corrective step
            s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            // Apply feedback step
            qDot1 -= beta * s0;
            qDot2 -= beta * s1;
            qDot3 -= beta * s2;
            qDot4 -= beta * s3;
        }

        // Integrate rate of change of quaternion to yield quaternion
        q0 += qDot1 * dt;
        q1 += qDot2 * dt;
        q2 += qDot3 * dt;
        q3 += qDot4 * dt;

        // Normalise quaternion
        recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;

        

        if(xSemaphoreTake(*fused_mutex, 0)){
            *q = Eigen::Quaternionf(q0, q1, q2, q3);
            q->normalize();

            *rpy = q->toRotationMatrix().eulerAngles(0, 1, 2);

            xSemaphoreGive(*fused_mutex);
        }
    }
}
