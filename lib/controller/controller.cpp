#include "controller.h"

namespace Controller {
    float mapf(float x, float a, float b, float A, float B) {
        return (x - a) * (B - A) / (b - a) + A;
    }

    float bitbang(float x, float a) {
        if (x > a || x < -a)
            return a;
        return 0;
    }

    float clip(float x, float a, float b) {
        if (x > b)
            return b;
        if (x < a)
            return a;
        return x;
    }

    float clip(float x, float a) {
        return clip(x, -a, a);
    }

    float PID::update(float error) {
        integral += error * dt;
        derivative = (error - prev_error) / dt;
        prev_error = error;
        return kp * error + ki * integral + kd * derivative;
    }

    void PID::reset() {
        integral = 0;
        derivative = 0;
        prev_error = 0;
    }
    
    bool MotorController::begin() {
        for (int i = 0; i < 4; i++)
            pids[i].reset();

        pinMode(ESC1, OUTPUT);
        pinMode(ESC2, OUTPUT);
        pinMode(ESC3, OUTPUT);
        pinMode(ESC4, OUTPUT);

        if (motors.attach(ESC1, 1000, 2000) > 15) {
            Serial.println("Failed to attach ESC1");
            return false;
        }
        if (motors.attach(ESC2, 1000, 2000) > 15) {
            Serial.println("Failed to attach ESC2");
            return false;
        }
        if (motors.attach(ESC3, 1000, 2000) > 15) {
            Serial.println("Failed to attach ESC3");
            return false;
        }
        if (motors.attach(ESC4, 1000, 2000) > 15) {
            Serial.println("Failed to attach ESC4");
            return false;
        }

        #ifdef CALIBRATE_ESC
        calibrate_esc();
        #endif

        for(int i = 0; i < 4; i++)
            motors.write(pins[i], 1000);
        

        xTaskCreatePinnedToCore(task_wrapper, "AHRS", 2048, this, 1, NULL, 1);
        return true;
    }

    void MotorController::calibrate_esc() {
        Serial.println("Calibrating ESCs. Start by disconnecting the battery.");
        Serial.println("Send any character to continue...");
        
        while(!Serial.available());
        while(Serial.available()) Serial.read();

        for(int i = 0; i < 4; i++)
            motors.write(pins[i], 2000);
        Serial.println("Connect the battery now. You will hear two beeps.");
        Serial.println("Wait a bit and send any character to continue...");
        
        while(!Serial.available());
        while(Serial.available()) Serial.read();

        for(int i = 0; i < 4; i++)
            motors.write(pins[i], 1000);

        Serial.println("Wait a little bit, you will hear two more beeps.");
        Serial.println("Wait a bit more and send any character to continue...");

        while(!Serial.available());
        while(Serial.available()) Serial.read();

        Serial.println("Now the ESCs are calibrated. Restart/Reflash the board and run normally.");
        while(true);
    }

    void MotorController::task() {
        while (true) {
            this->update();
            vTaskDelay(pdMS_TO_TICKS(1000 * this->dt));
        }
    }

    void MotorController::update() {
        Eigen::Vector3f state;
        float yaw_rate, motor_lvl[4];
        ControllerLevels controller;

        // Get Yaw Rate
        if(xSemaphoreTake(attr->gyr_raw_mutex, portMAX_DELAY)) {
            yaw_rate = attr->gyr_raw.z();
            xSemaphoreGive(attr->gyr_raw_mutex);
        } else return;
        
        // Get State
        if(xSemaphoreTake(attr->imu_fused_mutex, portMAX_DELAY)) {
            state = attr->rpy_fused;
            xSemaphoreGive(attr->imu_fused_mutex);
        } else return;

        // Get Controller Level
        if (xSemaphoreTake(attr->controller_level_mutex, portMAX_DELAY)) {
            controller = attr->controller_level;
            xSemaphoreGive(attr->controller_level_mutex);
        } else return;

        // Update PIDs
        float t = mapf(controller.ch00, 0, 1023, 0, MAX_THROTTLE);
        float r = pids[1].update(mapf(controller.ch01, 0, 1023, -MAX_CRAFT_ANGLE, MAX_CRAFT_ANGLE) - state.x());
        float p = pids[2].update(mapf(controller.ch02, 0, 1023, -MAX_CRAFT_ANGLE, MAX_CRAFT_ANGLE) - state.y());
        float y = pids[3].update(mapf(controller.ch03, 0, 1023, -MAX_CRAFT_YAW_RATE, MAX_CRAFT_YAW_RATE) - bitbang(yaw_rate, 0.02));

        // Motor Mixing Algorithm
        if (t > MIN_THROTTLE) {
            motor_lvl[0] = clip(t + r + p + y, 0, 1);
            motor_lvl[1] = clip(t - r + p - y, 0, 1);
            motor_lvl[2] = clip(t + r - p - y, 0, 1);
            motor_lvl[3] = clip(t - r - p + y, 0, 1);
        } else {
            motor_lvl[0] = t;
            motor_lvl[1] = t;
            motor_lvl[2] = t;
            motor_lvl[3] = t;
        }

        // Save motor level
        if (xSemaphoreTake(attr->motor_level_mutex, portMAX_DELAY)) {
            memcpy(attr->motor_level, motor_lvl, 4*sizeof(float));
            xSemaphoreGive(attr->motor_level_mutex);
        } else return;

        // Set Motor Level
        for (int i = 0; i < 4; i++) {
            motors.write(pins[i], 1000 + 1000 * motor_lvl[i]);
        }

    }
};