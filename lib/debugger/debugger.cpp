#include "debugger.h"

bool Debugger::begin() {
    xTaskCreatePinnedToCore(task_wrapper, "debugger", 2048, this, 1, NULL, 0);
    return true;
}

void Debugger::config(byte flags, unsigned short dt_) {
    debug_io_mode = flags & 0b00000001;
    debug_type = (flags & 0b00001110) >> 1;
    dt = dt_;
}

void Debugger::debug_msg() {
    while(true){
        if(debug_type == 0b000) imu_debugger();
        else if(debug_type == 0b001) ahrs_debugger();
        else if(debug_type == 0b010) ;
        else if(debug_type == 0b011) ;
        else if(debug_type == 0b100) ;
        else if(debug_type == 0b101) ;
        else if(debug_type == 0b110) ;
        else if(debug_type == 0b111) ;

        vTaskDelay(pdMS_TO_TICKS(dt));
    }
}

void Debugger::ahrs_debugger() {
    if (xSemaphoreTake(attr->uart0_mutex, portMAX_DELAY))
    {

        if (xSemaphoreTake(attr->imu_fused_mutex, portMAX_DELAY))
        {   
            if(!debug_io_mode)
                Serial.write((byte *)&(attr->imu_fused), sizeof(attr->imu_fused));
            else {
                Serial.print("Fused:\n\tRoll:  ");
                Serial.print((attr->rpy_fused).x());
                Serial.print("\n\tPitch: ");
                Serial.print((attr->rpy_fused).y());
                Serial.print("\n\tYaw:   ");
                Serial.println((attr->rpy_fused).z());
            }
            xSemaphoreGive(attr->imu_fused_mutex);
        }

        xSemaphoreGive(attr->uart0_mutex);
    }
}

void Debugger::imu_debugger() {
    if (xSemaphoreTake(attr->uart0_mutex, portMAX_DELAY)) {
        if (xSemaphoreTake(attr->acc_raw_mutex, portMAX_DELAY)) {
            if (!debug_io_mode)
                Serial.write((byte *)&(attr->acc_raw), sizeof(attr->acc_raw));
            else {
                Serial.print("Acc: ");
                Serial.print((attr->acc_raw).x());
                Serial.print(", ");
                Serial.print((attr->acc_raw).y());
                Serial.print(", ");
                Serial.println((attr->acc_raw).z());
            }
            xSemaphoreGive(attr->acc_raw_mutex);
        }
        if (xSemaphoreTake(attr->mag_raw_mutex, portMAX_DELAY)) {
            if (!debug_io_mode)
                Serial.write((byte *)&(attr->mag_raw), sizeof(attr->mag_raw));
            else {
                Serial.print("Mag: ");
                Serial.print((attr->mag_raw).x());
                Serial.print(", ");
                Serial.print((attr->mag_raw).y());
                Serial.print(", ");
                Serial.println((attr->mag_raw).z());
            }
            xSemaphoreGive(attr->mag_raw_mutex);
        }
        if (xSemaphoreTake(attr->gyr_raw_mutex, portMAX_DELAY)) {
            if (!debug_io_mode)
                Serial.write((byte *)&(attr->gyr_raw), sizeof(attr->gyr_raw));
            else {
                Serial.print("Gyro: ");
                Serial.print((attr->gyr_raw).x());
                Serial.print(", ");
                Serial.print((attr->gyr_raw).y());
                Serial.print(", ");
                Serial.println((attr->gyr_raw).z());
            }
            xSemaphoreGive(attr->gyr_raw_mutex);
        }

        xSemaphoreGive(attr->uart0_mutex);
    }
}