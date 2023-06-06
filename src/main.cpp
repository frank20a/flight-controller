#include "main.h"
#ifdef DEBUG
    #include "debuggers.h"
#endif

void setup()
{
    // ================= Start Comms =================
    Serial.begin(UART0_SPEED);                // Start UART0
    SPI2.begin(SCLK2, MISO2, MOSI2, CS_XMAG); // Start SPI2

    // ================= Start sensors =================
    // ==== IMU ====
    if (!lsm.begin())
    {
        Serial.println("Failed to communicate with LSM9DS0.");
        while (1)
            ;
    }
    Serial.println("LSM9DS0 online!");

    // ================= Start tasks =================
    // ==== AHRS ====
    ahrs_filter->begin();
    // ==== RC ====
    // ==== Control ====
    // ==== GPS ====
}

void loop() {
    #ifdef DEBUG
    DEBUGGER::debug_msg();
    #endif

    delay(65);
}

#ifdef DEBUG
namespace DEBUGGER {

    void ahrs_debugger(bool io_mode) {
        if (xSemaphoreTake(uart0_mutex, portMAX_DELAY))
        {

            if (xSemaphoreTake(imu_fused_mutex, portMAX_DELAY))
            {   
                if(!io_mode)
                    Serial.write((byte *)&imu_fused, sizeof(imu_fused));
                else {
                    Serial.print("Fused:\n\tRoll:  ");
                    Serial.print(rpy_fused.x());
                    Serial.print("\n\tPitch: ");
                    Serial.print(rpy_fused.y());
                    Serial.print("\n\tYaw:   ");
                    Serial.println(rpy_fused.z());
                }
                xSemaphoreGive(imu_fused_mutex);
            }

            xSemaphoreGive(uart0_mutex);
        }
    }
    
    void imu_debugger(bool io_mode) {
        if (xSemaphoreTake(uart0_mutex, portMAX_DELAY)) {
            if (xSemaphoreTake(acc_raw_mutex, portMAX_DELAY)) {
                if (!io_mode)
                    Serial.write((byte *)&acc_raw, sizeof(acc_raw));
                else {
                    Serial.print("Acc: ");
                    Serial.print(acc_raw.x());
                    Serial.print(", ");
                    Serial.print(acc_raw.y());
                    Serial.print(", ");
                    Serial.println(acc_raw.z());
                }
                xSemaphoreGive(acc_raw_mutex);
            }
            if (xSemaphoreTake(mag_raw_mutex, portMAX_DELAY)) {
                if (!io_mode)
                    Serial.write((byte *)&mag_raw, sizeof(mag_raw));
                else {
                    Serial.print("Mag: ");
                    Serial.print(mag_raw.x());
                    Serial.print(", ");
                    Serial.print(mag_raw.y());
                    Serial.print(", ");
                    Serial.println(mag_raw.z());
                }
                xSemaphoreGive(mag_raw_mutex);
            }
            if (xSemaphoreTake(gyro_raw_mutex, portMAX_DELAY)) {
                if (!io_mode)
                    Serial.write((byte *)&gyro_raw, sizeof(gyro_raw));
                else {
                    Serial.print("Gyro: ");
                    Serial.print(gyro_raw.x());
                    Serial.print(", ");
                    Serial.print(gyro_raw.y());
                    Serial.print(", ");
                    Serial.println(gyro_raw.z());
                }
                xSemaphoreGive(gyro_raw_mutex);
            }

            xSemaphoreGive(uart0_mutex);
        }
    }

}
#endif