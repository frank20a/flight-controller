#include "main.h"

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
    delay(1000);
    // ==== IMU ====
    AHRS::calibrate_gyro(spi2_mutex, lsm);
    xTaskCreate(AHRS::measure_task, "ACCEL", 1024, &params_a, 1, NULL);
    xTaskCreate(AHRS::measure_task, "MAG", 1024, &params_m, 1, NULL);
    xTaskCreate(AHRS::measure_task, "GYRO", 1024, &params_g, 1, NULL);
    // ==== AHRS ====
    ahrs_filter->start_task();
    // ==== RC ====
    // ==== Control ====
    // ==== GPS ====
}

void loop()
{
    // write_imu_raw();
    write_imu_fused();
    // print_imu_fused();

    delay(10);
}

void write_imu_fused()
{
    if (xSemaphoreTake(uart0_mutex, portMAX_DELAY))
    {

        if (xSemaphoreTake(imu_fused_mutex, portMAX_DELAY))
        {
            Serial.write((byte *)&imu_fused, sizeof(imu_fused));
            xSemaphoreGive(imu_fused_mutex);
        }

        xSemaphoreGive(uart0_mutex);
    }
}

void print_imu_fused()
{
    if (xSemaphoreTake(uart0_mutex, portMAX_DELAY))
    {
        if (xSemaphoreTake(imu_fused_mutex, portMAX_DELAY))
        {
            Serial.print("Fused:\n\tRoll:  ");
            Serial.print(rpy_fused.x());
            Serial.print("\n\tPitch: ");
            Serial.print(rpy_fused.y());
            Serial.print("\n\tYaw:   ");
            Serial.println(rpy_fused.z());
            xSemaphoreGive(imu_fused_mutex);
        }

        xSemaphoreGive(uart0_mutex);
    }
}

void write_imu_raw()
{
    if (xSemaphoreTake(uart0_mutex, portMAX_DELAY))
    {

        if (xSemaphoreTake(acc_raw_mutex, portMAX_DELAY))
        {
            Serial.write((byte *)&acc_raw, sizeof(acc_raw));
            xSemaphoreGive(acc_raw_mutex);
        }
        if (xSemaphoreTake(mag_raw_mutex, portMAX_DELAY))
        {
            Serial.write((byte *)&mag_raw, sizeof(mag_raw));
            xSemaphoreGive(mag_raw_mutex);
        }
        if (xSemaphoreTake(gyro_raw_mutex, portMAX_DELAY))
        {
            Serial.write((byte *)&gyro_raw, sizeof(gyro_raw));
            xSemaphoreGive(gyro_raw_mutex);
        }

        xSemaphoreGive(uart0_mutex);
    }
}

void print_imu_raw()
{
    if (xSemaphoreTake(acc_raw_mutex, portMAX_DELAY))
    {
        Serial.print("Acc: ");
        Serial.print(acc_raw.x());
        Serial.print(", ");
        Serial.print(acc_raw.y());
        Serial.print(", ");
        Serial.println(acc_raw.z());
        xSemaphoreGive(acc_raw_mutex);
    }
    if (xSemaphoreTake(mag_raw_mutex, portMAX_DELAY))
    {
        Serial.print("Mag: ");
        Serial.print(mag_raw.x());
        Serial.print(", ");
        Serial.print(mag_raw.y());
        Serial.print(", ");
        Serial.println(mag_raw.z());
        xSemaphoreGive(mag_raw_mutex);
    }
    if (xSemaphoreTake(gyro_raw_mutex, portMAX_DELAY))
    {
        Serial.print("Gyro: ");
        Serial.print(gyro_raw.x());
        Serial.print(", ");
        Serial.print(gyro_raw.y());
        Serial.print(", ");
        Serial.println(gyro_raw.z());
        xSemaphoreGive(gyro_raw_mutex);
    }
}
