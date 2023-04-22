#include "main.h"

void setup() {
  // ================= Start Comms =================
  Serial.begin(115200);                       // Start UART0
  SPI2.begin(SCLK2, MISO2, MOSI2, CS_XMAG);   // Start SPI2
  
  // ================= Start sensors =================
  // ==== IMU ====
  if (!lsm.begin()) {
      Serial.println("Failed to communicate with LSM9DS0.");
      while(1);
  }
  Serial.println("LSM9DS0 online!");

  // ================= Start tasks =================
  delay(1000);
  // ==== IMU ====
  params_a = {
    .lsm = &lsm,
    .spi_mutex = &spi2_mutex,
    .data_mutex = &acc_raw_mutex,
    .shared_data = &acc_raw,
    .rate = 400,
    .type = IMU_SENSORTYPE_ACCEL,
  };
  xTaskCreate(AHRS::measure_task, "ACCEL", 1024, &params_a, 1, NULL);
  params_m = {
    .lsm = &lsm,
    .spi_mutex = &spi2_mutex,
    .data_mutex = &mag_raw_mutex,
    .shared_data = &mag_raw,
    .rate = 100,
    .type = IMU_SENSORTYPE_MAG,
  };
  xTaskCreate(AHRS::measure_task, "MAG", 10000, &params_m, 1, NULL);
  params_g = {
    .lsm = &lsm,
    .spi_mutex = &spi2_mutex,
    .data_mutex = &gyro_raw_mutex,
    .shared_data = &gyro_raw,
    .rate = 190,
    .type = IMU_SENSORTYPE_GYRO,
  };
  xTaskCreate(AHRS::measure_task, "GYRO", 10000, &params_g, 1, NULL);
  // ==== RC ====
  // ==== AHRS ====
  // ==== Control ====
  // ==== GPS ====
}

void loop() {
  // if(xSemaphoreTake(uart0_mutex, portMAX_DELAY)){
  //   if(xSemaphoreTake(acc_raw_mutex, portMAX_DELAY)){
  //     Serial.write((byte *)&acc_raw, sizeof(gyro_raw));
  //     xSemaphoreGive(acc_raw_mutex);
  //   }
  //   if(xSemaphoreTake(mag_raw_mutex, portMAX_DELAY)){
  //     Serial.write((byte *)&mag_raw, sizeof(gyro_raw));
  //     xSemaphoreGive(mag_raw_mutex);
  //   }
  //   if(xSemaphoreTake(gyro_raw_mutex, portMAX_DELAY)){
  //     Serial.write((byte *)&gyro_raw, sizeof(gyro_raw));
  //     xSemaphoreGive(gyro_raw_mutex);
  //   }
    
  //   xSemaphoreGive(uart0_mutex);
  // }
  // delay(10);
}