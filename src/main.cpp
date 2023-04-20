#include "main.h"

void setup() {
  // Release mutexes
  
  // Start debug serial
  Serial.begin(115200);

  // Start SPI
  SPI2.begin(SCLK2, MISO2, MOSI2, CS_XMAG);
  
  // Start sensors
  if (!lsm.begin()) {
      Serial.println("Failed to communicate with LSM9DS0.");
      while(1);
  }
  Serial.println("LSM9DS0 online!");
  delay(1000);

  // Start tasks
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
}

void loop() {
  if(xSemaphoreTake(acc_raw_mutex, portMAX_DELAY)){
    Serial.print("Accel: ");
    Serial.print(acc_raw.x);
    Serial.print(", ");
    Serial.print(acc_raw.y);
    Serial.print(", ");
    Serial.println(acc_raw.z);
    xSemaphoreGive(acc_raw_mutex);
  }
  if(xSemaphoreTake(mag_raw_mutex, portMAX_DELAY)){
    Serial.print("Mag: ");
    Serial.print(mag_raw.x);
    Serial.print(", ");
    Serial.print(mag_raw.y);
    Serial.print(", ");
    Serial.println(mag_raw.z);
    xSemaphoreGive(mag_raw_mutex);
  }
  if(xSemaphoreTake(gyro_raw_mutex, portMAX_DELAY)){
    Serial.print("Gyro: ");
    Serial.print(gyro_raw.x);
    Serial.print(", ");
    Serial.print(gyro_raw.y);
    Serial.print(", ");
    Serial.println(gyro_raw.z);
    xSemaphoreGive(gyro_raw_mutex);
  }
  delay(100);
}