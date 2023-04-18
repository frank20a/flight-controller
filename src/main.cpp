#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "imu.h"
#include "datatypes.h"

// Mutexes
SemaphoreHandle_t spi2_mutex = xSemaphoreCreateMutex();
SemaphoreHandle_t uart0_mutex = xSemaphoreCreateMutex();
SemaphoreHandle_t uart1_mutex = xSemaphoreCreateMutex();
SemaphoreHandle_t uart2_mutex = xSemaphoreCreateMutex();
SemaphoreHandle_t imu_mutex = xSemaphoreCreateMutex();

// Shared memory
Vector3 imu_shared;

// Sensors
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(SCLK2, MISO2, MOSI2, CS_XMAG, CS_GYRO, 1000);

// Task Handles
TaskHandle_t xmag_task_handle;
TaskHandle_t gyro_task_handle;

void setup() {
  // Start debug serial
  Serial.begin(115200);
  
  // Start sensors
  if (!lsm.begin()) {
      Serial.println("Failed to communicate with LSM9DS0.");
      while(1);
  }
  Serial.println("LSM9DS0 online!");
  delay(1000);

  // Start tasks
  IMU::imu_task_parameters imu_params = {
      .lsm = &lsm,
      .spi_mutex = &spi2_mutex,
      .imu_mutex = &imu_mutex,
      .shared_measurement = &imu_shared
  };
  xTaskCreate(IMU::task, "IMU", 10000, &imu_params, 1, NULL);
}

void loop() {
}