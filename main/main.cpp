#include "main.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "lsm9ds0.h"
#include "structs.h"

#define MOSI2 CONFIG_MOSI2
#define MISO2 CONFIG_MISO2
#define SCLK2 CONFIG_SCLK2
#define LED (gpio_num_t)CONFIG_RGB


Main::Main(void) { 
    // Configure LED
    gpio_reset_pin(LED);
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);
}

void Main::run() {
    gpio_set_level(LED, 1);
    // ESP_LOGI("main", "LED ON");
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    gpio_set_level(LED, 0);
    // ESP_LOGI("main", "LED OFF");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

extern "C" void app_main(void)
{   
    // Create Main class
    Main main;


    // Confgure SPI2
    spi_bus_config_t buscfg = {
        .mosi_io_num = MOSI2,
        .miso_io_num = MISO2,
        .sclk_io_num = SCLK2,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);

    // Create Peripherals
    LSM9DS0::LSM9DS0 imu;
    
    while(true) {
        Vector3 accel = imu.get_accel();
        Vector3 mag = imu.get_mag();
        Vector3 gyro = imu.get_gyro();
        ESP_LOGI("main", "Temp: %f", imu.get_temp());
        ESP_LOGI("main", "Accel: %f, %f, %f", accel.x, accel.y, accel.z);
        ESP_LOGI("main", "Mag: %f, %f, %f", mag.x, mag.y, mag.z);
        ESP_LOGI("main", "Gyro: %f, %f, %f", gyro.x, gyro.y, gyro.z);
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}
