#include "lsm9ds0.h"

#include "esp_log.h"

namespace LSM9DS0 {

    LSM9DS0::LSM9DS0() {
        // Add Accel/Mag and Gyro devices to SPI bus
        ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &xm_config, &xm_handle));
        ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &g_config, &g_handle));

        // Check Device IDs
        ESP_LOGI("LSM9DS0", "XM ID: %d, G ID: %d", get_xm_id(), get_g_id());
        if (get_xm_id() != 73 || get_g_id() != 212){
            ESP_LOGI("LSM9DS0", "Device wrong IDs");
            return;
        }

        // Setup Accel/Mag
        uint8_t tx_buf_xm[8] = {
            0b00000000,     // CTRL_REG0_XM - DEFAULT
            0b10000111,     // CTRL_REG1_XM - 400Hz, Continuous, AZen, ZYez, AXen Enabled
            0b00011000,     // CTRL_REG2_XM - Accel scale 8G
            0b00000000,     // CTRL_REG3_XM - DEFAULT
            0b00000000,     // CTRL_REG4_XM - DEFAULT
            0b11110100,     // CTRL_REG5_XM - Enable Temp, High resolution Mag, 100Hz Mag, Interrupts disabled
            0b00000000,     // CTRL_REG6_XM - Mag scale 2Gauss
            0b00000000,     // CTRL_REG7_XM - Mag continuous conversion mode
        };
        spi_transaction_t trans_xm = {
            .cmd = 0b01,        // Auto increment address
            .addr = 0b011111, // Start at CTRL_REG0_XM
            .length = 8 * 8,
            .rxlength = 0,
            .tx_buffer = tx_buf_xm,
            .rx_buffer = NULL,
        };
        ESP_ERROR_CHECK(spi_device_transmit(xm_handle, &trans_xm));

        // Setup Gyro
        uint8_t tx_buf_g[5] = {
            0b10001111,     // CTRL_REG1_G - ODR: 380Hz, Cutoff: 20, PD: Normal, Zen, Zen, Yen Enabled
            0b00000000,     // CTRL_REG2_G - DEFAULT
            0b00000000,     // CTRL_REG3_G - DEFAULT
            0b00010000,     // CTRL_REG4_G - Gyro scale 500dps
            0b00000000,     // CTRL_REG5_G - DEFAULT
        };
        spi_transaction_t trans_g = {
            .cmd = 0b01,        // Auto increment address
            .addr = 0b100000,   // Start at CTRL_REG1_G
            .length = 8*5,
            .rxlength = 0,
            .tx_buffer = tx_buf_g,
            .rx_buffer = NULL,
        };
        ESP_ERROR_CHECK(spi_device_transmit(g_handle, &trans_g));

        initialized = true;
        ESP_LOGI("LSM9DS0", "Setup complete");
    }

    LSM9DS0::~LSM9DS0() {
        // Remove Accel/Mag and Gyro devices from SPI bus
        spi_bus_remove_device(xm_handle);
        spi_bus_remove_device(g_handle);
    }

    uint8_t LSM9DS0::get_xm_id() {
        uint8_t rx_buf;
        spi_transaction_t trans = {
            .cmd = 0b10,
            .addr = 0b001111,
            .length = 8,
            .rxlength = 8,
            .rx_buffer = &rx_buf,
        };
        ESP_ERROR_CHECK(spi_device_transmit(xm_handle, &trans));
        
        return rx_buf;
    }

    uint8_t LSM9DS0::get_g_id() {
        uint8_t rx_buf;
        spi_transaction_t trans = {
            .cmd = 0b10,
            .addr = 0b001111,
            .length = 8,
            .rxlength = 8,
            .rx_buffer = &rx_buf,
        };
        ESP_ERROR_CHECK(spi_device_transmit(g_handle, &trans));
        
        return rx_buf;
    }

    Vector3 LSM9DS0::get_accel(){
        uint8_t rx_buf[6] = {0};
        spi_transaction_t trans = {
            .cmd = 0b11,
            .addr = 0b101000,
            .length = 8 * 6,
            .rxlength = 8 * 6,
            .rx_buffer = rx_buf,
        };
        ESP_ERROR_CHECK(spi_device_transmit(xm_handle, &trans));
        
        return convert_buffer_to_vector3(rx_buf, 0.244 / 1000);
    }

    Vector3 LSM9DS0::get_mag(){
        uint8_t rx_buf[6] = {0};
        spi_transaction_t trans = {
            .cmd = 0b11,
            .addr = 0b001000,
            .length = 8 * 6,
            .rxlength = 8 * 6,
            .rx_buffer = rx_buf,
        };
        ESP_ERROR_CHECK(spi_device_transmit(xm_handle, &trans));

        return convert_buffer_to_vector3(rx_buf, 0.08 / 10);
    }

    Vector3 LSM9DS0::get_gyro(){
        uint8_t rx_buf[6] = {0};
        spi_transaction_t trans = {
            .cmd = 0b11,
            .addr = 0b101000,
            .length = 8 * 6,
            .rxlength = 8 * 6,
            .tx_buffer = NULL,
            .rx_buffer = rx_buf,
        };
        ESP_ERROR_CHECK(spi_device_transmit(g_handle, &trans));

        ESP_LOGI("DEBUG", "Gyro: %3d, %3d, %3d, %3d, %3d, %3d, %3d", rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4], rx_buf[5], rx_buf[6]);

        return convert_buffer_to_vector3(rx_buf, 0.0175 * 0.01745329);
    }

    float LSM9DS0::get_temp(){
        uint8_t rx_buf[1] = {0};
        spi_transaction_t trans = {
            .cmd = 0b10,
            .addr = 0b000101,
            .length = 8 * 1,
            .rxlength = 8 * 1,
            .rx_buffer = rx_buf,
        };
        ESP_ERROR_CHECK(spi_device_transmit(xm_handle, &trans));

        uint8_t xlo = rx_buf[0];
        uint16_t xhi = rx_buf[1];

        xhi <<= 8;
        xhi |= xlo;

        return 21.0f + (int)xhi / 8.0f;
    }

    Vector3 LSM9DS0::convert_buffer_to_vector3(uint8_t* buffer, float scale){
        uint8_t xlo;
        uint16_t xhi;
        Vector3 res;

        xlo = buffer[0];
        xhi = buffer[1];
        xhi <<= 8;
        xhi |= xlo;
        res.x = (float)xhi * scale;

        xlo = buffer[2];
        xhi = buffer[3];
        xhi <<= 8;
        xhi |= xlo;
        res.y = (float)xhi * scale;

        xlo = buffer[4];
        xhi = buffer[5];
        xhi <<= 8;
        xhi |= xlo;
        res.z = (float)xhi * scale;

        return res;
    }

}