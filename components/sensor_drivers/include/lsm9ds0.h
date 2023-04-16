#pragma once

#include "driver/spi_master.h"
#include "driver/spi_common.h"
#include "sdkconfig.h"

#include "structs.h"

#define CS_XMAG CONFIG_CS_XMAG
#define CS_GYRO CONFIG_CS_GYRO

namespace LSM9DS0 {

    class LSM9DS0 {
    public:
        LSM9DS0();
        ~LSM9DS0();

        uint8_t get_xm_id();
        uint8_t get_g_id();

        Vector3 get_accel();
        Vector3 get_mag();
        Vector3 get_gyro(); 
        float get_temp();

        bool initialized = false;

    private:
        Vector3 convert_buffer_to_vector3(uint8_t *buffer, float scale);

        spi_device_interface_config_t xm_config = {
            .command_bits = 2,
            .address_bits = 6,
            .dummy_bits = 0,
            .mode = 0,
            .clock_speed_hz = 500000,
            .spics_io_num = CS_XMAG,
            .queue_size = 1,
        };
        spi_device_interface_config_t g_config = {
            .command_bits = 2,
            .address_bits = 6,
            .dummy_bits = 0,
            .mode = 0,
            .clock_speed_hz = 500000,
            .spics_io_num = CS_GYRO,
            .queue_size = 1,
        };
        spi_device_handle_t xm_handle, g_handle;
    };
}