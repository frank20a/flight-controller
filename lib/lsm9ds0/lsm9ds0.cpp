#include "lsm9ds0.h"

namespace LSM9DS0 {
    LSM9DS0::LSM9DS0(SPIClass *spi_) {
        spi = spi_;
    }

    bool LSM9DS0::begin(){
        // Set the chip select pins as outputs
        pinMode(CS_GYRO, OUTPUT);
        pinMode(CS_XMAG, OUTPUT);
        digitalWrite(CS_GYRO, HIGH);
        digitalWrite(CS_XMAG, HIGH);

        // Check if the device is connected and on the right bus
        if (read(GYROTYPE, REGISTER_WHO_AM_I_G) != GYRO_ID && read(XMTYPE, REGISTER_WHO_AM_I_XM) != XMAG_ID)
            return false;

        byte buf;

        // Enable accelerometer continuous
        write(XMTYPE, REGISTER_CTRL_REG1_XM,  0b00001111 | ACCELDATARATE_400HZ);

        // Enable the magnetometer continuous & temperature sensor
        write(XMTYPE, REGISTER_CTRL_REG5_XM, 0b11100000 | MAGDATARATE_100HZ);

        // Enable the magnetometer continuous
        write(XMTYPE, REGISTER_CTRL_REG7_XM, 0);

        // Enable gyro continuous
        write(GYROTYPE, REGISTER_CTRL_REG1_G, 0x0F | GYRODATARATE_190HZ);

        // Set sensor full scale
        write(XMTYPE, REGISTER_CTRL_REG2_XM, ACCELRANGE_8G);
        a_scale = ACCEL_MG_LSB_8G;
        write(XMTYPE, REGISTER_CTRL_REG6_XM, MAGGAIN_2GAUSS);
        m_scale = MAG_MGAUSS_2GAUSS;
        write(GYROTYPE, REGISTER_CTRL_REG4_G, GYROSCALE_500DPS);
        g_scale = GYRO_DPS_DIGIT_500DPS;

        return true;
    }

    Eigen::Vector3d LSM9DS0::getAccel(){
        byte buffer[6];
        read(XMTYPE, REGISTER_OUT_X_L_A, buffer, 6);

        Eigen::Vector3d a(
            (int16_t)((int16_t)buffer[1] << 8 | buffer[0]) * a_scale * SENSORS_GRAVITY_STANDARD / 1000,
            (int16_t)((int16_t)buffer[3] << 8 | buffer[2]) * a_scale * SENSORS_GRAVITY_STANDARD / 1000,
            (int16_t)((int16_t)buffer[5] << 8 | buffer[4]) * a_scale * SENSORS_GRAVITY_STANDARD / 1000
        );
        return a;
    }

    Eigen::Vector3d LSM9DS0::getMag(){
        byte buffer[6];
        read(XMTYPE, REGISTER_OUT_X_L_M, buffer, 6);

        Eigen::Vector3d m(
            (int16_t)((int16_t)buffer[1] << 8 | buffer[0]) * m_scale / 10,
            (int16_t)((int16_t)buffer[3] << 8 | buffer[2]) * m_scale / 10,
            (int16_t)((int16_t)buffer[5] << 8 | buffer[4]) * m_scale / 10
        );
        return m;
    }

    Eigen::Vector3d LSM9DS0::getGyro(){
        byte buffer[6];
        read(GYROTYPE, REGISTER_OUT_X_L_G, buffer, 6);
        
        Eigen::Vector3d g(
            (int16_t)((int16_t)buffer[1] << 8 | buffer[0]) * g_scale * SENSORS_DPS_TO_RADS,
            (int16_t)((int16_t)buffer[3] << 8 | buffer[2]) * g_scale * SENSORS_DPS_TO_RADS,
            (int16_t)((int16_t)buffer[5] << 8 | buffer[4]) * g_scale * SENSORS_DPS_TO_RADS
        );
        return g;
    }

    float LSM9DS0::getTemp(){
        byte buffer[2];
        read(XMTYPE, REGISTER_TEMP_OUT_L_XM, buffer, 2);
        return (int16_t)((int16_t)buffer[1] << 8 | buffer[0]) / 8.0 + 21.0;
    }

    void LSM9DS0::write(bool type, byte reg, byte *buffer, byte len) {
        spi->beginTransaction(SPISettings(SPI2_SPEED, MSBFIRST, SPI_MODE0));
        digitalWrite(type ? CS_GYRO : CS_XMAG, LOW);
        spi->transfer(reg | 0x40);
        for (int i = 0; i < len; i++)
            spi->transfer(buffer[i]);
        digitalWrite(type ? CS_GYRO : CS_XMAG, HIGH);
        spi->endTransaction();
    }

    void LSM9DS0::write(bool type, byte reg, byte value) {
        byte *buffer = &value;
        write(type, reg, buffer, 1);
    }

    void LSM9DS0::read(bool type, byte reg, byte *buffer, byte len) {
        spi->beginTransaction(SPISettings(SPI2_SPEED, MSBFIRST, SPI_MODE0));
        digitalWrite(type ? CS_GYRO : CS_XMAG, LOW);
        spi->transfer(reg | 0xC0);
        for (int i = 0; i < len; i++)
            buffer[i] = spi->transfer(0x00);
        digitalWrite(type ? CS_GYRO : CS_XMAG, HIGH);
        spi->endTransaction();
    }

    byte LSM9DS0::read(bool type, byte reg) {
        byte value;
        read(type, reg, &value, 1);
        return value;
    }

}