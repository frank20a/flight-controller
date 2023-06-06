#pragma once

#include <SPI.h>
#include <cmath>
#include <ArduinoEigen.h>

// Constants
#define SENSORS_GRAVITY_EARTH (9.80665F)
#define SENSORS_GRAVITY_MOON (1.6F)
#define SENSORS_GRAVITY_SUN (275.0F)
#define SENSORS_GRAVITY_STANDARD (SENSORS_GRAVITY_EARTH)
#define SENSORS_MAGFIELD_EARTH_MAX (60.0F)
#define SENSORS_MAGFIELD_EARTH_MIN (30.0F)
#define SENSORS_PRESSURE_SEALEVELHPA (1013.25F)
#define SENSORS_DPS_TO_RADS (0.017453293F)
#define SENSORS_RADS_TO_DPS (57.29577793F)
#define SENSORS_GAUSS_TO_MICROTESLA (100)

// Sensor types
#define GYROTYPE (true)
#define XMTYPE (false)

// Sensor IDs
#define XMAG_ID (0b01001001)
#define GYRO_ID (0b11010100)

// Linear Acceleration: mg per LSB 
#define ACCEL_MG_LSB_2G (0.061F)
#define ACCEL_MG_LSB_4G (0.122F)
#define ACCEL_MG_LSB_6G (0.183F)
#define ACCEL_MG_LSB_8G (0.244F)
#define ACCEL_MG_LSB_16G (0.732F)

// Magnetic Field Strength: gauss range
#define MAG_MGAUSS_2GAUSS (0.08F)
#define MAG_MGAUSS_4GAUSS (0.16F)
#define MAG_MGAUSS_8GAUSS (0.32F)
#define MAG_MGAUSS_12GAUSS (0.48F)

// Angular Rate: dps per LSB
#define GYRO_DPS_DIGIT_245DPS (0.00875F)
#define GYRO_DPS_DIGIT_500DPS (0.01750F)
#define GYRO_DPS_DIGIT_2000DPS (0.07000F)

// Temperature: LSB per degree celsius
#define TEMP_LSB_DEGREE_CELSIUS (8)

// FIR Filter Sizes
#define FIR_ACC_SIZE 27
#define FIR_MAG_SIZE 9
#define FIR_GYR_SIZE 13

namespace LSM9DS0 {
    class LSM9DS0 {
    public:
        LSM9DS0(
            SPIClass *spi_, 
            SemaphoreHandle_t *spi_mutex_, 
            unsigned int acc_rate_,
            Eigen::Vector3f *acc_,
            SemaphoreHandle_t *acc_mutex_, 
            unsigned int mag_rate_,
            Eigen::Vector3f *mag_,
            SemaphoreHandle_t *mag_mutex_, 
            unsigned int gyr_rate_,
            Eigen::Vector3f *gyr_,
            SemaphoreHandle_t *gyr_mutex_
        ){
            spi = spi_;
            spi_mutex = spi_mutex_;

            acc_rate = acc_rate_;
            acc = acc_;
            acc_mutex = acc_mutex_;

            mag_rate = mag_rate_;
            mag = mag_;
            mag_mutex = mag_mutex_;

            gyr_rate = gyr_rate_;
            gyr = gyr_;
            gyr_mutex = gyr_mutex_;
        }
        bool begin();
        void calibrate_gyro();

        typedef enum {
            REGISTER_WHO_AM_I_G = 0x0F,
            REGISTER_CTRL_REG1_G = 0x20,
            REGISTER_CTRL_REG3_G = 0x22,
            REGISTER_CTRL_REG4_G = 0x23,
            REGISTER_OUT_X_L_G = 0x28,
            REGISTER_OUT_X_H_G = 0x29,
            REGISTER_OUT_Y_L_G = 0x2A,
            REGISTER_OUT_Y_H_G = 0x2B,
            REGISTER_OUT_Z_L_G = 0x2C,
            REGISTER_OUT_Z_H_G = 0x2D,
        } gyro_regs_t;

        typedef enum {
            REGISTER_TEMP_OUT_L_XM = 0x05,
            REGISTER_TEMP_OUT_H_XM = 0x06,
            REGISTER_STATUS_REG_M = 0x07,
            REGISTER_OUT_X_L_M = 0x08,
            REGISTER_OUT_X_H_M = 0x09,
            REGISTER_OUT_Y_L_M = 0x0A,
            REGISTER_OUT_Y_H_M = 0x0B,
            REGISTER_OUT_Z_L_M = 0x0C,
            REGISTER_OUT_Z_H_M = 0x0D,
            REGISTER_WHO_AM_I_XM = 0x0F,
            REGISTER_INT_CTRL_REG_M = 0x12,
            REGISTER_INT_SRC_REG_M = 0x13,
            REGISTER_CTRL_REG1_XM = 0x20,
            REGISTER_CTRL_REG2_XM = 0x21,
            REGISTER_CTRL_REG5_XM = 0x24,
            REGISTER_CTRL_REG6_XM = 0x25,
            REGISTER_CTRL_REG7_XM = 0x26,
            REGISTER_OUT_X_L_A = 0x28,
            REGISTER_OUT_X_H_A = 0x29,
            REGISTER_OUT_Y_L_A = 0x2A,
            REGISTER_OUT_Y_H_A = 0x2B,
            REGISTER_OUT_Z_L_A = 0x2C,
            REGISTER_OUT_Z_H_A = 0x2D,
        } xmag_regs_t;

        typedef enum {
            ACCELRANGE_2G = (0b000 << 3),
            ACCELRANGE_4G = (0b001 << 3),
            ACCELRANGE_6G = (0b010 << 3),
            ACCELRANGE_8G = (0b011 << 3),
            ACCELRANGE_16G = (0b100 << 3)
        } accel_range_t;

        typedef enum {
            ACCELDATARATE_POWERDOWN = (0b0000 << 4),
            ACCELDATARATE_3_125HZ = (0b0001 << 4),
            ACCELDATARATE_6_25HZ = (0b0010 << 4),
            ACCELDATARATE_12_5HZ = (0b0011 << 4),
            ACCELDATARATE_25HZ = (0b0100 << 4),
            ACCELDATARATE_50HZ = (0b0101 << 4),
            ACCELDATARATE_100HZ = (0b0110 << 4),
            ACCELDATARATE_200HZ = (0b0111 << 4),
            ACCELDATARATE_400HZ = (0b1000 << 4),
            ACCELDATARATE_800HZ = (0b1001 << 4),
            ACCELDATARATE_1600HZ = (0b1010 << 4)
        } accel_rate_t;

        typedef enum {
            MAGGAIN_2GAUSS = (0b00 << 5),
            MAGGAIN_4GAUSS = (0b01 << 5),
            MAGGAIN_8GAUSS = (0b10 << 5),
            MAGGAIN_12GAUSS = (0b11 << 5)
        } mag_scale_t;
        
        typedef enum {
            MAGDATARATE_3_125HZ = (0b000 << 2),
            MAGDATARATE_6_25HZ = (0b001 << 2),
            MAGDATARATE_12_5HZ = (0b010 << 2),
            MAGDATARATE_25HZ = (0b011 << 2),
            MAGDATARATE_50HZ = (0b100 << 2),
            MAGDATARATE_100HZ = (0b101 << 2)
        } mag_rate_t;
        
        typedef enum {
            GYROSCALE_245DPS = (0b00 << 4),
            GYROSCALE_500DPS = (0b01 << 4),
            GYROSCALE_2000DPS = (0b10 << 4)
        } gyro_scale_t;

        typedef enum {
            GYRODATARATE_95HZ = (0b00 << 6),
            GYRODATARATE_190HZ = (0b01 << 6),
            GYRODATARATE_380HZ = (0b10 << 6),
            GYRODATARATE_760HZ = (0b11 << 6)
        } gyro_rate_t;

    private:
        // Primitive SPI Methods
        void write(bool type, byte reg, byte *buffer, byte len);
        void write(bool type, byte reg, byte value);
        void read(bool type, byte reg, byte *buffer, byte len);
        byte read(bool type, byte reg);

        // Primitive Data Methods
        Eigen::Vector3f getAccel();
        Eigen::Vector3f getMag();
        Eigen::Vector3f getGyro();
        float getTemp();

        // Task handling Methods
        void task(byte type);
        static void a_task_wrapper(void *pvParam) { static_cast<LSM9DS0*>(pvParam)->task(0); }
        static void m_task_wrapper(void *pvParam) { static_cast<LSM9DS0*>(pvParam)->task(1); }
        static void g_task_wrapper(void *pvParam) { static_cast<LSM9DS0*>(pvParam)->task(2); }

        // Filter Methods
        Eigen::Vector3f filter_update(Eigen::Vector3f data, float *h, float *hist, unsigned char s, unsigned char *idx);
        Eigen::Vector3f filter_acc_update(Eigen::Vector3f data) { return filter_update(data, h_acc, acc_hist, FIR_ACC_SIZE, &acc_hist_idx); };
        Eigen::Vector3f filter_mag_update(Eigen::Vector3f data) { return filter_update(data, h_mag, mag_hist, FIR_MAG_SIZE, &mag_hist_idx); };
        Eigen::Vector3f filter_gyr_update(Eigen::Vector3f data) { return filter_update(data, h_gyr, gyr_hist, FIR_GYR_SIZE, &gyr_hist_idx); };

        // Data handling Members
        SPIClass *spi;
        SemaphoreHandle_t *spi_mutex, *acc_mutex, *mag_mutex, *gyr_mutex;
        Eigen::Vector3f *acc, *mag, *gyr;
        int acc_rate, mag_rate, gyr_rate;
        float a_scale, g_scale, m_scale;

        // Filter Members
        float h_acc[FIR_ACC_SIZE] = {
            -0.002038178257063346,
            -0.000891279528166555,
            0.003834856328818554,
            0.017848381513384096,
            0.044520862883303446,
            0.081744026456171404,
            0.121367479368390846,
            0.151911488670050360,
            0.163404725130222167,
            0.151911488670050360,
            0.121367479368390860,
            0.081744026456171431,
            0.044520862883303459,
            0.017848381513384092,
            0.003834856328818555,
            -0.000891279528166556,
            -0.002038178257063346
        };
        float h_mag[FIR_MAG_SIZE] = {
            -0.006140414697945076,
            -0.013581674476961783,
            0.051232297342717927,
            0.265655560905450672,
            0.405668461853476436,
            0.265655560905450727,
            0.051232297342717940,
            -0.013581674476961786,
            -0.006140414697945076
        };
        float h_gyr[FIR_GYR_SIZE] = {
            -0.004301908977802689,
            -0.005468682880462111,
            0.003190449833585041,
            0.045555056080673402,
            0.126207997243587056,
            0.210990162636316508,
            0.247653852128205509,
            0.210990162636316536,
            0.126207997243587111,
            0.045555056080673409,
            0.003190449833585041,
            -0.005468682880462115,
            -0.004301908977802689
        };
        float acc_hist[FIR_ACC_SIZE * 3] = {0}, mag_hist[FIR_MAG_SIZE * 3] = {0}, gyr_hist[FIR_GYR_SIZE * 3] = {0};
        unsigned char acc_hist_idx = 0, mag_hist_idx = 0, gyr_hist_idx = 0;
    };
}