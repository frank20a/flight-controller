#include "lsm9ds0.h"

Eigen::Matrix3f Aa {
    { 0.973209,  0.006248, -0.000107},
    { 0.006248,  0.959535,  0.000811},
    {-0.000107,  0.000811,  1.019086}
};
Eigen::Vector3f ba {-0.611425, 0.195279, -0.229477};

Eigen::Matrix3f Am {
    { 1.024384, -0.067449,  0.005591},
    {-0.067449,  0.984567, -0.059303},
    { 0.005591, -0.059303,  1.093657}
};
Eigen::Vector3f bm {-15.850987, 46.848109, -15.884498};

Eigen::Vector3f bg {0.0, 0.0, 0.0};

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
                    
    // Calibrate the magnetometer
    #ifdef CALIBRATE_IMU
    calibrate_gyro();
    #endif

    // Start services
    xTaskCreatePinnedToCore(a_task_wrapper, "ACC", 2048, this, 1, NULL, 0);
    xTaskCreatePinnedToCore(m_task_wrapper, "MAG", 2048, this, 1, NULL, 0);
    xTaskCreatePinnedToCore(g_task_wrapper, "GYR", 2048, this, 1, NULL, 0);

    return true;
}

Eigen::Vector3f LSM9DS0::getAccel(){
    byte buffer[6];
    read(XMTYPE, REGISTER_OUT_X_L_A, buffer, 6);

    Eigen::Vector3f a(
        (int16_t)((int16_t)buffer[1] << 8 | buffer[0]) * a_scale * SENSORS_GRAVITY_STANDARD / 1000,
        (int16_t)((int16_t)buffer[3] << 8 | buffer[2]) * a_scale * SENSORS_GRAVITY_STANDARD / 1000,
        (int16_t)((int16_t)buffer[5] << 8 | buffer[4]) * a_scale * SENSORS_GRAVITY_STANDARD / 1000
    );
    return a;
}

Eigen::Vector3f LSM9DS0::getMag(){
    byte buffer[6];
    read(XMTYPE, REGISTER_OUT_X_L_M, buffer, 6);

    Eigen::Vector3f m(
        (int16_t)((int16_t)buffer[1] << 8 | buffer[0]) * m_scale / 10,
        (int16_t)((int16_t)buffer[3] << 8 | buffer[2]) * m_scale / 10,
        (int16_t)((int16_t)buffer[5] << 8 | buffer[4]) * m_scale / 10
    );
    return m;
}

Eigen::Vector3f LSM9DS0::getGyro(){
    byte buffer[6];
    read(GYROTYPE, REGISTER_OUT_X_L_G, buffer, 6);
    
    Eigen::Vector3f g(
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

void LSM9DS0::task(byte type) {
    Eigen::Vector3f tmp, *shared_data;
    SemaphoreHandle_t *data_mutex;
    int rate;
    switch(type) {
    case 0:
        data_mutex = this->acc_mutex;
        rate = this->acc_rate;
        shared_data = this->acc;
        break;
    case 1:
        data_mutex = this->mag_mutex;
        rate = this->mag_rate;
        shared_data = this->mag;
        break;
    case 2: 
        data_mutex = this->gyr_mutex;
        rate = this->gyr_rate;
        shared_data = this->gyr;
        break;
    }

    while(true) {
        // Get data
        if(xSemaphoreTake(*(this->spi_mutex), portMAX_DELAY)){
            if (type == 0) {
                tmp = this->getAccel();

                #ifdef CALIBRATE_IMU
                tmp = Aa * (tmp - ba);
                #endif
            } else if (type == 1) {
                tmp = this->getMag();
                
                #ifdef CALIBRATE_IMU
                tmp = Am * (tmp - bm);
                #endif
            } else if (type == 2) {
                tmp = this->getGyro();
                
                #ifdef CALIBRATE_IMU
                tmp = tmp - bg;
                #endif
            } else 
                return;

            xSemaphoreGive(*(this->spi_mutex));
        }

        // Filter data
        #ifdef FILTER_IMU
            switch(type) {
            case 0:
                tmp = this->filter_acc_update(tmp);
                break;
            case 1:
                tmp = this->filter_mag_update(tmp);
                break;
            case 2: 
                tmp = this->filter_gyr_update(tmp);
                break;
        }
        #endif

        // Update shared data
        if(xSemaphoreTake(*(data_mutex), 0)){
            *shared_data = tmp;
            xSemaphoreGive(*(data_mutex));
        }

        vTaskDelay(pdMS_TO_TICKS(1000 / rate));
    }
}

void LSM9DS0::calibrate_gyro() {
    bg.setZero();
    delay(500);
    if (xSemaphoreTake(*spi_mutex, portMAX_DELAY)) {
        for (int i = 0; i < GYRO_CALIB_SAMPLES; i++) {
            bg += this->getGyro();
            vTaskDelay(pdMS_TO_TICKS(1000 / 150));
        }
        xSemaphoreGive(*spi_mutex);
    }
    bg /= GYRO_CALIB_SAMPLES;
}

Eigen::Vector3f LSM9DS0::filter_update(Eigen::Vector3f data, float *h, float *hist, unsigned char s, unsigned char *idx){
        Eigen::Vector3f res;
        res.setZero();
    
        hist[s*0 + *idx] = data.x();
        hist[s*1 + *idx] = data.y();
        hist[s*2 + *idx] = data.z();

        for(int i = 0; i < s; i++){
            res.x() += hist[s*0 + (i + *idx) % s] * h[i];
            res.y() += hist[s*1 + (i + *idx) % s] * h[i];
            res.z() += hist[s*2 + (i + *idx) % s] * h[i];
        }

        (*idx)++;
        *idx %= s;

        return res;
    }
