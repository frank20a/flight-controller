#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ArduinoEigen.h>
#include <SPI.h>
#include <RF24.h>

#include "main_attr.h"

void IRAM_ATTR get_packet();

class Radio {
    public:
        Radio(MainAttr *attr_) {
            attr = attr_; 
            radio = RF24(CE_RF, CS_RF);
        };
        bool begin();

    protected:
        void get_packet();
        void task();
        static void task_wrapper(void *pvParameters) {
            static_cast<Radio *>(pvParameters)->task();
        }
        
        MainAttr *attr;
        RF24 radio;
        uint16_t c = 0;
        const uint8_t addr[2][6] = {"CONTR", "DRONE"};
};