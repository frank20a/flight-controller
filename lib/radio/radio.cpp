#include "radio.h"

bool Radio::begin() {
    if (!radio.begin(&(attr->spi2)))
        return false;

    radio.setChannel(13);
    radio.setPALevel(RF24_PA_MIN);
    radio.setPayloadSize(12);
    radio.openReadingPipe(1, addr[1]);
    radio.openWritingPipe(addr[0]);
    radio.startListening();

    xTaskCreatePinnedToCore(task_wrapper, "Radio", 10000, this, 1, NULL, 0);

    return true;
}

void Radio::task() {
    while (true) {
        if (radio.available()) {
            get_packet();
            c = 0;

            vTaskDelay(pdMS_TO_TICKS(1000.0 / RADIO_RATE));
        } else {
            if(++c > 10 * RADIO_TIMEOUT * RADIO_RATE) {
                if(xSemaphoreTake(attr->controller_level_mutex, portMAX_DELAY)) {
                    memset(&(attr->controller_level), 0, sizeof(ControllerLevels));
                    xSemaphoreGive(attr->controller_level_mutex);
                }
            }

            vTaskDelay(pdMS_TO_TICKS(100.0 / RADIO_RATE));
        }
    }
}

void Radio::get_packet() {
    uint8_t packet[12];
    ControllerLevels ch_;

    radio.read(packet, 12);

    ch_.ch00 = ((*(uint16_t *)(packet + 0)) & 0b1111111111000000) >> 6;
    ch_.ch01 = ((*(uint16_t *)(packet + 2)) & 0b1111111111000000) >> 6;
    ch_.ch02 = ((*(uint16_t *)(packet + 4)) & 0b1111111111000000) >> 6;
    ch_.ch03 = ((*(uint16_t *)(packet + 6)) & 0b1111111111000000) >> 6;
    ch_.ch04 = packet[8];
    ch_.ch05 = packet[9];
    ch_.ch06 = packet[10];
    ch_.ch07 = packet[11];
    ch_.ch08 = (packet[1] & 0b00110000) >> 4;
    ch_.ch09 = (packet[1] & 0b00001100) >> 2;
    ch_.ch10 = (packet[1] & 0b00000011) >> 0;
    ch_.ch11 = (packet[3] & 0b00110000) >> 4;
    ch_.ch12 = (packet[3] & 0b00001100) >> 2;
    ch_.ch13 = (packet[3] & 0b00000011) >> 0;
    ch_.ch14 = (packet[5] & 0b00100000) >> 5;
    ch_.ch15 = (packet[5] & 0b00010000) >> 4;
    ch_.ch16 = (packet[5] & 0b00001000) >> 3;
    ch_.ch17 = (packet[5] & 0b00000100) >> 2;
    ch_.ch18 = (packet[5] & 0b00000010) >> 1;
    ch_.ch19 = (packet[5] & 0b00000001) >> 0;
    ch_.ch20 = (packet[7] & 0b00111000) >> 3;
    ch_.ch21 = (packet[7] & 0b00000111) >> 0;

    if(xSemaphoreTake(attr->controller_level_mutex, portMAX_DELAY)) {
        attr->controller_level = ch_;
        xSemaphoreGive(attr->controller_level_mutex);
    }

}
