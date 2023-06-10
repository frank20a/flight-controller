#include "main.h"

void setup() {
    // ================= Start Comms =================
    Serial.begin(UART0_SPEED);                // Start UART0
    main_attr.spi2.begin(SCLK2, MISO2, MOSI2, CS_XMAG); // Start SPI2

    delay(1000);

    // ================= Start Services =================
    // ==== IMU ====
    if (!main_srvs.lsm.begin()) {
        Serial.println("Failed to communicate with LSM9DS0.");
        while (1);
    }
    // ==== AHRS ====
    if (!main_srvs.ahrs_filter.begin()) {
        Serial.println("Failed to initialize AHRS filter.");
        while (1);
    }
    // ==== RC ====
    if (!main_srvs.radio.begin()) {
        Serial.println("Failed to initialize RC.");
        while (1);
    }
    // ==== Control ====
    if (!main_srvs.controller.begin()) {
        Serial.println("Failed to initialize Controller.");
        while (1);
    }
    // ==== GPS ====
    // ==== Debugger ====
    #ifdef DEBUG
    if (!main_srvs.debugger.begin()) {
        Serial.println("Failed to initialize Debugger.");
        while (1);
    }
    #endif
}

void loop() {
}