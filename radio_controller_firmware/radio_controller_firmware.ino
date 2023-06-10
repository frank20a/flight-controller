#include <SPI.h>
#include <RF24.h>

SPIClass* hspi;
RF24 radio(25, 26);
char buffer[12];
const uint8_t addr[2][6] = {"CONTR", "DRONE"};

void setup() {
    Serial.begin(115200);
    hspi = new SPIClass(HSPI);
    hspi->begin(14, 32, 33);

    pinMode(25, OUTPUT);
    pinMode(26, OUTPUT);

    if (!radio.begin(hspi)){
      Serial.println("RADIO FAILED");
      while(1);
    }

    radio.setChannel(13);
    radio.setPALevel(RF24_PA_MIN);
    radio.setPayloadSize(12);
    radio.openReadingPipe(1, addr[0]);
    radio.openWritingPipe(addr[1]);
    radio.stopListening();
}

void loop() {
  if (Serial.available() > 11) {
    Serial.readBytes(buffer, 12);
    if (!radio.write(buffer, 12))
      Serial.println("FAILED");
    else
      Serial.println("OK");
  }
}




