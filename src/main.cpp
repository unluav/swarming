#include <Arduino.h>
#include <SPI.h>
#include <CC1101.h>

CC1101 radio;

void setup() {
  SPI.begin();
  Serial.begin(9600);
  radio.init(5);
  radio.setRecieve();
}

void loop() {
  uint8_t data = radio.receiveByte();
  Serial.write(data);
  delay(1000);
}
