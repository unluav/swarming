#include <Arduino.h>
#include <SPI.h>
#include <CC1101.h>

CC1101 transmitter;
CC1101 receiver;

static const uint8_t defaultSettings[][2] =
{
  {CC1101_IOCFG0,      0x06},
  {CC1101_PKTCTRL0,    0x05},
  {CC1101_FSCTRL1,     0x0C},
  {CC1101_FREQ2,       0x23},
  {CC1101_FREQ1,       0x31},
  {CC1101_FREQ0,       0x3B},
  {CC1101_MDMCFG4,     0x2D},
  {CC1101_MDMCFG3,     0x3B},
  {CC1101_MDMCFG2,     0x13},
  {CC1101_DEVIATN,     0x62},
  {CC1101_MCSM0,       0x18},
  {CC1101_FOCCFG,      0x1D},
  {CC1101_BSCFG,       0x1C},
  {CC1101_AGCCTRL2,    0xC7},
  {CC1101_AGCCTRL1,    0x00},
  {CC1101_AGCCTRL0,    0xB0},
  {CC1101_WORCTRL,     0xFB},
  {CC1101_FREND1,      0xB6},
  {CC1101_FSCAL3,      0xEA},
  {CC1101_FSCAL2,      0x2A},
  {CC1101_FSCAL1,      0x00},
  {CC1101_FSCAL0,      0x1F},
  {CC1101_TEST0,       0x09},
};

void setup() {
  SPI.begin();
  Serial.begin(9600);
  transmitter.init(5);
  receiver.init(4);
  transmitter.loadSettings(defaultSettings, 23);
  receiver.loadSettings(defaultSettings, 23);
  transmitter.setTransmit();
  receiver.setRecieve();
}

void loop() {
  char tx_data[] = "Test";
  transmitter.transmit((uint8_t*) tx_data, 5);

  while(transmitter.bytesInTxBuffer() != 0);
  while(receiver.bytesInRxBuffer() != 5);

  uint8_t rxdata[5] = {0, 0, 0, 0, 0};
  receiver.receive(rxdata, 5);

  Serial.println((char *) rxdata);
  delay(1000);
}
