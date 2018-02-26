#include "cc1101.h"
#include "SPI.h"

#define ADDRESS_MASK 0x3f
#define READ_MASK 0x80
#define BURST_READ_MASK 0xc0
#define BURST_WRITE_MASK 0x40

#define SINGLE_TX 0x3f
#define BURST_TX 0x7f
#define SINGLE_RX 0xbf00
#define BURST_RX 0xff

void CC1101::start_transaction(){
  digitalWrite(select_pin, LOW);
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
}

void CC1101::stop_transaction(){
  digitalWrite(select_pin, HIGH);
  SPI.endTransaction();
}

void CC1101::init(uint8_t csn){
  select_pin = csn;
  pinMode(select_pin, OUTPUT);
  digitalWrite(select_pin, HIGH);
}

uint8_t CC1101::readReg(uint16_t addr) {
  addr &= ADDRESS_MASK;
  addr |= READ_MASK;
  addr = addr << 8;
  start_transaction();
  uint16_t data = SPI.transfer16(addr);
  stop_transaction();
  return data;
}

void CC1101::writeReg(uint8_t addr, uint8_t val){
  addr &= ADDRESS_MASK;
  uint16_t data = (addr << 8) | val;
  start_transaction();
  SPI.transfer16(data);
  stop_transaction();
}

void CC1101::burstReadReg(uint8_t addr, uint8_t * output, uint8_t len){
  addr &= ADDRESS_MASK;
  addr |= BURST_READ_MASK;
  start_transaction();
  SPI.transfer(addr);
  uint8_t i;
  for(i = 0; i < len; i++){
    output[i] = SPI.transfer(0);
  }
  stop_transaction();
}

void CC1101::burstWriteReg(uint8_t addr, uint8_t * input, uint8_t len){
  addr &= ADDRESS_MASK;
  addr |= BURST_WRITE_MASK;
  start_transaction();
  SPI.transfer(addr);
  uint8_t i;
  for(i = 0; i < len; i++){
    SPI.transfer(input[i]);
  }
  stop_transaction();
}

void CC1101::sendCommand(CC1101_Command command){
  start_transaction();
  SPI.transfer(command);
  stop_transaction();
}

uint8_t CC1101::receiveByte(){
  start_transaction();
  uint16_t data = SPI.transfer16(SINGLE_RX);
  stop_transaction();

  return data;
}

void CC1101::transmitByte(uint8_t data){
  start_transaction();
  SPI.transfer(SINGLE_TX);
  SPI.transfer(data);
  stop_transaction();
}

void CC1101::transmit(uint8_t * data, uint8_t len){
  start_transaction();
  SPI.transfer(BURST_TX);
  uint8_t i;
  for(i = 0; i < len; i++){
    SPI.transfer(data[i]);
  }
  stop_transaction();
}

void CC1101::setRecieve(){
  mode = RECEIVE;
  sendCommand(SRX);
}

void CC1101::setTransmit(){
  mode = TRANSMIT;
  sendCommand(STX);
}
