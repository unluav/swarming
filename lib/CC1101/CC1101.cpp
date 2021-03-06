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

uint8_t CC1101::readReg(CC1101_reg addr) {
  uint16_t reg = addr;
  reg |= READ_MASK;
  reg = reg << 8;
  start_transaction();
  uint16_t data = SPI.transfer16(reg);
  stop_transaction();
  return data;
}

void CC1101::writeReg(CC1101_reg addr, uint8_t val){
  uint16_t data = (addr << 8) | val;
  start_transaction();
  SPI.transfer16(data);
  stop_transaction();
}

void CC1101::burstReadReg(CC1101_reg addr, uint8_t * output, uint8_t len){
  start_transaction();
  SPI.transfer(addr | BURST_READ_MASK);
  uint8_t i;
  for(i = 0; i < len; i++){
    output[i] = SPI.transfer(0);
  }
  stop_transaction();
}

void CC1101::burstWriteReg(CC1101_reg addr, uint8_t * input, uint8_t len){
  start_transaction();
  SPI.transfer(addr | BURST_WRITE_MASK);
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

void CC1101::receive(uint8_t * data, uint8_t len){
  start_transaction();
  SPI.transfer(BURST_RX);
  uint8_t i;
  for(i = 0; i < len; i++){
    data[i] = SPI.transfer(0);
  }
  stop_transaction();
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
  sendCommand(CC1101_SRX);
}

void CC1101::setTransmit(){
  mode = TRANSMIT;
  sendCommand(CC1101_STX);
}

void CC1101::loadSettings(const uint8_t settings[][2], uint8_t len){
  uint8_t i;
  for(i = 0; i < len; i++){
    writeReg((CC1101_reg) settings[i][0], settings[i][1]);
  }
}

uint8_t CC1101::bytesInTxBuffer(){
  readReg(CC1101_TXBYTES);
}

uint8_t CC1101::bytesInRxBuffer(){
  readReg(CC1101_RXBYTES);
}

void CC1101::setChannel(uint8_t channel){
  writeReg(CC1101_CHANNR, channel);
}