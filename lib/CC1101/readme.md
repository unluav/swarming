# CC1101
This is a driver library for a cc1101 radio. It uses both Arduino gpio functions and Arduino's SPI library.
## Example
```c++
CC1101 radio;

// 5 is the chip select pin
radio.init(5)
// set transmit mode
radio.setTransmit();
// transmit byte 0xff
radio.transmitByte(0xff);
uint8_t buffer[4] = [1, 2, 3, 4];
// transmit all the values in buffer
radio.transmit(buffer, 4);
```
