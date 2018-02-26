#include "SPI.h"

enum CC1101_Command {
  SRES = 0x30, // Reset chip.
  SFSTXON = 0x31, // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1). If in RX (with CCA): Go to a wait state where only the synthesizer is running (for quick RX / TX turnaround).
  SXOFF = 0x32, // Turn off crystal oscillator.
  SCAL = 0x33, // Calibrate frequency synthesizer and turn it off. SCAL can be strobed from IDLE mode without setting manual calibration mode (MCSM0.FS_AUTOCAL=0)
  SRX = 0x34, // Enable RX. Perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1.
  STX = 0x35, // In IDLE state: Enable TX. Perform calibration first if MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled: Only go to TX if channel is clear.
  SIDLE = 0x36, // Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable.
  SWOR = 0x38, // Start automatic RX polling sequence (Wake-on-Radio) as described in Section 19.5 if WORCTRL.RC_PD=0.
  SPWD = 0x39, // Enter power down mode when CSn goes high.
  SFRX = 0x3A, // Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states.
  SFTX = 0x3B, // Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states.
  SWORRST = 0x3C, // Reset real time clock to Event1 value.
  SNOP = 0x3D // No operation. May be used to get access to the chip status byte.
};

enum CC1101_Mode {
  IDLE,
  RECEIVE,
  TRANSMIT
};

class CC1101 {
private:
  void start_transaction();
  void stop_transaction();
public:
  uint8_t select_pin;
  CC1101_Mode mode = IDLE;
  void init(uint8_t csn);
  uint8_t readReg(uint16_t addr);
  void writeReg(uint8_t addr, uint8_t val);
  void burstReadReg(uint8_t addr, uint8_t * output, uint8_t len);
  void burstWriteReg(uint8_t addr, uint8_t * input, uint8_t len);
  void sendCommand(CC1101_Command command);
  uint8_t receiveByte();
  void transmitByte(uint8_t data);
  void transmit(uint8_t * data, uint8_t len);
  void setRecieve();
  void setTransmit();
};

// status registers
#define PARTNUM 0x30 // Part number for CC1101
#define VERSION 0x31 // Current version number
#define FREQEST 0x32 // Frequency Offset Estimate
#define LQI 0x33 //Demodulator estimate for Link Quality
#define RSSI 0x34 //Received signal strength indication
#define MARCSTATE 0x35 //Control state machine state
#define WORTIME1 0x36 //High byte of WOR timer
#define WORTIME0 0x37 //Low byte of WOR timer
#define PKTSTATUS 0x38 //Current GDOx status and packet status
#define VCO_VC_DAC 0x39 // Current setting from PLL calibration module
#define TXBYTES 0x3A //Underflow and number of bytes in the TX FIFO
#define RXBYTES 0x3B //Overflow and number of bytes in the RX FIFO
#define RCCTRL1_STATUS 0x3C //Last RC oscillator calibration result
#define RCCTRL0_STATUS 0x3D //Last RC oscillator calibration result

// configuration registers
// don't really recommend touching these
#define IOCFG2 0x00 // GDO2 output pin configuration
#define IOCFG1 0x01 // GDO1 output pin configuration
#define IOCFG0 0x02 // GDO0 output pin configuration
#define FIFOTHR 0x03 // RX FIFO and TX FIFO thresholds
#define SYNC1 0x04 // Sync word, high byte
#define SYNC0 0x05 // Sync word, low byte
#define PKTLEN 0x06 // Packet length
#define PKTCTRL1 0x07 // Packet automation control
#define PKTCTRL0 0x08 // Packet automation control
#define ADDR 0x09 // Device address
#define CHANNR 0x0A // Channel number
#define FSCTRL1 0x0B // Frequency synthesizer control
#define FSCTRL0 0x0C // Frequency synthesizer control
#define FREQ2 0x0D // Frequency control word, high byte
#define FREQ1 0x0E // Frequency control word, middle byte
#define FREQ0 0x0F // Frequency control word, low byte
#define MDMCFG4 0x10 // Modem configuration
#define MDMCFG3 0x11 // Modem configuration
#define MDMCFG2 0x12 // Modem configuration
#define MDMCFG1 0x13 // Modem configuration
#define MDMCFG0 0x14 // Modem configuration
#define DEVIATN 0x15 // Modem deviation setting
#define MCSM2 0x16 // Main Radio Control State Machine configuration
#define MCSM1 0x17 // Main Radio Control State Machine configuration
#define MCSM0 0x18 // Main Radio Control State Machine configuration
#define FOCCFG 0x19 // Frequency Offset Compensation configuration
#define BSCFG 0x1A // Bit Synchronization configuration
#define AGCTRL2 0x1B // AGC control
#define AGCTRL1 0x1C // AGC control
#define AGCTRL0 0x1D // AGC control
#define WOREVT1 0x1E // High byte Event 0 timeout
#define WOREVT0 0x1F // Low byte Event 0 timeout
#define WORCTRL 0x20 // Wake On Radio control
#define FREND1 0x21 // Front end RX configuration
#define FREND0 0x22 // Front end TX configuration
#define FSCAL3 0x23 // Frequency synthesizer calibration
#define FSCAL2 0x24 // Frequency synthesizer calibration
#define FSCAL1 0x25 // Frequency synthesizer calibration
#define FSCAL0 0x26 // Frequency synthesizer calibration
#define RCCTRL1 0x27 // RC oscillator configuration
#define RCCTRL0 0x28 // RC oscillator configuration
#define FSTEST 0x29 // Frequency synthesizer calibration
#define PTEST 0x2A // Production test
#define AGCTEST 0x2B // AGC test
#define TEST2 0x2C // Various test settings
#define TEST1 0x2D // Various test settings
#define TEST0 0x2E // Various test settings
