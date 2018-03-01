#include "SPI.h"

enum CC1101_Command {
  CC1101_SRES = 0x30, // Reset chip.
  CC1101_SFSTXON = 0x31, // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1). If in RX (with CCA): Go to a wait state where only the synthesizer is running (for quick RX / TX turnaround).
  CC1101_SXOFF = 0x32, // Turn off crystal oscillator.
  CC1101_SCAL = 0x33, // Calibrate frequency synthesizer and turn it off. SCAL can be strobed from IDLE mode without setting manual calibration mode (MCSM0.FS_AUTOCAL=0)
  CC1101_SRX = 0x34, // Enable RX. Perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1.
  CC1101_STX = 0x35, // In IDLE state: Enable TX. Perform calibration first if MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled: Only go to TX if channel is clear.
  CC1101_SIDLE = 0x36, // Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable.
  CC1101_SWOR = 0x38, // Start automatic RX polling sequence (Wake-on-Radio) as described in Section 19.5 if WORCTRL.RC_PD=0.
  CC1101_SPWD = 0x39, // Enter power down mode when CSn goes high.
  CC1101_SFRX = 0x3A, // Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states.
  CC1101_SFTX = 0x3B, // Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states.
  CC1101_SWORRST = 0x3C, // Reset real time clock to Event1 value.
  CC1101_SNOP = 0x3D // No operation. May be used to get access to the chip status byte.
};

enum CC1101_reg {
  // status registers
  CC1101_PARTNUM = 0x30, // Part number for CC1101
  CC1101_VERSION = 0x31, // Current version number
  CC1101_FREQEST = 0x32, // Frequency Offset Estimate
  CC1101_LQI = 0x33, //Demodulator estimate for Link Quality
  CC1101_RSSI = 0x34, //Received signal strength indication
  CC1101_MARCSTATE = 0x35, //Control state machine state
  CC1101_WORTIME1 = 0x36, //High byte of WOR timer
  CC1101_WORTIME0 = 0x37, //Low byte of WOR timer
  CC1101_PKTSTATUS = 0x38, //Current GDOx status and packet status
  CC1101_VCO_VC_DAC = 0x39, // Current setting from PLL calibration module
  CC1101_TXBYTES = 0x3A, //Underflow and number of bytes in the TX FIFO
  CC1101_RXBYTES = 0x3B, //Overflow and number of bytes in the RX FIFO
  CC1101_RCCTRL1_STATUS = 0x3C, //Last RC oscillator calibration result
  CC1101_RCCTRL0_STATUS = 0x3D, //Last RC oscillator calibration result

  // configuration registers
  // don't really recommend touching these
  CC1101_IOCFG2 = 0x00, // GDO2 output pin configuration
  CC1101_IOCFG1 = 0x01, // GDO1 output pin configuration
  CC1101_IOCFG0 = 0x02, // GDO0 output pin configuration
  CC1101_FIFOTHR = 0x03, // RX FIFO and TX FIFO thresholds
  CC1101_SYNC1 = 0x04, // Sync word, high byte
  CC1101_SYNC0 = 0x05, // Sync word, low byte
  CC1101_PKTLEN = 0x06, // Packet length
  CC1101_PKTCTRL1 = 0x07, // Packet automation control
  CC1101_PKTCTRL0 = 0x08, // Packet automation control
  CC1101_ADDR = 0x09, // Device address
  CC1101_CHANNR = 0x0A, // Channel number
  CC1101_FSCTRL1 = 0x0B, // Frequency synthesizer control
  CC1101_FSCTRL0 = 0x0C, // Frequency synthesizer control
  CC1101_FREQ2 = 0x0D, // Frequency control word, high byte
  CC1101_FREQ1 = 0x0E, // Frequency control word, middle byte
  CC1101_FREQ0 = 0x0F, // Frequency control word, low byte
  CC1101_MDMCFG4 = 0x10, // Modem configuration
  CC1101_MDMCFG3 = 0x11, // Modem configuration
  CC1101_MDMCFG2 = 0x12, // Modem configuration
  CC1101_MDMCFG1 = 0x13, // Modem configuration
  CC1101_MDMCFG0 = 0x14, // Modem configuration
  CC1101_DEVIATN = 0x15, // Modem deviation setting
  CC1101_MCSM2 = 0x16, // Main Radio Control State Machine configuration
  CC1101_MCSM1 = 0x17, // Main Radio Control State Machine configuration
  CC1101_MCSM0 = 0x18, // Main Radio Control State Machine configuration
  CC1101_FOCCFG = 0x19, // Frequency Offset Compensation configuration
  CC1101_BSCFG = 0x1A, // Bit Synchronization configuration
  CC1101_AGCCTRL2 = 0x1B, // AGC control
  CC1101_AGCCTRL1 = 0x1C, // AGC control
  CC1101_AGCCTRL0 = 0x1D, // AGC control
  CC1101_WOREVT1 = 0x1E, // High byte Event 0 timeout
  CC1101_WOREVT0 = 0x1F, // Low byte Event 0 timeout
  CC1101_WORCTRL = 0x20, // Wake On Radio control
  CC1101_FREND1 = 0x21, // Front end RX configuration
  CC1101_FREND0 = 0x22, // Front end TX configuration
  CC1101_FSCAL3 = 0x23, // Frequency synthesizer calibration
  CC1101_FSCAL2 = 0x24, // Frequency synthesizer calibration
  CC1101_FSCAL1 = 0x25, // Frequency synthesizer calibration
  CC1101_FSCAL0 = 0x26, // Frequency synthesizer calibration
  CC1101_RCCTRL1 = 0x27, // RC oscillator configuration
  CC1101_RCCTRL0 = 0x28, // RC oscillator configuration
  CC1101_FSTEST = 0x29, // Frequency synthesizer calibration
  CC1101_PTEST = 0x2A, // Production test
  CC1101_AGCTEST = 0x2B, // AGC test
  CC1101_TEST2 = 0x2C, // Various test settings
  CC1101_TEST1 = 0x2D, // Various test settings
  CC1101_TEST0 = 0x2E, // Various test settings
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
  uint8_t readReg(CC1101_reg addr);
  void writeReg(CC1101_reg addr, uint8_t val);
  void burstReadReg(CC1101_reg addr, uint8_t * output, uint8_t len);
  void burstWriteReg(CC1101_reg addr, uint8_t * input, uint8_t len);
  void sendCommand(CC1101_Command command);
  uint8_t receiveByte();
  void receive(uint8_t * data, uint8_t len)
  void transmitByte(uint8_t data);
  void transmit(uint8_t * data, uint8_t len);
  void setRecieve();
  void setTransmit();
  uint8_t bytesInTxBuffer();
  uint8_t bytesInRxBuffer();
  void setChannel(uint8_t channel);
  void loadSettings(const uint8_t settings[][2], uint8_t len);
};
