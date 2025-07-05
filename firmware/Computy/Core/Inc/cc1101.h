#ifndef CC1101
#define CC1101

#include <stdint.h>
#include <stdbool.h>

// CC1101 Command Strobe Registers
#define CC1101_SRES     0x30    // Reset chip
#define CC1101_SNOP     0x3D    // No operation

// Identification registers
#define CC1101_PARTNUM  0x30    // Part number (should read 0x00 or 0x04)
#define CC1101_VERSION  0x31    // Version (should read 0x14, 0x04, or 0x07)
#define CC1101_SIDLE    0x36  // Exit RX/TX, go to IDLE
#define CC1101_SFTX     0x3B  // Flush TX FIFO
#define CC1101_TXFIFO   0x3F  // TX FIFO (write)
#define CC1101_STX      0x35  // Enable TX
#define CC1101_MARCSTATE 0x35 // Main Radio Control State Machine state


// Function prototypes
void cc1101_init(void);
bool cc1101_test_spi(void);
uint8_t cc1101_read_reg(uint8_t addr);
void cc1101_write_reg(uint8_t addr, uint8_t value);
void cc1101_write_reg_burst(uint8_t addr, uint8_t* buff, int len);
void cc1101_cmd_strobe(uint8_t cmd);
void cc1101_send_packet(uint8_t* buff, int len);
#endif
