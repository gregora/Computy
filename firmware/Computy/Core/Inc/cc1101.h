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

// Function prototypes
void cc1101_init(void);
bool cc1101_test_spi(void);
uint8_t cc1101_read_reg(uint8_t addr);
void cc1101_write_reg(uint8_t addr, uint8_t value);
void test_tx_functionality();
#endif
