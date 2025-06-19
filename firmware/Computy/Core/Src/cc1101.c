#include "cc1101.h"
#include "stm32f2xx_hal.h"  // Or your specific HAL

// SPI handle (must be initialized elsewhere)
extern SPI_HandleTypeDef hspi1;

// Chip Select pin configuration (PB6)
#define CC1101_CS_PORT   GPIOB
#define CC1101_CS_PIN    GPIO_PIN_6

// Helper macros for CS line
#define CC1101_CS_LOW()  HAL_GPIO_WritePin(CC1101_CS_PORT, CC1101_CS_PIN, GPIO_PIN_RESET)
#define CC1101_CS_HIGH() HAL_GPIO_WritePin(CC1101_CS_PORT, CC1101_CS_PIN, GPIO_PIN_SET)

#define CC1101_STX  0x35  // Transmit command (from datasheet)

void cc1101_init(void) {
    // Initialize only the CS pin (PB6)
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = CC1101_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(CC1101_CS_PORT, &GPIO_InitStruct);
    
    CC1101_CS_HIGH();
    cc1101_reset();
}

void cc1101_reset(void) {
    CC1101_CS_LOW();
    uint8_t cmd = CC1101_SRES;
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    CC1101_CS_HIGH();
    HAL_Delay(1);  // Wait >40us (using 1ms for safety)
}

bool cc1101_test_spi(void) {

	uint8_t partnum = cc1101_read_reg(CC1101_PARTNUM);
    uint8_t version = cc1101_read_reg(CC1101_VERSION);
    uint8_t LQI = cc1101_read_reg(0xF3);
    uint8_t PKTSTATUS = cc1101_read_reg(0xF7);
    uint8_t RSSI = cc1101_read_reg(0xF4);
    uint8_t MARCSTATE = cc1101_read_reg(0xF5);
    uint8_t TEST1 = cc1101_read_reg(0x2D);
    uint8_t TEST2 = cc1101_read_reg(0x2C);
    uint8_t FSTEST = cc1101_read_reg(0x29);

    // Check against known CC1101 values
    return  (version != 0x00);
}

uint8_t cc1101_read_reg(uint8_t addr) {
    // Configuration registers need burst header
    uint8_t tx_data[2], rx_data[2];

	if(addr <= 0x2F) {  // All config registers are 0x00-0x2F
        tx_data[0] = 0x80 | 0x40 | addr;  // Burst read header
    }
    else {  // Status/test registers
        tx_data[0] = 0x80 | addr;  // Normal read
    }
    tx_data[1] = 0x00; // dummy byte
    
    CC1101_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, HAL_MAX_DELAY);
    CC1101_CS_HIGH();
    
    return rx_data[1];
}

void cc1101_write_reg(uint8_t addr, uint8_t value) {
    uint8_t tx_data[2];

    if(addr <= 0x2F) {
        tx_data[0] = 0x40 | addr;  // Burst write header
    } else {
        tx_data[0] = addr;         // Normal write
    }

    tx_data[1] = value;
    
    CC1101_CS_LOW();
    HAL_SPI_Transmit(&hspi1, tx_data, 2, HAL_MAX_DELAY);
    CC1101_CS_HIGH();
}


void test_tx_functionality() {
    // 1. Configure basic RF settings (FSK, 433MHz, 38.4kbps)
    cc1101_write_reg(0x10, 0x0C);  // MODEMCFG1 (FSK, no Manchester)
    cc1101_write_reg(0x11, 0x00);  // MODEMCFG2 (standard settings)
    cc1101_write_reg(0x0C, 0x1D);  // FSCTRL1 (IF freq)
    cc1101_write_reg(0x0D, 0x40);  // FSCTRL0 (Freq offset)
    cc1101_write_reg(0x09, 0x12);  // CHANNR (channel 0)

    // 2. Send a test packet
    uint8_t tx_data[] = {0xAA, 0x55, 0x01, 0x02, 0x03};
    cc1101_write_reg(0x7F, 0xAA);  // Write to PATABLE (power setting)
    HAL_Delay(10);

    // 3. Enable TX mode
    cc1101_write_reg(0x35, CC1101_STX);  // STX command
    HAL_Delay(100);  // Transmission time

    // 4. Check MARCSTATE (0xF5) to confirm TX completion
    uint8_t state = cc1101_read_reg(0xF5);
    printf("MARCSTATE: 0x%02X (should be 0x01=IDLE after TX)\n", state);
}
