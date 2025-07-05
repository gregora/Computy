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

    // Basic I/O and FIFO Configuration
    cc1101_write_reg(0x00, 0x29);  // IOCFG2 - GDO2 Output Pin Configuration
    cc1101_write_reg(0x01, 0x2E);  // IOCFG1 - GDO1 Output Pin Configuration
    cc1101_write_reg(0x02, 0x06);  // IOCFG0 - GDO0 Output Pin Configuration
    cc1101_write_reg(0x03, 0x47);  // FIFOTHR - RX/TX FIFO Thresholds

    // Sync Word and Packet Configuration
    cc1101_write_reg(0x04, 0xD3);  // SYNC1 - Sync Word High Byte
    cc1101_write_reg(0x05, 0x91);  // SYNC0 - Sync Word Low Byte
    cc1101_write_reg(0x06, 0xFF);  // PKTLEN - Packet Length
    cc1101_write_reg(0x07, 0x04);  // PKTCTRL1 - Packet Automation Control
    cc1101_write_reg(0x08, 0x05);  // PKTCTRL0 - Packet Automation Control
    cc1101_write_reg(0x09, 0x00);  // ADDR - Device Address

    // Frequency and Modem Configuration
    cc1101_write_reg(0x0D, 0x10);  // FREQ2 - Frequency Control Word High
    cc1101_write_reg(0x0E, 0xB4);  // FREQ1 - Frequency Control Word Mid
    cc1101_write_reg(0x0F, 0x2E);  // FREQ0 - Frequency Control Word Low
    cc1101_write_reg(0x10, 0xCA);  // MDMCFG4 - Modem Configuration
    cc1101_write_reg(0x11, 0x83);  // MDMCFG3 - Modem Configuration
    cc1101_write_reg(0x12, 0x93);  // MDMCFG2 - Modem Configuration
    cc1101_write_reg(0x13, 0x22);  // MDMCFG1 - Modem Configuration
    cc1101_write_reg(0x14, 0xF8);  // MDMCFG0 - Modem Configuration
    cc1101_write_reg(0x15, 0x34);  // DEVIATN - Modem Deviation Setting

    // State Machine and AGC Configuration
    cc1101_write_reg(0x16, 0x07);  // MCSM2 - Main Radio Control State Machine
    cc1101_write_reg(0x17, 0x30);  // MCSM1 - Main Radio Control State Machine
    cc1101_write_reg(0x18, 0x18);  // MCSM0 - Main Radio Control State Machine
    cc1101_write_reg(0x1B, 0x43);  // AGCCTRL2 - AGC Control
    cc1101_write_reg(0x1C, 0x40);  // AGCCTRL1 - AGC Control
    cc1101_write_reg(0x1D, 0x91);  // AGCCTRL0 - AGC Control

    // Front End and Calibration
    cc1101_write_reg(0x21, 0x56);  // FREND1 - Front End RX Configuration
    cc1101_write_reg(0x22, 0x10);  // FREND0 - Front End TX Configuration
    cc1101_write_reg(0x23, 0xE9);  // FSCAL3 - Frequency Synthesizer Calibration
    cc1101_write_reg(0x24, 0x2A);  // FSCAL2 - Frequency Synthesizer Calibration
    cc1101_write_reg(0x25, 0x00);  // FSCAL1 - Frequency Synthesizer Calibration
    cc1101_write_reg(0x26, 0x1F);  // FSCAL0 - Frequency Synthesizer Calibration

    // Calibrate and verify
    cc1101_cmd_strobe(0x33);  // SCAL - Calibrate frequency synthesizer
    while(cc1101_read_reg(0xF5) != 0x01);  // Wait for MARCSTATE to return to IDLE

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

	uint8_t tx[2] = {addr, value};

    
    CC1101_CS_LOW();
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    CC1101_CS_HIGH();
}

void cc1101_write_reg_burst(uint8_t addr, uint8_t* buff, int len) {

    if(len > 1) {
        addr = 0x40 | addr;  // Burst write header
    }


    CC1101_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &addr, 1, HAL_MAX_DELAY);

    HAL_SPI_Transmit(&hspi1, buff, len, HAL_MAX_DELAY);
    CC1101_CS_HIGH();
}

void cc1101_cmd_strobe(uint8_t cmd) {

    CC1101_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY); // Send strobe byte
    CC1101_CS_HIGH();
}

void cc1101_send_packet(uint8_t* buff, int len) {

    cc1101_cmd_strobe(0x36); // Exit RX / TX, turn off frequency synthesizer

    uint8_t packets_left;
    uint8_t state;

    state = cc1101_read_reg(0xF5);

    cc1101_write_reg(0x06, len);

    // 2. Send a test packet
    // uint8_t tx_data[] = {0xAA, 0x55, 0x01, 0x02, 0x03};
    cc1101_write_reg_burst(0x3F, buff, len);
    state = cc1101_read_reg(0xF5);


    HAL_Delay(10);

    state = cc1101_read_reg(0xF5);

    packets_left = cc1101_read_reg(0x3A);

    cc1101_cmd_strobe(CC1101_STX);
    state = cc1101_read_reg(0xF5);
    packets_left = cc1101_read_reg(0x3A);


    HAL_Delay(100);  // Transmission time

    packets_left = cc1101_read_reg(0x3A);


    // 4. Check MARCSTATE (0xF5) to confirm TX completion
    state = cc1101_read_reg(0xF5);

    cc1101_cmd_strobe(0x3B);


}
