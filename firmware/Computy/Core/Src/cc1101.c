/*
 * cc1101.c
 *
 *  Created on: Mar 11, 2020
 *      Author: suleyman.eskil but the library has Mr. Ilynx
 *      https://www.freelancer.com/u/ilynx?ref_project_id=24212020
 */

#include"cc1101.h"

SPI_HandleTypeDef* hal_spi;
UART_HandleTypeDef* hal_uart;

uint16_t CS_Pin;
GPIO_TypeDef* CS_GPIO_Port;

#define WRITE_BURST             0x40
#define READ_SINGLE             0x80
#define READ_BURST              0xC0


#define BYTES_IN_RXFIFO         0x7F
#define LQI                     1
#define CRC_OK                  0x80


#define PKTSTATUS_CCA           0x10
#define PKTSTATUS_CS            0x40


#define RANDOM_OFFSET           67
#define RANDOM_MULTIPLIER       109
#define RSSI_VALID_DELAY_US     1300

//static UINT8 rnd_seed = 0;

HAL_StatusTypeDef __spi_write(uint8_t *addr, uint8_t *pData, uint16_t size){
	HAL_StatusTypeDef status;
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); //set Chip Select to Low

	status = HAL_SPI_Transmit(hal_spi, addr, 1, 0xFFFF);
	if(status==HAL_OK && pData!=NULL)
		status = HAL_SPI_Transmit(hal_spi, pData, size, 0xFFFF);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); //set Chip Select to High
	return status;

}

HAL_StatusTypeDef __spi_read(uint8_t *addr, uint8_t *pData, uint16_t size){

	HAL_StatusTypeDef status;

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); //set Chip Select to Low


	status = HAL_SPI_Transmit(hal_spi, addr, 1, 0xFFFF);
	status = HAL_SPI_Receive(hal_spi, pData, size, 0xFFFF);


	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); //set Chip Select to High

	return status;

}

void TI_write_reg(UINT8 addr, UINT8 value)
{
	__spi_write(&addr, &value, 1);
}

void TI_write_burst_reg(BYTE addr, BYTE* buffer, BYTE count)
{
	addr = (addr | WRITE_BURST);
	__spi_write(&addr, buffer, count);
}

void TI_strobe(BYTE strobe)
{
	__spi_write(&strobe, 0, 0);
}


BYTE TI_read_reg(BYTE addr)
{
	uint8_t data;
	addr= (addr | READ_SINGLE);
	__spi_read(&addr, &data, 1);
	return data;
}

BYTE TI_read_status(BYTE addr)
{
	uint8_t data;
	addr= (addr | READ_BURST);
	__spi_read(&addr, &data, 1);
	return data;
}

void TI_read_burst_reg(BYTE addr, BYTE* buffer, BYTE count)
{
	addr= (addr | READ_BURST);
	__spi_read(&addr, buffer, count);
}

BOOL TI_receive_packet(BYTE* rxBuffer, UINT8 *length)
{
	BYTE status[2];
	UINT8 packet_len;
	// This status register is safe to read since it will not be updated after
	// the packet has been received (See the CC1100 and 2500 Errata Note)
	if (TI_read_status(CCxxx0_RXBYTES) & BYTES_IN_RXFIFO)
	{
		// Read length byte
		packet_len = TI_read_reg(CCxxx0_RXFIFO);

		// Read data from RX FIFO and store in rxBuffer
		if (packet_len <= *length)
		{
			TI_read_burst_reg(CCxxx0_RXFIFO, rxBuffer, packet_len);
			*length = packet_len;

			// Read the 2 appended status bytes (status[0] = RSSI, status[1] = LQI)
			TI_read_burst_reg(CCxxx0_RXFIFO, status, 2);
			//while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0));
			//while(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0));
			// MSB of LQI is the CRC_OK bit
			return(status[LQI] & CRC_OK);
		}
		else
		{
			*length = packet_len;

			// Make sure that the radio is in IDLE state before flushing the FIFO
			// (Unless RXOFF_MODE has been changed, the radio should be in IDLE state at this point)
			TI_strobe(CCxxx0_SIDLE);

			// Flush RX FIFO
			TI_strobe(CCxxx0_SFRX);
			return(FALSE);
		}
	}
	else return(FALSE);
}

void init_serial(UART_HandleTypeDef* huart){

	hal_uart = huart;
}


void TI_send_packet(BYTE* txBuffer, UINT8 size)
{
	BYTE status;

  	TI_strobe(CCxxx0_SIDLE); //ïåðåâîäèì ìîäåì â IDLE

    TI_write_reg(CCxxx0_TXFIFO, size);

	status = TI_read_status(CCxxx0_TXBYTES);

    TI_write_burst_reg(CCxxx0_TXFIFO, txBuffer, size);

	status = TI_read_status(CCxxx0_TXBYTES);

    TI_strobe(CCxxx0_STX);
}


//For 433MHz, +10dBm
//it is also high
BYTE paTable[] = {0xc6,0xc6,0xc6,0xc6,0xc6,0xc6,0xc6,0xc6};


void TI_write_settings()
{
	TI_write_reg(0x4, 0xD3);
	TI_write_reg(0x5, 0x91);
	TI_write_reg(0x6, 0x0);
	TI_write_reg(0x7, 0x4);
	TI_write_reg(0x8, 0x5);
	TI_write_reg(0x9, 0x0);
	TI_write_reg(0xA, 0x0);
	TI_write_reg(0xB, 0x6);
	TI_write_reg(0xC, 0x23);
	TI_write_reg(0xD, 0x10);
	TI_write_reg(0xE, 0xB0);
	TI_write_reg(0xF, 0x71);
	TI_write_reg(0x10, 0xB);
	TI_write_reg(0x11, 0xF8);
	TI_write_reg(0x12, 0x2);
	TI_write_reg(0x13, 0x2);
	TI_write_reg(0x14, 0xF8);
	TI_write_reg(0x15, 0x47);
	TI_write_reg(0x16, 0x7);
	TI_write_reg(0x17, 0x30);
	TI_write_reg(0x18, 0x18);
	TI_write_reg(0x19, 0x16);
	TI_write_reg(0x1A, 0x1C);
	TI_write_reg(0x1B, 0xC7);
	TI_write_reg(0x1C, 0x0);
	TI_write_reg(0x1D, 0xB2);
	TI_write_reg(0x1E, 0x87);
	TI_write_reg(0x1F, 0x6B);
	TI_write_reg(0x20, 0xF8);
	TI_write_reg(0x21, 0x56);
	TI_write_reg(0x22, 0x10);
	TI_write_reg(0x23, 0xE9);
	TI_write_reg(0x24, 0x2A);
	TI_write_reg(0x25, 0x0);
	TI_write_reg(0x26, 0x1F);
	TI_write_reg(0x27, 0x41);
	TI_write_reg(0x28, 0x0);
}

void Power_up_reset()
{
	//Güç geldikten sonra CC1101 i Macro resetlemek için


	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	HAL_Delay(1);

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)); //CS pini LOW yaptığımızd MISO pini adres yazılmadan önce low da beklemeli
	TI_strobe(CCxxx0_SRES);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}


void TI_init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin)
{
	//UINT8 i;
	//UINT16 delay;
	BYTE status;
	hal_spi = hspi;
	CS_GPIO_Port = cs_port;
	CS_Pin = cs_pin;


	for(int i=0; i<10; i++){
	status = TI_read_status(CCxxx0_VERSION);
		  if(status!=0x14)
		  {
		  }
	}
	TI_strobe(CCxxx0_SFRX); //î÷èùàåì RX FIFO
	TI_strobe(CCxxx0_SFTX); //î÷èùàåì TX FIFO
	TI_write_settings();
	TI_write_burst_reg(CCxxx0_PATABLE, paTable, 8);//is it true

	TI_write_reg(CCxxx0_FIFOTHR, 0x07);

	TI_strobe(CCxxx0_SIDLE); //ïåðåâîäèì ìîäåì â IDLE
	TI_strobe(CCxxx0_SFRX); //î÷èùàåì RX FIFO
	TI_strobe(CCxxx0_SFTX); //î÷èùàåì TX FIFO

	//TI_strobe(CCxxx0_SRX);

	/*delay = RSSI_VALID_DELAY_US;

	do
	{
		status = TI_read_status(CCxxx0_PKTSTATUS) & (PKTSTATUS_CCA | PKTSTATUS_CS);

		if (status)
		{
			break;
		}

		ACC = 16;

		while(--ACC);

		delay -= 64;
	} while(delay > 0);

	for(i = 0; i < 16; i++)
	{
	  rnd_seed = (rnd_seed << 1) | (TI_read_status(CCxxx0_RSSI) & 0x01);
	}

	rnd_seed |= 0x0080;*/

	TI_strobe(CCxxx0_SIDLE);
}
