/*
 * ibus.c
 *
 *  Created on: Aug 24, 2025
 *      Author: gregor
 */
#include "ibus.h"


void parse_ibus(uint8_t* circular_buffer, uint16_t* channels){

	int start = 0;
	for(int i = 0; i < 32; i++){
		if(circular_buffer[i] == 0x20 && circular_buffer[(i + 1) % 32] == 0x40){
			start = i;
			break;
		}
	}

	uint16_t checksum_calculated = 0xFFFF;
	for(int i = start; i < 30 + start; i++){
		checksum_calculated -= circular_buffer[i % 32];
	}

	uint16_t checksum_received = (((uint16_t) circular_buffer[(31 + start) % 32]) << 8) + circular_buffer[(30 + start) % 32];

	if(checksum_received == checksum_calculated){
		for (int i = 0; i < 14; i++){
			channels[i] = (((uint16_t) circular_buffer[(start + 2*(i+1) + 1) % 32]) << 8) + circular_buffer[(start + 2*(i+1) + 0) % 32];
		}
	}

}
