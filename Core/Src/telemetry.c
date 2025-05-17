/*
 * telemetry.c
 *
 *  Created on: May 17, 2025
 *      Author: cyfin
 */

#include "telemetry.h"
#include "can.h"
#include "registry.h"

extern uint8_t my_address;

uint32_t on_reg_index = 0;
uint32_t last_start_ts = 0;

void Telemetry_Init(void) {
	// initialize telemetry
	on_reg_index = 0;
	last_start_ts = HAL_GetTick();
}

void Telemetry_Loop(void) {
	if (on_reg_index < registry_count) {
		uint8_t reg_len = registry_table[on_reg_index].size;
		uint8_t payload[5]; // 1 byte for index, 4 bytes for data
		payload[0] = on_reg_index;
		if (reg_len > 4) reg_len = 4; // limit to 4 bytes
		memcpy(&payload[1], registry_table[on_reg_index].addr, reg_len);
		uint8_t result = CAN_SendMessage((2 << 8) | my_address, payload, reg_len + 1);
		if (result == 0) {
			// find next enabled register
			on_reg_index++;
			while (on_reg_index < registry_count && !registry_table[on_reg_index].enabled) {
				on_reg_index++;
			}
		}
	} else if (HAL_GetTick() - last_start_ts > MAX_INTERVAL) {
		// reset on timeout
		on_reg_index = 0;
		last_start_ts = HAL_GetTick();
	}
}
