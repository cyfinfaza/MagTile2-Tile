/*
 * telemetry.c
 *
 *  Created on: May 17, 2025
 *      Author: cyfin
 */

#include "telemetry.h"
#include "can.h"
#include "registry.h"
#include <string.h>
#include "mt2_types.h"

extern uint8_t my_address;
extern MT2_Slave_Status slave_faults;
extern MT2_Global_State global_state;

uint32_t on_reg_index = 0;
uint32_t last_start_ts = 0;
uint32_t last_start_fault_alert = 0;

#define SLAVE_FAULTS_REG 0x05

void Telemetry_Init(void) {
	// initialize telemetry
	on_reg_index = 0;
	last_start_ts = HAL_GetTick();
}

int Telemetry_SendRegister(uint8_t reg, uint8_t priority) {
	uint8_t reg_len = registry_table[reg].size;
	uint8_t payload[5]; // 1 byte for index, 4 bytes for data
	payload[0] = reg;
	if (reg_len > 4) reg_len = 4; // limit to 4 bytes
	memcpy(&payload[1], registry_table[reg].addr, reg_len);
	return CAN_SendMessage((priority << 8) | my_address, payload, reg_len + 1);
}

void Telemetry_Loop(void) {
	if (slave_faults.byte && global_state.flags.global_arm) {
		if (HAL_GetTick() - last_start_fault_alert > FAULT_REPORT_INTERVAL) {
			last_start_fault_alert = HAL_GetTick();
			Telemetry_SendRegister(SLAVE_FAULTS_REG, FAULT_REPORT_PRIORITY);
		}
	} else {
		last_start_fault_alert = 0; // reset if no faults
	}
	if (on_reg_index < registry_count) {
		uint8_t result = Telemetry_SendRegister(on_reg_index, TELEMETRY_PRIORITY);
		if (result == 0) {
			// find next enabled register
			on_reg_index++;
			while (on_reg_index < registry_count && !registry_table[on_reg_index].enabled) {
				on_reg_index++;
			}
		}
	} else if (HAL_GetTick() - last_start_ts > TELEMETRY_INTERVAL) {
		// reset on timeout
		on_reg_index = 0;
		last_start_ts = HAL_GetTick();
	}
}
