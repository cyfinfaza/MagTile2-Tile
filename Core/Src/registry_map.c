/*
 * registry_map.c
 *
 *  Created on: May 16, 2025
 *      Author: cyfin
 */

#include "registry.h"
#include "mt2_types.h"
#include "adc.h"

extern MT2_Slave_Status slave_status;
extern MT2_Slave_Faults slave_faults;
extern MT2_Global_State global_state;
extern MT2_Slave_Settings slave_settings;

extern float v_sense_5;
extern float v_sense_12;
extern float v_sense_hv;
extern uint16_t mcu_temp;

extern uint8_t adj_west_addr;
extern uint8_t adj_north_addr;
extern uint8_t adj_east_addr;
extern uint8_t adj_south_addr;

#define REGISTRY_COUNT 255


// Manual entries
const Registry_RegConfig registry_table[REGISTRY_COUNT] = {
	Registry_DEFINE(0x04, &slave_status, Registry_READONLY),
	Registry_DEFINE(0x05, &slave_faults, Registry_READONLY),
	Registry_DEFINE(0x06, &global_state, Registry_READWRITE),
	Registry_DEFINE(0x07, &slave_settings, Registry_READWRITE),

	Registry_DEFINE(0x08, &v_sense_5, Registry_READONLY),
	Registry_DEFINE(0x09, &v_sense_12, Registry_READONLY),
	Registry_DEFINE(0x0A, &v_sense_hv, Registry_READONLY),
	Registry_DEFINE(0x0B, &mcu_temp, Registry_READONLY),

	Registry_DEFINE(0x0C, &adj_west_addr, Registry_READONLY),
	Registry_DEFINE(0x0D, &adj_north_addr, Registry_READONLY),
	Registry_DEFINE(0x0E, &adj_east_addr, Registry_READONLY),
	Registry_DEFINE(0x0F, &adj_south_addr, Registry_READONLY),

	Registry_DEFINE(0x10, &coil_setpoint[0], Registry_READWRITE),
	Registry_DEFINE(0x11, &coil_setpoint[1], Registry_READWRITE),
	Registry_DEFINE(0x12, &coil_setpoint[2], Registry_READWRITE),
	Registry_DEFINE(0x13, &coil_setpoint[3], Registry_READWRITE),
	Registry_DEFINE(0x14, &coil_setpoint[4], Registry_READWRITE),
	Registry_DEFINE(0x15, &coil_setpoint[5], Registry_READWRITE),
	Registry_DEFINE(0x16, &coil_setpoint[6], Registry_READWRITE),
	Registry_DEFINE(0x17, &coil_setpoint[7], Registry_READWRITE),
	Registry_DEFINE(0x18, &coil_setpoint[8], Registry_READWRITE),
	Registry_DEFINE(0x20, &coil_current_reading[0], Registry_READONLY),
	Registry_DEFINE(0x21, &coil_current_reading[1], Registry_READONLY),
	Registry_DEFINE(0x22, &coil_current_reading[2], Registry_READONLY),
	Registry_DEFINE(0x23, &coil_current_reading[3], Registry_READONLY),
	Registry_DEFINE(0x24, &coil_current_reading[4], Registry_READONLY),
	Registry_DEFINE(0x25, &coil_current_reading[5], Registry_READONLY),
	Registry_DEFINE(0x26, &coil_current_reading[6], Registry_READONLY),
	Registry_DEFINE(0x27, &coil_current_reading[7], Registry_READONLY),
	Registry_DEFINE(0x28, &coil_current_reading[8], Registry_READONLY),
	Registry_DEFINE(0x30, &coil_temp[0], Registry_READONLY),
	Registry_DEFINE(0x31, &coil_temp[1], Registry_READONLY),
	Registry_DEFINE(0x32, &coil_temp[2], Registry_READONLY),
	Registry_DEFINE(0x33, &coil_temp[3], Registry_READONLY),
	Registry_DEFINE(0x34, &coil_temp[4], Registry_READONLY),
	Registry_DEFINE(0x35, &coil_temp[5], Registry_READONLY),
	Registry_DEFINE(0x36, &coil_temp[6], Registry_READONLY),
	Registry_DEFINE(0x37, &coil_temp[7], Registry_READONLY),
	Registry_DEFINE(0x38, &coil_temp[8], Registry_READONLY),
};

const uint32_t registry_count = REGISTRY_COUNT;
