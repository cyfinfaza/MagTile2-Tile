/*
 * mt2_types.h
 *
 *  Created on: May 16, 2025
 *      Author: cyfin
 */

#ifndef INC_MT2_TYPES_H_
#define INC_MT2_TYPES_H_

#define FAULT_REPORT_PRIORITY 0
#define MASTER_OUT_PRIORITY 1
#define TELEMETRY_PRIORITY 2

typedef enum {
	READONLY = 0,
	READWRITE = 1,
} MT2_RW_Access;

typedef union {
	uint8_t byte;
	struct {
		uint8_t alive: 1;
		uint8_t arm_ready: 1;
		uint8_t arm_active: 1;
		uint8_t coils_nonzero: 1;
		uint8_t shutdown_from_fault: 1;
	} flags;
} MT2_Slave_Status;

typedef union {
	uint8_t byte;
	struct {
		uint8_t temp_fault: 1;
		uint8_t current_spike_fault: 1;
		uint8_t vsense_fault: 1; // TODO
		uint8_t invalid_value_fault: 1;
		uint8_t communication_fault: 1;
		uint8_t address_conflict_fault: 1;
		uint8_t hv_rail_sag_fault: 1;
	} flags;
} MT2_Slave_Faults;

typedef union {
	uint8_t byte;
	struct {
		uint8_t global_arm: 1;
		uint8_t global_fault_clear: 1;
	} flags;
} MT2_Global_State;

typedef union {
	uint8_t byte;
	struct {
		uint8_t identify: 1; // TODO
		uint8_t local_fault_clear: 1;
	} flags;
} MT2_Slave_Settings;

#endif /* INC_MT2_TYPES_H_ */
