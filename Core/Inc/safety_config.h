/*
 * safety_config.h
 *
 *  Created on: May 27, 2025
 *      Author: cywestbrook
 */

#ifndef INC_SAFETY_CONFIG_H_
#define INC_SAFETY_CONFIG_H_

// PID & Current
#define MAX_SETPOINT 3000
#define MAX_SPIKE 3200

// Temp sensing
#define MAX_TEMP 45.0f * 100.0f // 45 degrees C in C/100
#define MIN_TEMP -20.0f * 100.0f // -20 degrees C in C/100

// Communication
#define CAN_TIMEOUT 250 // ms

#endif /* INC_SAFETY_CONFIG_H_ */
