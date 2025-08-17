/*
 * telemetry.h
 *
 *  Created on: May 16, 2025
 *      Author: cyfin
 */

#ifndef INC_TELEMETRY_H_
#define INC_TELEMETRY_H_

#define TELEMETRY_INTERVAL 200 // ms
#define FAULT_REPORT_INTERVAL 10 // ms

void Telemetry_Init(void);

void Telemetry_Loop(void);

#endif /* INC_TELEMETRY_H_ */
