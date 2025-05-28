/*
 * can.h
 *
 *  Created on: May 16, 2025
 *      Author: cyfin
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "stm32g4xx_hal.h"

extern uint32_t can_last_heard_from_master;

void CAN_Init(FDCAN_HandleTypeDef* can_selection);
HAL_StatusTypeDef CAN_SendMessage(uint32_t id, uint8_t* data, uint8_t len);

#endif /* INC_CAN_H_ */
