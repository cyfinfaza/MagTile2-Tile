/*
 * graph_uart.h
 *
 *  Created on: May 23, 2025
 *      Author: cyfin
 */

#ifndef INC_GRAPH_UART_H_
#define INC_GRAPH_UART_H_

#include "stm32g4xx_hal.h"

void GraphUART_Init(UART_HandleTypeDef *north_selection, UART_HandleTypeDef *east_selection, UART_HandleTypeDef *south_selection,
		UART_HandleTypeDef *west_selection);

void GraphUART_PeriodicUpdate(void);

void JumpToBootloader_G4(void);

#endif /* INC_GRAPH_UART_H_ */
