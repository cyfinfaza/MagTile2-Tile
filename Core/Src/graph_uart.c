/*
 * graph_uart.c
 *
 *  Created on: May 23, 2025
 *      Author: cyfin
 */

#include "graph_uart.h"
#include "stm32g4xx_hal.h"

extern uint8_t my_address;

#define UART_BUFFER_SIZE 8

// Define the directions — used throughout
#define DIRECTION_XMACRO \
	X(north) \
	X(east)  \
	X(south) \
	X(west)

// Declare UART handles
#define X(dir) UART_HandleTypeDef *huart_##dir;
DIRECTION_XMACRO
#undef X

// Declare RX buffers
#define X(dir) uint8_t uart_rx_buffer_##dir[UART_BUFFER_SIZE];
DIRECTION_XMACRO
#undef X

// Declare adjacency address externs
#define X(dir) extern uint8_t adj_##dir##_addr;
DIRECTION_XMACRO
#undef X

// Initialization function
void GraphUART_Init(UART_HandleTypeDef *north_selection, UART_HandleTypeDef *east_selection, UART_HandleTypeDef *south_selection,
		UART_HandleTypeDef *west_selection) {
	// Assign UART handles
#define X(dir) huart_##dir = dir##_selection;
	DIRECTION_XMACRO
#undef X

	// Start receive interrupts
#define X(dir) HAL_UARTEx_ReceiveToIdle_IT(huart_##dir, uart_rx_buffer_##dir, UART_BUFFER_SIZE);
	DIRECTION_XMACRO
#undef X
}

// Periodic update: broadcast address
void GraphUART_PeriodicUpdate() {
	uint8_t data[2] = { 0x00, my_address };

#define X(dir) HAL_UART_Transmit_IT(huart_##dir, data, 2);
	DIRECTION_XMACRO
#undef X
}

// UART receive callback
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *incoming_huart, uint16_t size) {
#define X(dir) \
	if (incoming_huart == huart_##dir) { \
		uint8_t *uart_rx_buffer = uart_rx_buffer_##dir; \
		if (size == 2 && uart_rx_buffer[0] == 0x00) { \
			adj_##dir##_addr = uart_rx_buffer[1]; \
		} \
		HAL_UARTEx_ReceiveToIdle_IT(huart_##dir, uart_rx_buffer_##dir, UART_BUFFER_SIZE); \
		return; \
	}
	DIRECTION_XMACRO
#undef X
}
