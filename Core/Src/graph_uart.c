/*
 * graph_uart.c
 *
 *  Created on: May 23, 2025
 *      Author: cyfin
 */

#include "graph_uart.h"
#include "stm32g4xx_hal.h"

extern uint8_t my_address;

uint8_t transmit_buffer[2];

#define UART_BUFFER_SIZE 8
#define GRAPHUART_TIMEOUT 500  // Timeout in milliseconds

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

// Declare last seen timestamps
#define X(dir) uint32_t last_seen_##dir = 0;
DIRECTION_XMACRO
#undef X

// Declare adjacency address externs
#define X(dir) extern uint8_t adj_##dir##_addr;
DIRECTION_XMACRO
#undef X

// --- Helper: clear RX error flags and drain FIFO ---
static void UART_FlushAndStart(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t len) {
	__HAL_UART_CLEAR_OREFLAG(huart);
	__HAL_UART_CLEAR_FEFLAG(huart);
	__HAL_UART_CLEAR_PEFLAG(huart);
	while (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE)) {
		volatile uint8_t trash = (uint8_t) huart->Instance->RDR;
		(void) trash;
	}
	HAL_UARTEx_ReceiveToIdle_IT(huart, buf, len);
}

// Initialization function
void GraphUART_Init(UART_HandleTypeDef *north_selection, UART_HandleTypeDef *east_selection, UART_HandleTypeDef *south_selection,
		UART_HandleTypeDef *west_selection) {
	// Assign UART handles
#define X(dir) huart_##dir = dir##_selection;
	DIRECTION_XMACRO
#undef X

	// Start receive interrupts safely
#define X(dir) UART_FlushAndStart(huart_##dir, uart_rx_buffer_##dir, UART_BUFFER_SIZE);
	DIRECTION_XMACRO
#undef X
}

// Periodic update: broadcast address and handle timeouts
void GraphUART_PeriodicUpdate() {
	transmit_buffer[0] = 0x00;
	transmit_buffer[1] = my_address;

	uint32_t now = HAL_GetTick();

#define X(dir) \
	HAL_UART_Transmit_IT(huart_##dir, transmit_buffer, 2); \
	if (now - last_seen_##dir > GRAPHUART_TIMEOUT) { \
		adj_##dir##_addr = 0; \
	}
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
			last_seen_##dir = HAL_GetTick(); \
		} \
		HAL_StatusTypeDef res = HAL_UARTEx_ReceiveToIdle_IT(huart_##dir, uart_rx_buffer_##dir, UART_BUFFER_SIZE); \
		if (res != HAL_OK) { \
			UART_FlushAndStart(huart_##dir, uart_rx_buffer_##dir, UART_BUFFER_SIZE); \
		} \
		return; \
	}
	DIRECTION_XMACRO
#undef X
}
