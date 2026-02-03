/*
 * graph_uart.c
 *
 *  Created on: May 23, 2025
 *      Author: cyfin
 */

#include "graph_uart.h"
#include "stm32g4xx_hal.h"
#include "mt2_types.h"

extern uint8_t my_address;
extern MT2_Slave_Status slave_status;
extern MT2_Slave_Faults slave_faults;

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
	if(!slave_faults.flags.communication_fault || slave_status.flags.shutdown_from_fault) \
		HAL_UART_Transmit_IT(huart_##dir, transmit_buffer, 2); \
	if (now - last_seen_##dir > GRAPHUART_TIMEOUT) { \
		adj_##dir##_addr = 0; \
	}
	DIRECTION_XMACRO
#undef X
}

// One-call jump into the STM32G47x ROM bootloader (System Memory).
// Works without touching option bytes or BOOT0 pin.
//
// Call this from thread context (not inside an IRQ).
// Does not return.
//
// Written by ChatGPT
__attribute__((noreturn))
void JumpToBootloader_G4(void)
{
    typedef void (*pFunction)(void);
    const uint32_t SYSMEM_BASE = 0x1FFF0000UL; // STM32G4 system memory base (AN2606)

    __disable_irq();

    /* Stop SysTick */
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    /* Deinit peripherals and clocks back to reset state */
    HAL_RCC_DeInit();
    HAL_DeInit();

    /* Fully mask & clear NVIC to avoid stray IRQs after VTOR switch */
    for (uint32_t i = 0; i < 8; i++) {              // 8 words = up to 256 IRQs on CM4
        NVIC->ICER[i] = 0xFFFFFFFFUL;               // disable
        NVIC->ICPR[i] = 0xFFFFFFFFUL;               // clear pending
    }
    __DSB(); __ISB();

    /* Optional: remap System Memory at 0x00000000 (supported on G4) */
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();

    /* Also point VTOR at System Memory (belt-and-suspenders) */
    SCB->VTOR = SYSMEM_BASE;

    /* Load MSP and the bootloader entry (Reset_Handler) from System Memory */
    uint32_t boot_msp   = *(__IO uint32_t *)(SYSMEM_BASE + 0x00U);
    uint32_t boot_entry = *(__IO uint32_t *)(SYSMEM_BASE + 0x04U);
    pFunction Boot = (pFunction)boot_entry;

    /* Set the Main Stack Pointer and jump */
    __set_MSP(boot_msp);
    __DSB(); __ISB();
    Boot();

    while (1) { /* never returns */ }
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
		else if (size == 1 && uart_rx_buffer[0] == 0x7F && !slave_status.flags.arm_active) { \
			JumpToBootloader_G4(); \
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
