/*
 * i2c_slave.h
 *
 *  Created on: Apr 11, 2025
 *      Author: cywestbrook
 */

#ifndef INC_I2C_SLAVE_H_
#define INC_I2C_SLAVE_H_

#include "stm32g4xx_hal.h" // Change to your STM32 family
#include <stdint.h>
#include <stdbool.h>

#define MAX_REGISTERS 64

typedef enum {
	READONLY = 0,
	READWRITE = 1,
} I2C_RW_Access;

typedef struct {
    uint8_t size : 3;     // 1 to 4 bytes
    I2C_RW_Access access : 1;   // 0 = RO, 1 = RW
    uint8_t enabled : 1;  // 1 = valid, 0 = NACK
    void* mem_ptr;        // memory address for register data
} I2C_Register;

extern I2C_Register i2c_register_map[MAX_REGISTERS];

void I2C_Slave_Init(I2C_HandleTypeDef* hi2c);
void I2C_Register_Init(uint8_t reg_addr, uint8_t size, I2C_RW_Access access, void *mem_ptr);
void I2C_Event_Handler(I2C_HandleTypeDef *hi2c);
void I2C_Error_Handler(I2C_HandleTypeDef *hi2c);

#endif /* INC_I2C_SLAVE_H_ */
