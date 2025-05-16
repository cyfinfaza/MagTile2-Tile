/*
 * i2c_slave.c
 *
 *  Created on: Apr 11, 2025
 *      Author: cywestbrook
 */

#include "i2c_slave.h"
#include <string.h>

I2C_Register i2c_register_map[MAX_REGISTERS];

static uint8_t rx_buffer[4];
static uint8_t tx_buffer[4];
static uint8_t current_reg = 0xFF;

static I2C_HandleTypeDef* i2c_handle;

uint8_t myAddr;

extern uint8_t i2c_blink;

void I2C_Slave_Init(I2C_HandleTypeDef* hi2c) {
    i2c_handle = hi2c;
    myAddr = hi2c->Init.OwnAddress1;
    // register 0 always just returns my address
    I2C_RegisterInit(0, 1, READONLY, &myAddr);
    HAL_I2C_EnableListen_IT(hi2c);
}

void I2C_RegisterInit(uint8_t reg_addr, uint8_t size, I2C_RW_Access access,
		void *mem_ptr) {
	if (reg_addr < MAX_REGISTERS) {
		i2c_register_map[reg_addr].size = size;
		i2c_register_map[reg_addr].access = access;
		i2c_register_map[reg_addr].enabled = 1;
		i2c_register_map[reg_addr].mem_ptr = mem_ptr;
	}
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode) {
    i2c_blink = !i2c_blink;
	if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, rx_buffer, 1, I2C_NEXT_FRAME);  // Expect register index
    } else if (current_reg < MAX_REGISTERS && i2c_register_map[current_reg].enabled) {
        memcpy(tx_buffer, i2c_register_map[current_reg].mem_ptr, i2c_register_map[current_reg].size);
        HAL_I2C_Slave_Seq_Transmit_IT(hi2c, tx_buffer, i2c_register_map[current_reg].size, I2C_LAST_FRAME);
    } else {
        static uint8_t dummy = 0x00;
        HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &dummy, 1, I2C_LAST_FRAME);
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (current_reg == 0xFF) {
        current_reg = rx_buffer[0];

        // If the register is writable, expect its value next
        if (current_reg < MAX_REGISTERS &&
            i2c_register_map[current_reg].enabled &&
            i2c_register_map[current_reg].access) {

            HAL_I2C_Slave_Seq_Receive_IT(hi2c, rx_buffer,
                i2c_register_map[current_reg].size, I2C_LAST_FRAME);
        }
    } else if (current_reg < MAX_REGISTERS &&
               i2c_register_map[current_reg].enabled &&
               i2c_register_map[current_reg].access) {

        memcpy(i2c_register_map[current_reg].mem_ptr, rx_buffer, i2c_register_map[current_reg].size);
        current_reg = 0xFF;
    }
}


void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    current_reg = 0xFF;
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
    HAL_I2C_EnableListen_IT(hi2c);
}

void I2C_Event_Handler(I2C_HandleTypeDef *hi2c) {
    // Managed by HAL callbacks
}

void I2C_Error_Handler(I2C_HandleTypeDef *hi2c) {
    HAL_I2C_EnableListen_IT(hi2c);
}
