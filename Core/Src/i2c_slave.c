/*
 * i2c_slave.c
 *
 *  Created on: Apr 11, 2025
 *      Author: cywestbrook
 */

#include "i2c_slave.h"
#include "registry.h"
#include <string.h>

static uint8_t rx_buffer[4];
static uint8_t tx_buffer[4];
static uint8_t current_reg = 0xFF;

static I2C_HandleTypeDef* i2c_handle;

uint8_t myAddr;

extern uint8_t i2c_blink;

void I2C_Slave_Init(I2C_HandleTypeDef* hi2c) {
    i2c_handle = hi2c;
    myAddr = hi2c->Init.OwnAddress1;

    // No longer need to register manually — myAddr must be defined in registry_table[] externally

    HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode) {
    i2c_blink = !i2c_blink;
    if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, rx_buffer, 1, I2C_NEXT_FRAME);  // Expect register index
    } else if (current_reg < registry_count && registry_table[current_reg].enabled) {
        const Registry_RegConfig *reg = &registry_table[current_reg];
        memcpy(tx_buffer, reg->addr, reg->size);
        HAL_I2C_Slave_Seq_Transmit_IT(hi2c, tx_buffer, reg->size, I2C_LAST_FRAME);
    } else {
        static uint8_t dummy = 0x00;
        HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &dummy, 1, I2C_LAST_FRAME);
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (current_reg == 0xFF) {
        current_reg = rx_buffer[0];

        if (current_reg < registry_count &&
            registry_table[current_reg].enabled &&
            registry_table[current_reg].access == Registry_READWRITE) {

            HAL_I2C_Slave_Seq_Receive_IT(hi2c, rx_buffer,
                registry_table[current_reg].size, I2C_LAST_FRAME);
        }
    } else if (current_reg < registry_count &&
               registry_table[current_reg].enabled &&
               registry_table[current_reg].access == Registry_READWRITE) {

        memcpy((void *)registry_table[current_reg].addr, rx_buffer, registry_table[current_reg].size);
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
