/*
 * can.c
 *
 *  Created on: May 16, 2025
 *      Author: cyfin
 */

#include "can.h"
#include "registry.h"
#include <stdint.h>
#include <string.h>

FDCAN_HandleTypeDef *hfdcan;

extern uint8_t can_blink;

extern uint8_t my_address;

uint32_t can_last_heard_from_master = 0;

void CAN_Init(FDCAN_HandleTypeDef *can_selection) {
	hfdcan = can_selection;
	HAL_FDCAN_Start(hfdcan);
	HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	FDCAN_FilterTypeDef sFilterConfig;
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x000;        // Accept everything
	sFilterConfig.FilterID2 = 0x000;
	HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig);
}

HAL_StatusTypeDef CAN_SendMessage(uint32_t id, uint8_t *data, uint8_t len) {
	FDCAN_TxHeaderTypeDef TxHeader;
	TxHeader.Identifier = id;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;
	TxHeader.FDFormat = FDCAN_FD_CAN;           // Enable CAN FD
	TxHeader.BitRateSwitch = FDCAN_BRS_ON; // Enable faster bitrate during data phase
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = len;
	return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *which_fdcan, uint32_t RxFifo0ITs) {
	if (which_fdcan == hfdcan) {
		FDCAN_RxHeaderTypeDef rx_header;
		uint8_t rx_data[64];
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data)
				!= HAL_OK) {
//			Error_Handler();
		}
		uint8_t addr = rx_header.Identifier & 0xFF;
		if ((addr != my_address) && (addr != 0)) {
			return; // not for me
		}
		can_blink = 1;
		can_last_heard_from_master = HAL_GetTick();
		uint8_t reg = rx_data[0];
		uint8_t len = rx_header.DataLength - 1;
		if (len > 4) {
			return; // too long
		}
		// check if the register is enabled and writable
		if (reg < registry_count && registry_table[reg].enabled
				&& registry_table[reg].access == Registry_READWRITE) {
			// copy the data to the register
			memcpy((void *)registry_table[reg].addr, &rx_data[1], len);
		}
	}
}
