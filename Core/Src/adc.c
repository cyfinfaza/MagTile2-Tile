/*
 * adc.c
 *
 *  Created on: Apr 13, 2025
 *      Author: cywestbrook
 */


#include "adc.h"
#include "stm32g4xx_hal.h"

uint16_t ADC1_DMA_BUFFER[ADC1_CHANNELS*2];
uint16_t ADC2_DMA_BUFFER[ADC2_CHANNELS*2];
uint16_t ADC3_DMA_BUFFER[ADC3_CHANNELS*2];
uint16_t ADC4_DMA_BUFFER[ADC4_CHANNELS*2];
uint16_t ADC5_DMA_BUFFER[ADC5_CHANNELS*2];

// raw readings
uint16_t V_SENSE_HV = 0;
uint16_t V_SENSE_12 = 0;
uint16_t V_SENSE_5 = 0;

uint16_t TEMP_SENSE_1 = 0;
uint16_t TEMP_SENSE_2 = 0;
uint16_t TEMP_SENSE_3 = 0;
uint16_t TEMP_SENSE_4 = 0;
uint16_t TEMP_SENSE_5 = 0;
uint16_t TEMP_SENSE_6 = 0;
uint16_t TEMP_SENSE_7 = 0;
uint16_t TEMP_SENSE_8 = 0;
uint16_t TEMP_SENSE_9 = 0;

uint16_t COIL_CURRENT_1 = 0;
uint16_t COIL_CURRENT_2 = 0;
uint16_t COIL_CURRENT_3 = 0;
uint16_t COIL_CURRENT_4 = 0;
uint16_t COIL_CURRENT_5 = 0;
uint16_t COIL_CURRENT_6 = 0;
uint16_t COIL_CURRENT_7 = 0;
uint16_t COIL_CURRENT_8 = 0;
uint16_t COIL_CURRENT_9 = 0;

// calculated values
float v_sense_hv = 0;
float v_sense_12 = 0;
float v_sense_5 = 0;

uint16_t coil_1_current_reading = 0;
uint16_t coil_2_current_reading = 0;
uint16_t coil_3_current_reading = 0;
uint16_t coil_4_current_reading = 0;
uint16_t coil_5_current_reading = 0;
uint16_t coil_6_current_reading = 0;
uint16_t coil_7_current_reading = 0;
uint16_t coil_8_current_reading = 0;
uint16_t coil_9_current_reading = 0;

int16_t coil_1_temp = 0;
int16_t coil_2_temp = 0;
int16_t coil_3_temp = 0;
int16_t coil_4_temp = 0;
int16_t coil_5_temp = 0;
int16_t coil_6_temp = 0;
int16_t coil_7_temp = 0;
int16_t coil_8_temp = 0;
int16_t coil_9_temp = 0;

// temp sense conversion deg C/100 = ((ADC/2^16)*3 + 0.4) * 0.0195 * 100
#define CONVERT_TEMP_SENSE(x) ((x * 3.3f / 65535.0f - 0.5f) / 0.010f * 100.0f)

// ea conversion mA = x / 2^16 * 3 / 0.12 * 1000
#define CONVERT_EA_SENSE(x) ((x * 3.0f / 65535.0f) / 0.12f * 1000.0f)

void ADC_Init(ADC_HandleTypeDef* hadc1, ADC_HandleTypeDef* hadc2, ADC_HandleTypeDef* hadc3, ADC_HandleTypeDef* hadc4, ADC_HandleTypeDef* hadc5) {
	// calibrate the ADCs
	HAL_ADCEx_Calibration_Start(hadc1, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(hadc2, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(hadc3, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(hadc4, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(hadc5, ADC_SINGLE_ENDED);

	// start the ADCs in DMA mode
	HAL_ADC_Start_DMA(hadc1, (uint32_t*)&ADC1_DMA_BUFFER, ADC1_CHANNELS*2);
	HAL_ADC_Start_DMA(hadc2, (uint32_t*)&ADC2_DMA_BUFFER, ADC2_CHANNELS*2);
	HAL_ADC_Start_DMA(hadc3, (uint32_t*)&ADC3_DMA_BUFFER, ADC3_CHANNELS*2);
	HAL_ADC_Start_DMA(hadc4, (uint32_t*)&ADC4_DMA_BUFFER, ADC4_CHANNELS*2);
	HAL_ADC_Start_DMA(hadc5, (uint32_t*)&ADC5_DMA_BUFFER, ADC5_CHANNELS*2);
}

void ADC1_ProcessBuffer(uint16_t* buffer) {
	// read the values from the ADC buffer
	V_SENSE_HV = buffer[0];
	V_SENSE_12 = buffer[1];
	V_SENSE_5 = buffer[2];

	// calculate measurements
	v_sense_12 = V_SENSE_12 * 0.0080566406;
	v_sense_5 = V_SENSE_5 * 0.0014648438;
	v_sense_hv = V_SENSE_HV * 0.0194091797;
}

void ADC2_ProcessBuffer(uint16_t *buffer) {
	// read the values from the ADC buffer
	TEMP_SENSE_1 = buffer[0];
	TEMP_SENSE_2 = buffer[1];
	TEMP_SENSE_3 = buffer[2];
    TEMP_SENSE_4 = buffer[3];
    TEMP_SENSE_5 = buffer[4];
    TEMP_SENSE_6 = buffer[5];
    TEMP_SENSE_7 = buffer[6];
    TEMP_SENSE_8 = buffer[7];
    TEMP_SENSE_9 = buffer[8];

    // calculate
    coil_1_temp = CONVERT_TEMP_SENSE(TEMP_SENSE_1);
    coil_2_temp = CONVERT_TEMP_SENSE(TEMP_SENSE_2);
    coil_3_temp = CONVERT_TEMP_SENSE(TEMP_SENSE_3);
    coil_4_temp = CONVERT_TEMP_SENSE(TEMP_SENSE_4);
    coil_5_temp = CONVERT_TEMP_SENSE(TEMP_SENSE_5);
    coil_6_temp = CONVERT_TEMP_SENSE(TEMP_SENSE_6);
    coil_7_temp = CONVERT_TEMP_SENSE(TEMP_SENSE_7);
    coil_8_temp = CONVERT_TEMP_SENSE(TEMP_SENSE_8);
    coil_9_temp = CONVERT_TEMP_SENSE(TEMP_SENSE_9);
}

void ADC3_ProcessBuffer(uint16_t *buffer) {
	// read the values from the ADC buffer
	COIL_CURRENT_1 = buffer[0];
	COIL_CURRENT_2 = buffer[1];
	COIL_CURRENT_3 = buffer[2];

	// calculate measurements
	coil_1_current_reading = CONVERT_EA_SENSE(COIL_CURRENT_1);
	coil_2_current_reading = CONVERT_EA_SENSE(COIL_CURRENT_2);
	coil_3_current_reading = CONVERT_EA_SENSE(COIL_CURRENT_3);
}

void ADC4_ProcessBuffer(uint16_t *buffer) {
	// read the values from the ADC buffer
	COIL_CURRENT_4 = buffer[0];
	COIL_CURRENT_5 = buffer[1];
	COIL_CURRENT_6 = buffer[2];

	// calculate measurements
	coil_4_current_reading = CONVERT_EA_SENSE(COIL_CURRENT_4);
	coil_5_current_reading = CONVERT_EA_SENSE(COIL_CURRENT_5);
	coil_6_current_reading = CONVERT_EA_SENSE(COIL_CURRENT_6);
}

void ADC5_ProcessBuffer(uint16_t *buffer) {
	// read the values from the ADC buffer
	COIL_CURRENT_7 = buffer[0];
	COIL_CURRENT_8 = buffer[1];
	COIL_CURRENT_9 = buffer[2];

	// calculate measurements
	coil_7_current_reading = CONVERT_EA_SENSE(COIL_CURRENT_7);
	coil_8_current_reading = CONVERT_EA_SENSE(COIL_CURRENT_8);
	coil_9_current_reading = CONVERT_EA_SENSE(COIL_CURRENT_9);
}

void HAL_ADC_HalfConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if (hadc->Instance == ADC1) {
		ADC1_ProcessBuffer(&ADC1_DMA_BUFFER[0]);
	} else if (hadc->Instance == ADC2) {
		ADC2_ProcessBuffer(&ADC2_DMA_BUFFER[0]);
	} else if (hadc->Instance == ADC3) {
		ADC3_ProcessBuffer(&ADC3_DMA_BUFFER[0]);
	} else if (hadc->Instance == ADC4) {
		ADC4_ProcessBuffer(&ADC4_DMA_BUFFER[0]);
	} else if (hadc->Instance == ADC5) {
		ADC5_ProcessBuffer(&ADC5_DMA_BUFFER[0]);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if (hadc->Instance == ADC1) {
		ADC1_ProcessBuffer(&ADC1_DMA_BUFFER[ADC1_CHANNELS]);
	} else if (hadc->Instance == ADC2) {
		ADC2_ProcessBuffer(&ADC2_DMA_BUFFER[ADC2_CHANNELS]);
	} else if (hadc->Instance == ADC3) {
		ADC3_ProcessBuffer(&ADC3_DMA_BUFFER[ADC3_CHANNELS]);
	} else if (hadc->Instance == ADC4) {
		ADC4_ProcessBuffer(&ADC4_DMA_BUFFER[ADC4_CHANNELS]);
	} else if (hadc->Instance == ADC5) {
		ADC5_ProcessBuffer(&ADC5_DMA_BUFFER[ADC5_CHANNELS]);
	}
}
