/*
 * adc.c
 *
 *  Created on: Apr 13, 2025
 *      Author: cywestbrook
 */


#include "adc.h"
#include "stm32g4xx_hal.h"
#include <string.h>
#include "safety_config.h"
#include "mt2_types.h"

uint16_t ADC1_DMA_BUFFER[ADC1_CHANNELS*2];
uint16_t ADC2_DMA_BUFFER[ADC2_CHANNELS*2];
uint16_t ADC3_DMA_BUFFER[ADC3_CHANNELS*2];
uint16_t ADC4_DMA_BUFFER[ADC4_CHANNELS*2];
uint16_t ADC5_DMA_BUFFER[ADC5_CHANNELS*2];

// raw readings
uint16_t V_SENSE_HV = 0;
uint16_t V_SENSE_12 = 0;
uint16_t V_SENSE_5 = 0;

uint16_t TEMP_SENSE[NUM_COILS];
uint16_t COIL_CURRENT[NUM_COILS];

// calculated values
float v_sense_hv = 0;
float v_sense_12 = 0;
float v_sense_5 = 0;

uint16_t coil_current_reading[NUM_COILS];
int16_t coil_temp[NUM_COILS];

// PID
#define PID_I_CLAMP 0.1f

uint16_t coil_setpoint[NUM_COILS];
uint16_t coil_pwm_ccr[NUM_COILS];
int32_t pid_error[NUM_COILS] = {0};
int32_t pid_error_integral[NUM_COILS] = {0};
float pid_pwm_change[NUM_COILS] = {0};
float pid_pwm_output[NUM_COILS] = {0};

float Kp = 0.005f;
float Ki = 0.0005f;

extern MT2_Slave_Faults slave_faults;
extern MT2_Slave_Status slave_status;

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
	memcpy(TEMP_SENSE, buffer, sizeof(TEMP_SENSE));

	if (!slave_status.flags.arm_active && !slave_status.flags.shutdown_from_fault) {
		slave_faults.flags.temp_fault = 0; // clear temp sense fault if not armed
	}

    // calculate
	for (int i = 0; i < NUM_COILS; i++) {
		coil_temp[i] = CONVERT_TEMP_SENSE(TEMP_SENSE[i]);
		if (coil_temp[i] > MAX_TEMP || coil_temp[i] < MIN_TEMP) {
			slave_faults.flags.temp_fault = 1;
		}
	}
}

void ADC345_ProcessBuffer(uint16_t *buffer, uint8_t coil_offset) {
	// read the 3 values from the ADC buffer to the correct part of the coil_current
	memcpy(&COIL_CURRENT[coil_offset], buffer, sizeof(COIL_CURRENT[0])*3);

	// calculate measurements
	for (int i = 0; i < 3; i++) {
		coil_current_reading[i + coil_offset] = CONVERT_EA_SENSE(COIL_CURRENT[i + coil_offset]);
	}

	// run the PID controller
	PID_Solve3(coil_offset);
	PID_SetCCR3(coil_offset);
}

void HAL_ADC_HalfConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if (hadc->Instance == ADC1) {
		ADC1_ProcessBuffer(&ADC1_DMA_BUFFER[0]);
	} else if (hadc->Instance == ADC2) {
		ADC2_ProcessBuffer(&ADC2_DMA_BUFFER[0]);
	} else if (hadc->Instance == ADC3) {
		ADC345_ProcessBuffer(&ADC3_DMA_BUFFER[0], 0);
	} else if (hadc->Instance == ADC4) {
		ADC345_ProcessBuffer(&ADC4_DMA_BUFFER[0], 3);
	} else if (hadc->Instance == ADC5) {
		ADC345_ProcessBuffer(&ADC5_DMA_BUFFER[0], 6);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if (hadc->Instance == ADC1) {
		ADC1_ProcessBuffer(&ADC1_DMA_BUFFER[ADC1_CHANNELS]);
	} else if (hadc->Instance == ADC2) {
		ADC2_ProcessBuffer(&ADC2_DMA_BUFFER[ADC2_CHANNELS]);
	} else if (hadc->Instance == ADC3) {
		ADC345_ProcessBuffer(&ADC3_DMA_BUFFER[ADC3_CHANNELS], 0);
	} else if (hadc->Instance == ADC4) {
		ADC345_ProcessBuffer(&ADC4_DMA_BUFFER[ADC4_CHANNELS], 3);
	} else if (hadc->Instance == ADC5) {
		ADC345_ProcessBuffer(&ADC5_DMA_BUFFER[ADC5_CHANNELS], 6);
	}
}

void PID_SetCCR3() {
	// set the PWM output for the coils
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, coil_pwm_ccr[0]);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, coil_pwm_ccr[1]);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, coil_pwm_ccr[2]);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, coil_pwm_ccr[3]);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, coil_pwm_ccr[4]);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, coil_pwm_ccr[5]);
	__HAL_TIM_SET_COMPARE(&htim20, TIM_CHANNEL_1, coil_pwm_ccr[6]);
	__HAL_TIM_SET_COMPARE(&htim20, TIM_CHANNEL_2, coil_pwm_ccr[7]);
	__HAL_TIM_SET_COMPARE(&htim20, TIM_CHANNEL_3, coil_pwm_ccr[8]);
}

void PID_DisableAll() {
	for (int i = 0; i < 9; i++) {
		coil_pwm_ccr[i] = 0;
		coil_setpoint[i] = 0;
		pid_pwm_output[i] = 0;
		pid_error_integral[i] = 0;
	}
	PID_SetCCR3();
}

//		EA_SENSE_1 = HAL_ADC_GetValue(hadc);
//		coil_current_1 = (float) EA_SENSE_1 / 65535.0f * 3.0f / 0.12f;
//		if (current_setpoint > 3 || coil_current_1 > 3.2) {
//			TIM1->CCR1 = 0;
//			current_setpoint = 0;
//			return;
//		}
//		pid_error = current_setpoint - coil_current_1;
//		pid_error_integral += pid_error;
//		// clamp integral to +- 10
//		if (pid_error_integral > PID_I_CLAMP) {
//			pid_error_integral = PID_I_CLAMP;
//		} else if (pid_error_integral < -PID_I_CLAMP) {
//			pid_error_integral = -PID_I_CLAMP;
//		}
//		pid_pwm_change = Kp * pid_error + Ki * pid_error_integral;
//		pid_pwm_output += pid_pwm_change;
//		if (pid_pwm_output >= 0.98f) {
//			pid_pwm_output = 0.98f;
//		} else if (pid_pwm_output < 0.0f) {
//			pid_pwm_output = 0.0f;
//		}
//		coil_pwm_ccr_1 = pid_pwm_output * 1600.0f;
//		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, coil_pwm_ccr_1);

// function below implements code from above but using variables from this file

void PID_Solve3(uint8_t coil_offset) {
	if (!slave_status.flags.arm_active && !slave_status.flags.shutdown_from_fault) {
		slave_faults.flags.current_spike_fault = 0;
		slave_faults.flags.invalid_value_fault = 0;
	}
	if (slave_faults.byte || !slave_status.flags.arm_active) {
		PID_DisableAll();
		return;
	}
	for (int i = coil_offset; i < (3 + coil_offset); i++) {
		if (coil_setpoint[i] > MAX_SETPOINT) {
			slave_faults.flags.invalid_value_fault = 1;
		}
		if (coil_current_reading[i] > MAX_SPIKE) {
			slave_faults.flags.current_spike_fault = 1;
		}
		if (coil_setpoint[i] > MAX_SETPOINT || coil_current_reading[i] > MAX_SPIKE) {
			coil_pwm_ccr[i] = 0;
			coil_setpoint[i] = 0;
			return;
		}
		pid_error[i] = coil_setpoint[i] - coil_current_reading[i];
		pid_error_integral[i] += pid_error[i];
		// clamp integral to +- 10
		if (pid_error_integral[i] > PID_I_CLAMP) {
			pid_error_integral[i] = PID_I_CLAMP;
		} else if (pid_error_integral[i] < -PID_I_CLAMP) {
			pid_error_integral[i] = -PID_I_CLAMP;
		}
		pid_pwm_change[i] = Kp * (float)pid_error[i]/1000.0f + Ki * (float)pid_error_integral[i]/1000.0f;
		pid_pwm_output[i] += pid_pwm_change[i];
		if (pid_pwm_output[i] >= 0.98f) {
			pid_pwm_output[i] = 0.98f;
		} else if (pid_pwm_output[i] < 0.0f) {
			pid_pwm_output[i] = 0.0f;
		}
		coil_pwm_ccr[i] = pid_pwm_output[i] * 1600.0f;
	}
}
