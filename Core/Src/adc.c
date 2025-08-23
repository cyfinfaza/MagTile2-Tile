/*
 * adc.c
 *
 *  Created on: Apr 13, 2025
 *      Author: cywestbrook
 */


#include "adc.h"
#include "stm32g4xx_hal.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "safety_config.h"
#include "mt2_types.h"
#include "stm32g4xx_hal_adc_ex.h"

uint16_t ADC1_DMA_BUFFER[ADC1_CHANNELS*2];
uint16_t ADC2_DMA_BUFFER[ADC2_CHANNELS*2];
uint16_t ADC3_DMA_BUFFER[ADC3_CHANNELS*2];
uint16_t ADC4_DMA_BUFFER[ADC4_CHANNELS*2];
uint16_t ADC5_DMA_BUFFER[ADC5_CHANNELS*2];

// raw readings
uint16_t V_SENSE_HV = 0;
uint16_t V_SENSE_12 = 0;
uint16_t V_SENSE_5 = 0;
uint16_t INTERNAL_TEMP_SENSE = 0; // Internal temperature sensor reading

uint16_t TEMP_SENSE[NUM_COILS];
uint16_t COIL_CURRENT[NUM_COILS];

// calculated values
float v_sense_hv = 0;
float v_sense_12 = 0;
float v_sense_5 = 0;

extern float master_v_sense_hv;
uint32_t hv_sagging_since = 0; // timestamp for when HV started sagging

uint16_t mcu_temp;

float master_v_sense_hv = 0; // Master controller HV sense voltage

uint16_t coil_current_reading[NUM_COILS];
float coil_temp_from_resistance[NUM_COILS];
float temp_sensor_reading[NUM_COILS];
int16_t coil_temp[NUM_COILS];

extern int adc3_loop_counter;
extern int adc4_loop_counter;
extern int adc5_loop_counter;

#define CLAMP(_value, _min, _max) \
	((_value) < (_min) ? (_min) : ((_value) > (_max) ? (_max) : (_value)))

#define MAX(_val1, _val2) \
	((_val1) > (_val2) ? (_val1) : (_val2))

#define MIN(_val1, _val2) \
	((_val1) < (_val2) ? (_val1) : (_val2))


// PID
#define PID_I_CLAMP 0.1f

uint16_t coil_setpoint[NUM_COILS];
uint16_t coil_pwm_ccr[NUM_COILS];
int32_t pid_error[NUM_COILS] = {0};
int32_t pid_error_integral[NUM_COILS] = {0};
float pid_pwm_change[NUM_COILS] = {0};
float pid_pwm_output[NUM_COILS] = {0};

float Kp = 0.005f; // Real Kp (updated in real time)
float Ki = 0.0005f; // Real Ki (updated in real time)

float K_nominal_at_vin = 34.0f; // Test voltage for nominal Kp/Ki
float Kp_nominal = 0.005f; // Nominal Kp at test voltage
float Ki_nominal = 0.0005f; // Nominal Ki at test voltage

#define PWM_PERIOD 1600.0f


// Power Safety
#define R_COIL_ASSUMED 4
#define FET_MAX_RMS_CURRENT 3.0f // Max RMS current for FETs
#define GATE_DRIVER_MAX_DUTY_CYCLE 0.98f
float max_allowed_duty_cycle = 0;


#define D_MAX_LUT_SIZE 2000
#define D_MAX_LUT_MAX_VIN 52.0f
float d_max_lut[D_MAX_LUT_SIZE];
float get_max_duty_cycle_for_vin(float vin) {
	if (vin < 0.0f || vin > D_MAX_LUT_MAX_VIN) {
		return 0;
	}
	unsigned int index = (uint16_t)(vin / D_MAX_LUT_MAX_VIN * (D_MAX_LUT_SIZE - 1));
	return d_max_lut[index];
}
void populate_d_max_lut() {
	for (unsigned int i = 0; i < D_MAX_LUT_SIZE; i++) {
		float vin = (float)i / (D_MAX_LUT_SIZE - 1) * D_MAX_LUT_MAX_VIN;
		float max_d = powf(R_COIL_ASSUMED * FET_MAX_RMS_CURRENT / vin, 2.0f/3.0f);
		d_max_lut[i] = CLAMP(max_d, 0.0f, GATE_DRIVER_MAX_DUTY_CYCLE);
	}
}

float coil_estimated_resistance[NUM_COILS] = {0};
int16_t coil_estimated_resistance_report[NUM_COILS] = {0};


extern MT2_Slave_Faults slave_faults;
extern MT2_Slave_Status slave_status;

#define VREF 3.0f

// temp sense conversion deg C/100 = ((ADC/2^16)*3 + 0.4) * 0.0195 * 100
#define CONVERT_TEMP_SENSE(_x) ((_x * 3.3f / 65535.0f - 0.5f) / 0.010f * 100.0f)

// ea conversion mA = x / 2^16 * 3 / 0.12 * 1000
#define CONVERT_EA_SENSE(_x) ((_x * 3.0f / 65535.0f) / 0.12f * 1000.0f)

#define CONVERT_VSENSE(_x, _r_top, _r_bottom, _bit_depth) \
	(_x / _bit_depth * VREF / (_r_bottom / (_r_top + _r_bottom)))


void ADC_Init(ADC_HandleTypeDef* hadc1, ADC_HandleTypeDef* hadc2, ADC_HandleTypeDef* hadc3, ADC_HandleTypeDef* hadc4, ADC_HandleTypeDef* hadc5) {
	// calibrate the ADCs
	HAL_ADCEx_Calibration_Start(hadc1, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(hadc2, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(hadc3, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(hadc4, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(hadc5, ADC_SINGLE_ENDED);

	populate_d_max_lut(); // populate the max duty cycle lookup table
	hv_sagging_since = HAL_GetTick(); // initialize hv sag timer

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
	INTERNAL_TEMP_SENSE = buffer[3];

	// calculate measurements
	v_sense_12 = CONVERT_VSENSE(V_SENSE_12, 10.0f, 1.0f, 65535.0f);
	v_sense_5 = CONVERT_VSENSE(V_SENSE_5, 10.0f, 10.0f, 65535.0f);
	v_sense_hv = CONVERT_VSENSE(V_SENSE_HV, 51.0f, 2.0f, 65535.0f);
	mcu_temp = __HAL_ADC_CALC_TEMPERATURE(VREF*1000.0f, INTERNAL_TEMP_SENSE>>4, ADC_RESOLUTION_12B);

	if (!slave_status.flags.arm_active && !slave_status.flags.shutdown_from_fault) {
		slave_faults.flags.hv_rail_sag_fault = 0; // clear temp sense fault if not armed
	}

	if (master_v_sense_hv - v_sense_hv < MAX_HV_SAG) { // if HV is not sagging, reset the timer
		hv_sagging_since = HAL_GetTick();
	} else if (HAL_GetTick() - hv_sagging_since > MAX_HV_SAG_TIME) { // if HV has been sagging for too long, set fault
		slave_faults.flags.hv_rail_sag_fault = 1;
	}

	// set Ki/Kp based on input voltage
	float compensation_factor = CLAMP(K_nominal_at_vin / v_sense_hv, 0.5f, 2.0f);
	Kp = Kp_nominal * compensation_factor;
	Ki = Ki_nominal * compensation_factor;

	// determine max allowed duty cycle based on input voltage
	max_allowed_duty_cycle = get_max_duty_cycle_for_vin(v_sense_hv);

}

void ADC2_ProcessBuffer(uint16_t *buffer) {
	// read the values from the ADC buffer
	memcpy(TEMP_SENSE, buffer, sizeof(TEMP_SENSE));

	if (!slave_status.flags.arm_active && !slave_status.flags.shutdown_from_fault) {
		slave_faults.flags.temp_fault = 0; // clear temp sense fault if not armed
	}

    // calculate
	for (int i = 0; i < NUM_COILS; i++) {
		temp_sensor_reading[i] = CONVERT_TEMP_SENSE(TEMP_SENSE[i]) / 100.0f; // convert to deg C
		if (coil_current_reading[i] > 200 && coil_setpoint[i] > 200) {
			float temp_from_resistance_reading = (coil_estimated_resistance[i] / 4.0f - 1.0f) / 0.00393f + 24.0f;
			coil_temp_from_resistance[i] -= CLAMP(coil_temp_from_resistance[i] - temp_from_resistance_reading, -0.003f, 0.003f);
		} else {
			// if there is not enough current for a valid resistance measurement, cool to temp sensor reading
			coil_temp_from_resistance[i] -= (coil_temp_from_resistance[i] - temp_sensor_reading[i]) * 0.0000005f;
		}
		coil_temp[i] = MAX(temp_sensor_reading[i], coil_temp_from_resistance[i]) * 100.0f; // convert to C/100
//		coil_temp[i] = temp_from_resistance*100.0f; // convert to C/100
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

void ITM_Send32(uint32_t port, uint32_t value) {
    while (!(ITM->PORT[port].u32)) ;  // Wait until ready
    ITM->PORT[port].u32 = value;
}

void HAL_ADC_HalfConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if (hadc->Instance == ADC1) {
		ADC1_ProcessBuffer(&ADC1_DMA_BUFFER[0]);
	} else if (hadc->Instance == ADC2) {
		ADC2_ProcessBuffer(&ADC2_DMA_BUFFER[0]);
	} else if (hadc->Instance == ADC3) {
		ITM_Send32(0, ADC3_DMA_BUFFER[0]); // Debugging line
		adc3_loop_counter++;
		ADC345_ProcessBuffer(&ADC3_DMA_BUFFER[0], 0);
	} else if (hadc->Instance == ADC4) {
		adc4_loop_counter++;
		ADC345_ProcessBuffer(&ADC4_DMA_BUFFER[0], 3);
	} else if (hadc->Instance == ADC5) {
		adc5_loop_counter++;
		ADC345_ProcessBuffer(&ADC5_DMA_BUFFER[0], 6);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if (hadc->Instance == ADC1) {
		ADC1_ProcessBuffer(&ADC1_DMA_BUFFER[ADC1_CHANNELS]);
	} else if (hadc->Instance == ADC2) {
		ADC2_ProcessBuffer(&ADC2_DMA_BUFFER[ADC2_CHANNELS]);
	} else if (hadc->Instance == ADC3) {
		adc3_loop_counter++;
		ADC345_ProcessBuffer(&ADC3_DMA_BUFFER[ADC3_CHANNELS], 0);
	} else if (hadc->Instance == ADC4) {
		adc4_loop_counter++;
		ADC345_ProcessBuffer(&ADC4_DMA_BUFFER[ADC4_CHANNELS], 3);
	} else if (hadc->Instance == ADC5) {
		adc5_loop_counter++;
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
		coil_estimated_resistance[i] = (v_sense_hv) * pid_pwm_output[i] / (float)coil_current_reading[i] * 1000.0f;
		if (isnan(coil_estimated_resistance[i]) || coil_setpoint[i] < 100 || abs((int)coil_setpoint[i] - (int)coil_current_reading[i]) > 400) {
			coil_estimated_resistance_report[i] = -1;
		} else {
			coil_estimated_resistance_report[i] = (uint16_t)(CLAMP(coil_estimated_resistance[i], 0.0f, 30.0f) * 1000.0f);
		}
		if (coil_setpoint[i] > MAX_SETPOINT || coil_current_reading[i] > MAX_SPIKE) {
			coil_pwm_ccr[i] = 0;
			coil_setpoint[i] = 0;
			pid_pwm_output[i] = 0;
			return;
		}
		// compute P and I error
		pid_error[i] = coil_setpoint[i] - coil_current_reading[i];
		pid_error_integral[i] += pid_error[i];
		// clamp integral
		pid_error_integral[i] = CLAMP(pid_error_integral[i], -PID_I_CLAMP, PID_I_CLAMP);
		// compute output change
		pid_pwm_change[i] = Kp * (float)pid_error[i]/1000.0f + Ki * (float)pid_error_integral[i]/1000.0f;
		// apply output change
		pid_pwm_output[i] += pid_pwm_change[i];
		// clamp output
		pid_pwm_output[i] = CLAMP(pid_pwm_output[i], 0.0f, max_allowed_duty_cycle);
		// convert to CCR value
		coil_pwm_ccr[i] = (uint16_t)(pid_pwm_output[i] * PWM_PERIOD);	}
}
