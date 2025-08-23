/*
 * adc.h
 *
 *  Created on: Apr 13, 2025
 *      Author: cywestbrook
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include <stdint.h>
#include "stm32g4xx_hal.h"

#define NUM_COILS 9

// ADC1 is voltage sensors and internal temperature sensor
#define ADC1_CHANNELS 4

// ADC2 is external coil temp sensors
#define ADC2_CHANNELS 9

// ADC3 is current sensors for coils 1-3
#define ADC3_CHANNELS 3

// ADC4 is current sensors for coils 4-6
#define ADC4_CHANNELS 3

// ADC5 is current sensors for coils 7-9
#define ADC5_CHANNELS 3

// raw readings
extern uint16_t V_SENSE_HV;
extern uint16_t V_SENSE_12;
extern uint16_t V_SENSE_5;

extern uint16_t TEMP_SENSE[NUM_COILS];

extern uint16_t COIL_CURRENT[NUM_COILS];

// calculated values
extern float v_sense_hv;
extern float v_sense_12;
extern float v_sense_5;

extern float master_v_sense_hv;

// PID
extern uint16_t coil_setpoint[NUM_COILS];
extern uint16_t coil_pwm_ccr[NUM_COILS];
extern int32_t pid_error[NUM_COILS];
extern int32_t pid_error_integral[NUM_COILS];
extern float pid_pwm_change[NUM_COILS];
extern float pid_pwm_output[NUM_COILS];

extern float Kp;
extern float Ki;

extern uint16_t coil_current_reading[NUM_COILS];

extern int16_t coil_estimated_resistance_report[NUM_COILS];

extern int16_t coil_temp[NUM_COILS];

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim20;

void ADC_Init(ADC_HandleTypeDef* hadc1, ADC_HandleTypeDef* hadc2, ADC_HandleTypeDef* hadc3, ADC_HandleTypeDef* hadc4, ADC_HandleTypeDef* hadc5);
void ADC1_ProcessBuffer(uint16_t* buffer);
void ADC2_ProcessBuffer(uint16_t* buffer);
void ADC345_ProcessBuffer(uint16_t* buffer, uint8_t coil_offset);
void PID_DisableAll();
void PID_Solve3(uint8_t coil_offset);
void PID_SetCCR3();

#endif /* INC_ADC_H_ */
