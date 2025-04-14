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

// ADC1 is voltage sensors
#define ADC1_CHANNELS 3

// ADC2 is temp sensors
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

extern uint16_t TEMP_SENSE_1;
extern uint16_t TEMP_SENSE_2;
extern uint16_t TEMP_SENSE_3;
extern uint16_t TEMP_SENSE_4;
extern uint16_t TEMP_SENSE_5;
extern uint16_t TEMP_SENSE_6;
extern uint16_t TEMP_SENSE_7;
extern uint16_t TEMP_SENSE_8;
extern uint16_t TEMP_SENSE_9;

extern uint16_t COIL_CURRENT_1;
extern uint16_t COIL_CURRENT_2;
extern uint16_t COIL_CURRENT_3;
extern uint16_t COIL_CURRENT_4;
extern uint16_t COIL_CURRENT_5;
extern uint16_t COIL_CURRENT_6;
extern uint16_t COIL_CURRENT_7;
extern uint16_t COIL_CURRENT_8;
extern uint16_t COIL_CURRENT_9;

// calculated values
extern float v_sense_hv;
extern float v_sense_12;
extern float v_sense_5;

extern uint16_t coil_1_current_reading;
extern uint16_t coil_2_current_reading;
extern uint16_t coil_3_current_reading;
extern uint16_t coil_4_current_reading;
extern uint16_t coil_5_current_reading;
extern uint16_t coil_6_current_reading;
extern uint16_t coil_7_current_reading;
extern uint16_t coil_8_current_reading;
extern uint16_t coil_9_current_reading;

extern int16_t coil_1_temp;
extern int16_t coil_2_temp;
extern int16_t coil_3_temp;
extern int16_t coil_4_temp;
extern int16_t coil_5_temp;
extern int16_t coil_6_temp;
extern int16_t coil_7_temp;
extern int16_t coil_8_temp;
extern int16_t coil_9_temp;

void ADC_Init(ADC_HandleTypeDef* hadc1, ADC_HandleTypeDef* hadc2, ADC_HandleTypeDef* hadc3, ADC_HandleTypeDef* hadc4, ADC_HandleTypeDef* hadc5);
void ADC1_ProcessBuffer(uint16_t* buffer);
void ADC2_ProcessBuffer(uint16_t* buffer);
void ADC3_ProcessBuffer(uint16_t* buffer);
void ADC4_ProcessBuffer(uint16_t* buffer);
void ADC5_ProcessBuffer(uint16_t* buffer);

#endif /* INC_ADC_H_ */
