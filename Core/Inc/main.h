/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IND_B_Pin GPIO_PIN_4
#define IND_B_GPIO_Port GPIOE
#define GD6_Pin GPIO_PIN_9
#define GD6_GPIO_Port GPIOB
#define BOOT0_SENSE_Pin GPIO_PIN_8
#define BOOT0_SENSE_GPIO_Port GPIOB
#define GD4_Pin GPIO_PIN_6
#define GD4_GPIO_Port GPIOB
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define RX_N_Pin GPIO_PIN_6
#define RX_N_GPIO_Port GPIOD
#define TX_N_Pin GPIO_PIN_5
#define TX_N_GPIO_Port GPIOD
#define CAN_TX_Pin GPIO_PIN_1
#define CAN_TX_GPIO_Port GPIOD
#define GD8_Pin GPIO_PIN_3
#define GD8_GPIO_Port GPIOE
#define RX_W_Pin GPIO_PIN_1
#define RX_W_GPIO_Port GPIOE
#define I2C_SDA_Pin GPIO_PIN_7
#define I2C_SDA_GPIO_Port GPIOB
#define IND_G_Pin GPIO_PIN_5
#define IND_G_GPIO_Port GPIOB
#define CAN_RX_Pin GPIO_PIN_0
#define CAN_RX_GPIO_Port GPIOD
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define GD7_Pin GPIO_PIN_2
#define GD7_GPIO_Port GPIOE
#define TX_W_Pin GPIO_PIN_0
#define TX_W_GPIO_Port GPIOE
#define IND_R_Pin GPIO_PIN_4
#define IND_R_GPIO_Port GPIOB
#define RX_E_Pin GPIO_PIN_11
#define RX_E_GPIO_Port GPIOC
#define TX_E_Pin GPIO_PIN_10
#define TX_E_GPIO_Port GPIOC
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define GD3_Pin GPIO_PIN_10
#define GD3_GPIO_Port GPIOA
#define GD2_Pin GPIO_PIN_9
#define GD2_GPIO_Port GPIOA
#define HSE_IN_Pin GPIO_PIN_0
#define HSE_IN_GPIO_Port GPIOF
#define HSE_OUT_Pin GPIO_PIN_1
#define HSE_OUT_GPIO_Port GPIOF
#define GD9_Pin GPIO_PIN_8
#define GD9_GPIO_Port GPIOC
#define GD1_Pin GPIO_PIN_8
#define GD1_GPIO_Port GPIOA
#define TEMP8_Pin GPIO_PIN_2
#define TEMP8_GPIO_Port GPIOC
#define TEMP6_Pin GPIO_PIN_0
#define TEMP6_GPIO_Port GPIOC
#define TEMP7_Pin GPIO_PIN_1
#define TEMP7_GPIO_Port GPIOC
#define GD5_Pin GPIO_PIN_7
#define GD5_GPIO_Port GPIOC
#define TEMP9_Pin GPIO_PIN_3
#define TEMP9_GPIO_Port GPIOC
#define TEMP2_Pin GPIO_PIN_1
#define TEMP2_GPIO_Port GPIOA
#define TEMP1_Pin GPIO_PIN_0
#define TEMP1_GPIO_Port GPIOA
#define EA8_Pin GPIO_PIN_10
#define EA8_GPIO_Port GPIOD
#define RX_S_Pin GPIO_PIN_9
#define RX_S_GPIO_Port GPIOD
#define VS_HV_Pin GPIO_PIN_2
#define VS_HV_GPIO_Port GPIOA
#define VS_12_Pin GPIO_PIN_3
#define VS_12_GPIO_Port GPIOA
#define EA7_Pin GPIO_PIN_8
#define EA7_GPIO_Port GPIOE
#define EA2_Pin GPIO_PIN_9
#define EA2_GPIO_Port GPIOE
#define EA5_Pin GPIO_PIN_15
#define EA5_GPIO_Port GPIOE
#define VS_5_Pin GPIO_PIN_14
#define VS_5_GPIO_Port GPIOB
#define EA9_Pin GPIO_PIN_11
#define EA9_GPIO_Port GPIOD
#define TEMP3_Pin GPIO_PIN_6
#define TEMP3_GPIO_Port GPIOA
#define EA4_Pin GPIO_PIN_14
#define EA4_GPIO_Port GPIOE
#define TX_S_Pin GPIO_PIN_10
#define TX_S_GPIO_Port GPIOB
#define TEMP4_Pin GPIO_PIN_7
#define TEMP4_GPIO_Port GPIOA
#define TEMP5_Pin GPIO_PIN_4
#define TEMP5_GPIO_Port GPIOC
#define EA1_Pin GPIO_PIN_1
#define EA1_GPIO_Port GPIOB
#define EA3_Pin GPIO_PIN_13
#define EA3_GPIO_Port GPIOE
#define EA6_Pin GPIO_PIN_12
#define EA6_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
