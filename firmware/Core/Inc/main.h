/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
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
#define TOUCH_S_Pin GPIO_PIN_0
#define TOUCH_S_GPIO_Port GPIOA
#define TOUCH_LED_Pin GPIO_PIN_1
#define TOUCH_LED_GPIO_Port GPIOA
#define TOUCH_D_Pin GPIO_PIN_2
#define TOUCH_D_GPIO_Port GPIOA
#define TOUCH_U_Pin GPIO_PIN_3
#define TOUCH_U_GPIO_Port GPIOA
#define PT100_Pin GPIO_PIN_4
#define PT100_GPIO_Port GPIOA
#define NTC_Pin GPIO_PIN_5
#define NTC_GPIO_Port GPIOA
#define PWM_CTRL_Pin GPIO_PIN_8
#define PWM_CTRL_GPIO_Port GPIOA
#define OLED_RST_Pin GPIO_PIN_15
#define OLED_RST_GPIO_Port GPIOA
#define FET1_Pin GPIO_PIN_5
#define FET1_GPIO_Port GPIOB
#define FET2_Pin GPIO_PIN_6
#define FET2_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_7
#define LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define VREFINT_CAL_ADDRESS 0x1FFFF7BA
#define PWM_CTRL_CHANNEL	TIM_CHANNEL_1
#define NVM_BASE_ADDRESS	0x08007C00UL

typedef struct {
	// ADC handle
	ADC_HandleTypeDef * adc_handle;

	uint16_t dma_buffer[4];
	uint16_t * raw_pt100;
	uint16_t * raw_ntc;
	uint16_t * raw_int_temp;
	uint16_t * raw_int_ref;

	volatile uint8_t dma_flag;

} ANALOG;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
