/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
static char __LOG_BUFFER[128];

#define loge(...) do {\
		int size = sprintf(__LOG_BUFFER, __VA_ARGS__); \
		HAL_UART_Transmit(&huart3,(unsigned char*) "ERR:", 4, HAL_MAX_DELAY); \
		HAL_UART_Transmit(&huart3,(unsigned char*) __LOG_BUFFER, size + 1, HAL_MAX_DELAY); \
	}while(0)

// #define PRINT_DEBUG
#ifdef PRINT_DEBUG
#define logd(...) do {\
		int size = sprintf(__LOG_BUFFER, __VA_ARGS__); \
		HAL_UART_Transmit(&huart3,(unsigned char*) "DBG:", 4, HAL_MAX_DELAY); \
		HAL_UART_Transmit(&huart3,(unsigned char*) __LOG_BUFFER, size + 1, HAL_MAX_DELAY); \
	}while(0)
#else
#define logd(...) do {} while(0);
#endif

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AT45DB_NSS_Pin GPIO_PIN_12
#define AT45DB_NSS_GPIO_Port GPIOB
#define TEC_HEATER_Pin GPIO_PIN_8
#define TEC_HEATER_GPIO_Port GPIOC
#define TEC_COOLER_Pin GPIO_PIN_8
#define TEC_COOLER_GPIO_Port GPIOA
#define LED_BLUE_Pin GPIO_PIN_9
#define LED_BLUE_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
