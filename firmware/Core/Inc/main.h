/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_BUTTON_Pin GPIO_PIN_13
#define USER_BUTTON_GPIO_Port GPIOC
#define CAM_D0_Pin GPIO_PIN_2
#define CAM_D0_GPIO_Port GPIOC
#define CAM_D1_Pin GPIO_PIN_3
#define CAM_D1_GPIO_Port GPIOC
#define CAM_RESET_N_Pin GPIO_PIN_0
#define CAM_RESET_N_GPIO_Port GPIOA
#define CAM_PWDN_Pin GPIO_PIN_1
#define CAM_PWDN_GPIO_Port GPIOA
#define DBG_UART_TX_Pin GPIO_PIN_2
#define DBG_UART_TX_GPIO_Port GPIOA
#define DBG_UART_RX_Pin GPIO_PIN_3
#define DBG_UART_RX_GPIO_Port GPIOA
#define STATUS_LED_Pin GPIO_PIN_5
#define STATUS_LED_GPIO_Port GPIOA
#define CAM_D2_Pin GPIO_PIN_4
#define CAM_D2_GPIO_Port GPIOC
#define CAM_D3_Pin GPIO_PIN_5
#define CAM_D3_GPIO_Port GPIOC
#define FLASH_CS_N_Pin GPIO_PIN_12
#define FLASH_CS_N_GPIO_Port GPIOB
#define FLASH_SPI_SCK_Pin GPIO_PIN_13
#define FLASH_SPI_SCK_GPIO_Port GPIOB
#define FLASH_SPI_MISO_Pin GPIO_PIN_14
#define FLASH_SPI_MISO_GPIO_Port GPIOB
#define FLASH_SPI_MOSI_Pin GPIO_PIN_15
#define FLASH_SPI_MOSI_GPIO_Port GPIOB
#define CAM_D4_Pin GPIO_PIN_6
#define CAM_D4_GPIO_Port GPIOC
#define CAM_D5_Pin GPIO_PIN_7
#define CAM_D5_GPIO_Port GPIOC
#define CAM_D6_Pin GPIO_PIN_8
#define CAM_D6_GPIO_Port GPIOC
#define CAM_PCLK_Pin GPIO_PIN_9
#define CAM_PCLK_GPIO_Port GPIOC
#define CAM_XCLK_Pin GPIO_PIN_8
#define CAM_XCLK_GPIO_Port GPIOA
#define CLI_UART_TX_Pin GPIO_PIN_9
#define CLI_UART_TX_GPIO_Port GPIOA
#define CLI_UART_RX_Pin GPIO_PIN_10
#define CLI_UART_RX_GPIO_Port GPIOA
#define CAM_D7_Pin GPIO_PIN_10
#define CAM_D7_GPIO_Port GPIOC
#define CAM_HREF_Pin GPIO_PIN_11
#define CAM_HREF_GPIO_Port GPIOC
#define CAM_HREF_EXTI_IRQn EXTI15_10_IRQn
#define CAM_VSYNC_Pin GPIO_PIN_12
#define CAM_VSYNC_GPIO_Port GPIOC
#define CAM_VSYNC_EXTI_IRQn EXTI15_10_IRQn
#define CAM_SIOC_Pin GPIO_PIN_6
#define CAM_SIOC_GPIO_Port GPIOB
#define CAM_SIOD_Pin GPIO_PIN_7
#define CAM_SIOD_GPIO_Port GPIOB
#define IMU_I2C_SCL_Pin GPIO_PIN_8
#define IMU_I2C_SCL_GPIO_Port GPIOB
#define IMU_I2C_SDA_Pin GPIO_PIN_9
#define IMU_I2C_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
