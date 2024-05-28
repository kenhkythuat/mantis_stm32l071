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
#include "stm32l0xx_hal.h"

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
void sendingToSimcomA76xx(char *cmd);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USART4_TX_ESP32_Pin GPIO_PIN_0
#define USART4_TX_ESP32_GPIO_Port GPIOA
#define USART4_RX_ESP32_Pin GPIO_PIN_1
#define USART4_RX_ESP32_GPIO_Port GPIOA
#define USART2_TX_GNSS_Pin GPIO_PIN_2
#define USART2_TX_GNSS_GPIO_Port GPIOA
#define USART2_RX_GNSS_Pin GPIO_PIN_3
#define USART2_RX_GNSS_GPIO_Port GPIOA
#define SPI1_SCK_SDCARD_Pin GPIO_PIN_5
#define SPI1_SCK_SDCARD_GPIO_Port GPIOA
#define SPI1_MISO_SDCARD_Pin GPIO_PIN_6
#define SPI1_MISO_SDCARD_GPIO_Port GPIOA
#define SPI1_MOSI_SDCARD_Pin GPIO_PIN_7
#define SPI1_MOSI_SDCARD_GPIO_Port GPIOA
#define SPI1_CS_SDCARD_Pin GPIO_PIN_4
#define SPI1_CS_SDCARD_GPIO_Port GPIOC
#define ENABLE_A76XX_Pin GPIO_PIN_1
#define ENABLE_A76XX_GPIO_Port GPIOB
#define I2C2_SCL_DS3231_Pin GPIO_PIN_10
#define I2C2_SCL_DS3231_GPIO_Port GPIOB
#define I2C2_SDA_DS3231_Pin GPIO_PIN_11
#define I2C2_SDA_DS3231_GPIO_Port GPIOB
#define LED_GREEN_Pin GPIO_PIN_0
#define LED_GREEN_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_2
#define LED_RED_GPIO_Port GPIOB
#define LED_BLUE_Pin GPIO_PIN_5
#define LED_BLUE_GPIO_Port GPIOC
#define LED_STATUS_Pin GPIO_PIN_11
#define LED_STATUS_GPIO_Port GPIOA
#define USART1_TX_A76XX_Pin GPIO_PIN_9
#define USART1_TX_A76XX_GPIO_Port GPIOA
#define USART1_RX_A76XX_Pin GPIO_PIN_10
#define USART1_RX_A76XX_GPIO_Port GPIOA
#define PAYLOAD_1_Pin GPIO_PIN_15
#define PAYLOAD_1_GPIO_Port GPIOA
#define PAYLOAD_2_Pin GPIO_PIN_10
#define PAYLOAD_2_GPIO_Port GPIOC
#define PAYLOAD_3_Pin GPIO_PIN_11
#define PAYLOAD_3_GPIO_Port GPIOC
#define PAYLOAD_4_Pin GPIO_PIN_12
#define PAYLOAD_4_GPIO_Port GPIOC
#define PAYLOAD_5_Pin GPIO_PIN_2
#define PAYLOAD_5_GPIO_Port GPIOD
#define PAYLOAD_6_Pin GPIO_PIN_3
#define PAYLOAD_6_GPIO_Port GPIOB
#define PAYLOAD_7_Pin GPIO_PIN_4
#define PAYLOAD_7_GPIO_Port GPIOB
#define PAYLOAD_8_Pin GPIO_PIN_5
#define PAYLOAD_8_GPIO_Port GPIOB
#define PAYLOAD_9_Pin GPIO_PIN_6
#define PAYLOAD_9_GPIO_Port GPIOB
#define PAYLOAD_10_Pin GPIO_PIN_7
#define PAYLOAD_10_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
