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
#include "stdbool.h"
float Level_Pin (void);
void write_load_statues();
void flash_erase(uint32_t numberpages);
void write_flash(int move ,uint32_t Data);
uint32_t read_page();
extern uint32_t value_page0;
extern uint32_t value_page1;
extern uint32_t value_page2;
extern uint32_t value_page3;
extern uint32_t value_Relay;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define a7672 1
#define a7670 2
#define a7677 3
#define true 1
#define false 0

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
extern void send_to_simcom_a76xx(char *cmd);
extern void create_JSON(void);
extern float fn_check_signal_simcom(void);
extern int acquire_gsm_mqtt_client(void);
extern int enable_mqtt_on_gsm_modem(void);
extern int connect_mqtt_server_by_gsm(void);
extern int subscribe_mqtt_via_gsm(void);
extern int publish_mqtt_via_gsm(void);
extern int read_signal_quality(void);
extern int stop_mqtt_via_gsm(void);
extern int check_error_mqtt_via_gsm(void);
extern int update_status(void);
extern void restart_stm32(void);
extern int init_cricket(void);
extern int event_wait_function(void);
extern int check_active_payload(void);
extern void read_flash_payload(void);
extern void led_status(char cmd);

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
//void send_to_simcom_a76xx(char *cmd);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_BLUE_Pin GPIO_PIN_5
#define LED_BLUE_GPIO_Port GPIOC
#define LED_GREEN_Pin GPIO_PIN_0
#define LED_GREEN_GPIO_Port GPIOB
#define A76XX_PWRKEY_Pin GPIO_PIN_1
#define A76XX_PWRKEY_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_2
#define LED_RED_GPIO_Port GPIOB
#define LED_STATUS_Pin GPIO_PIN_11
#define LED_STATUS_GPIO_Port GPIOA
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
extern char rx_buffer[800];
extern char rx_data_sim[800];
extern char array_at_command[150];
extern int is_pb_done;
extern int isATOK;
extern int on_relay;
extern int is_connect_mqtt;
extern int previousTick;
extern int timeOutConnectMQTT;
extern int payLoadPin,payLoadStatus;
extern char array_json[150];
extern float data_percentage_pin;
extern float signal_strength;
extern int rssi;
extern int is_connect_simcom;
extern bool is_fn_enable_mqtt;
extern bool is_fn_connect_mqtt;
extern bool is_fn_check_sim;
extern bool is_fn_subcribe_mqtt;
extern bool is_fn_publish_mqtt;
extern bool is_fn_acquier_mqtt;
extern bool is_fn_update_status;
extern GPIO_TypeDef* GPIO_LOAD_PORT[10];
extern unsigned int GPIO_LOAD_PIN[10];
enum GmsModemState
{
	Off,
	On,
	InternetReady,
	MqttReady,
	Subscribed,
	UpdateToServer,
	DisconnectMqtt
};
extern enum GmsModemState CurrentStatusSimcom;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
