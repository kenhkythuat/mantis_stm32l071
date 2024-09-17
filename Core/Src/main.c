/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cJSON.h"
#include "config.h"
#include "stdbool.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//int _write(int file, char *ptr, int len) {
//
//  int i;
//  for (i = 0; i < len; i++) {
//    ITM_SendChar(*ptr++);
//  }
//  return len;
//}

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
GPIO_TypeDef *GPIO_LOAD_PORT[10] = {PAYLOAD_1_GPIO_Port, PAYLOAD_2_GPIO_Port, PAYLOAD_3_GPIO_Port, PAYLOAD_4_GPIO_Port,PAYLOAD_5_GPIO_Port,PAYLOAD_6_GPIO_Port,PAYLOAD_7_GPIO_Port,PAYLOAD_8_GPIO_Port,PAYLOAD_9_GPIO_Port,PAYLOAD_10_GPIO_Port};
unsigned int GPIO_LOAD_PIN[10] = {PAYLOAD_1_Pin, PAYLOAD_2_Pin, PAYLOAD_3_Pin, PAYLOAD_4_Pin,PAYLOAD_5_Pin,PAYLOAD_6_Pin,PAYLOAD_7_Pin,PAYLOAD_8_Pin,PAYLOAD_9_Pin,PAYLOAD_10_Pin};

char AT_COMMAND[100];
float SignalStrength = 0;
int update_10_minute;
int onReay = 0;
int rssi = -99;

int timeOutConnectMQTT = 15000;
int isPBDONE = 0;
int payLoadPin, payLoadStatus;

char rxBuffer[150];
char rx_data_sim[150];
int previousTick;
int isConnectSimcomA76xx = 0;
int isConnectMQTT = 0;
float Data_Percentage_pin;
int  update_status_to_server;

bool fn_Enable_MQTT = false;
bool fn_Connect_MQTT = false;
bool fn_CheckSim = false;
bool fn_Subcribe_MQTT = false;
bool fn_Publish_MQTT = false;
bool fn_Acquier_MQTT = false;
bool fn_update_status = false;

uint32_t address = 0x0801FC00;
uint32_t data = 0x01;
uint32_t read_data = 3;
uint32_t value_page[4];
// char rxBuffer[50];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
void turnOnA76XX(void); // reset module sim A76XX
void processPayload(void);
int connectSimcomA76xx();
int connectMQTT();
void sendingToSimcomA76xx(char *cmd);
void ledStatus(char cmd);
int sendStatusPayloadToMQTT(void);
void informPayloadToServer(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == htim6.Instance) {
    if (isConnectMQTT) {
      update_status_to_server = 1;
      update_10_minute++;
    }
    IWDG->KR = 0xAAAA;
  }
  HAL_TIM_Base_Start_IT(&htim6);
}
void ledStatus(char cmd){
	if(cmd == 'R'){
		HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, SET);
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, SET);
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, RESET);
	}
	else if(cmd == 'B'){
		HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, SET);
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, RESET);
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, SET);
	}
	else if(cmd == 'G'){
		HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, RESET);
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, SET);
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, SET);
	}
	else if(cmd == 'W'){
			HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, RESET);
			HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, RESET);
			HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, RESET);
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  printf("-----Welcome to Agriconnect-----\n");
  printf("-----Hello Cricket-----\n");
#if SAVE_LOAD
    read_flash_payload();
#endif

  HAL_UARTEx_ReceiveToIdle_IT(&huart1, (uint8_t *)rxBuffer, 150);
  HAL_TIM_Base_Start_IT(&htim6);
  turnOnA76XX();
  isPBDONE = event_wait_function();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (!isConnectMQTT) {
      isConnectMQTT = init_cricket();
    }
    if (update_status_to_server == 1) {
        fn_update_status = update_status();
        update_status_to_server = 0;
    }
    if(update_10_minute>=3)
    {
	  //Data_Percentage_pin = Level_Pin();
	  rssi = read_signal_quality();
#if SAVE_LOAD
    	read_statusload();
#endif
    	update_10_minute=0;
    }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_8;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 31999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 4999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_BLUE_Pin|PAYLOAD_2_Pin|PAYLOAD_3_Pin|PAYLOAD_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin|A76XX_PWRKEY_Pin|LED_RED_Pin|GPIO_PIN_12
                          |GPIO_PIN_14|PAYLOAD_6_Pin|PAYLOAD_7_Pin|PAYLOAD_8_Pin
                          |PAYLOAD_9_Pin|PAYLOAD_10_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_STATUS_Pin|PAYLOAD_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PAYLOAD_5_GPIO_Port, PAYLOAD_5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_BLUE_Pin PAYLOAD_2_Pin PAYLOAD_3_Pin PAYLOAD_4_Pin */
  GPIO_InitStruct.Pin = LED_BLUE_Pin|PAYLOAD_2_Pin|PAYLOAD_3_Pin|PAYLOAD_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin A76XX_PWRKEY_Pin LED_RED_Pin PB12
                           PB14 PAYLOAD_6_Pin PAYLOAD_7_Pin PAYLOAD_8_Pin
                           PAYLOAD_9_Pin PAYLOAD_10_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|A76XX_PWRKEY_Pin|LED_RED_Pin|GPIO_PIN_12
                          |GPIO_PIN_14|PAYLOAD_6_Pin|PAYLOAD_7_Pin|PAYLOAD_8_Pin
                          |PAYLOAD_9_Pin|PAYLOAD_10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_STATUS_Pin PAYLOAD_1_Pin */
  GPIO_InitStruct.Pin = LED_STATUS_Pin|PAYLOAD_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PAYLOAD_5_Pin */
  GPIO_InitStruct.Pin = PAYLOAD_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PAYLOAD_5_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void turnOnA76XX(void) {
	ledStatus('R');
  printf("Enable SIMCOM\n");
  HAL_GPIO_WritePin(A76XX_PWRKEY_GPIO_Port, A76XX_PWRKEY_Pin, SET);
  HAL_Delay(3000);
  HAL_GPIO_WritePin(A76XX_PWRKEY_GPIO_Port, A76XX_PWRKEY_Pin, RESET);
  HAL_Delay(3000);
  HAL_GPIO_WritePin(A76XX_PWRKEY_GPIO_Port, A76XX_PWRKEY_Pin, SET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(A76XX_PWRKEY_GPIO_Port, A76XX_PWRKEY_Pin, RESET);
  HAL_Delay(1000);
//
//    HAL_GPIO_WritePin(A76XX_PWRKEY_GPIO_Port, A76XX_PWRKEY_Pin, RESET);
//    HAL_Delay(3500);
//    HAL_GPIO_WritePin(A76XX_PWRKEY_GPIO_Port, A76XX_PWRKEY_Pin, SET);
//    HAL_Delay(3500);
//    HAL_GPIO_WritePin(A76XX_PWRKEY_GPIO_Port, A76XX_PWRKEY_Pin, RESET);
//    HAL_Delay(200);
//    HAL_GPIO_WritePin(A76XX_PWRKEY_GPIO_Port, A76XX_PWRKEY_Pin, SET);
//    HAL_Delay(1000);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
