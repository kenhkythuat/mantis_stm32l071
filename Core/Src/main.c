/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "config.h"
#include "stdbool.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

GPIO_TypeDef*  GPIO_LOAD_PORT[] = {PAYLOAD_1_GPIO_Port,PAYLOAD_2_GPIO_Port,PAYLOAD_3_GPIO_Port,PAYLOAD_4_GPIO_Port,PAYLOAD_5_GPIO_Port,PAYLOAD_6_GPIO_Port,PAYLOAD_7_GPIO_Port,PAYLOAD_8_GPIO_Port,PAYLOAD_9_GPIO_Port,PAYLOAD_10_GPIO_Port};
unsigned int GPIO_LOAD_PIN[] = {PAYLOAD_1_Pin,PAYLOAD_2_Pin,PAYLOAD_3_Pin,PAYLOAD_4_Pin,PAYLOAD_5_Pin,PAYLOAD_6_Pin,PAYLOAD_7_Pin,PAYLOAD_8_Pin,PAYLOAD_9_Pin,PAYLOAD_10_Pin};


char AT_RESET[]= "AT+CRESET\r\n";
char AT_CHECK_A76XX[]= "AT\r\n";
char AT_CHECK_ESIM[]= "AT+CGREG?\r\n";
char AT_START_MQTT[]= "AT+CMQTTSTART\r\n";
char AT_ACQUIRE_CLIENT[]="AT+CMQTTACCQ=0,\"%s\",0\r\n";
char AT_CONNECT_MQTT[]="AT+CMQTTCONNECT=0,\"%s:%d\",60,1,\"%s\",\"%s\"\r\n";
char AT_SET_PUBLISH_TOPIC[]= "AT+CMQTTTOPIC=0,%d\r\n";
char AT_SET_PUBLISH_PAYLOAD[]="AT+CMQTTPAYLOAD=0,%d\r\n";
char AT_PUBLISH[]="AT+CMQTTPUB=0,1,60\r\n";
char AT_SET_SUBCRIBE_0_9_TOPIC[]="AT+CMQTTSUBTOPIC=0,%d,1\r\n";
char AT_SET_SUBCRIBE_10_18_TOPIC[]="AT+CMQTTSUBTOPIC=0,%d,1\r\n";
char AT_SUBCRIBE_TOPIC[]= "%s%d\r\n";
char AT_SUBCRIBE[]="AT+CMQTTSUB=0\r\n";
char AT_COMMAND[100];
char AT_INFORM_PAYLOAD[]="{%d:%d}\r\n";

char STATUS_PAYLOAD_ARRAY_TOTAL[]="{\"1\":0,\"2\":0,\"3\":0,\"4\":0,\"5\":0,\"6\":0,\"7\":0,\"8\":0,\"9\":0,\"10\":0,\"11\":0,\"12\":0,\"13\":0,\"14\":0,\"15\":0,\"16\":0,\"17\":0,\"18\":0}";
int timeOutConnectA76XX= 30000;
int timeOutConnectMQTT= 15000;
int isATOK= 0;
int isPBDONE= 0;
int payLoadPin,payLoadStatus;

int rxDataCouter=0;
char simcomRxBuffer[100];
int payloadIndex;
char rxData;
uint16_t rxIndex;
int loadflag = 0;
char rxBuffer[100];
int previousTick;

int isConnectSimcomA76xx= 0;
int isConnectMQTT= 0;
int sendPayloadStatusToServer;

//extern int  lenghtOfStatusPayloadArray;
int lengthOfStatusPayloadArray;
char STATUS_PAYLOAD_ARRAY_0_9[LENGTH_STATUS_PAYLOAD_0_9];
char STATUS_PAYLOAD_ARRAY_10_18[LENGTH_STATUS_PAYLOAD_10_18];
int statusOfLoad;
int testlength;
int ledStatusSendTopic=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART4_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_IWDG_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void turnOnA76XX(void);	//reset module sim A76XX
void processPayload(void);
int connectSimcomA76xx();
int connectMQTT();
void sendingToSimcomA76xx(char *cmd);
void ledStatus(char cmd);
int sendStatusPayloadToMQTT();
void informPayloadToServer(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{
//	if(huart -> Instance == USART1)
//	{
//		processPayload();
//		HAL_UARTEx_ReceiveToIdle_IT(huart, (uint8_t *) simcomRxBuffer, 124);
//	}
//}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart -> Instance == USART1)
	{
		if((rxData!='\r')&&(rxData!='\n')){
			simcomRxBuffer[rxIndex++]=rxData;
			rxDataCouter++;
		}
		else{
			if(isConnectMQTT==1){
				switch(rxDataCouter){
				case strlen(FARM)+16:
					payLoadPin = (int)simcomRxBuffer[strlen(FARM)+15] -48;
					loadflag = 1;
					break;
				case strlen(FARM)+17:
					payLoadPin = ((int)simcomRxBuffer[strlen(FARM)+15] -48)*10+((int)simcomRxBuffer[strlen(FARM)+16]-48);
					loadflag = 1;
					break;
				case 1:
					if((loadflag==1)&&(payLoadPin<=NUMBER_LOADS)){
						payLoadStatus = (int)simcomRxBuffer[0] -48;
						if((payLoadStatus==0)||(payLoadStatus==1)){
						HAL_GPIO_WritePin(GPIO_LOAD_PORT[payLoadPin-1],GPIO_LOAD_PIN[payLoadPin-1],payLoadStatus);
						}
						//informPayloadToServer();
						loadflag = 0;
					}
					break;
				case 14:
					if(strstr((char *)simcomRxBuffer,"CMQTTPUB: 0,0")){
						IWDG->KR = 0xAAAA;
						ledStatusSendTopic=1;
					}
				default:
				}
			}
			rxDataCouter=0;
			rxIndex=0;
		}
	}
	HAL_UART_Receive_IT(&huart1, &rxData, 1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 if(htim->Instance == htim6.Instance)
 {
	 if(isConnectMQTT){
		 sendPayloadStatusToServer=1;
	 }
 }
 HAL_TIM_Base_Start_IT(&htim6);
}
void sendingToSimcomA76xx(char *cmd)
{
  HAL_UART_Transmit(&huart1,(uint8_t *)cmd,strlen(cmd),1000);
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
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART4_UART_Init();
  MX_TIM6_Init();
  //MX_IWDG_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
//  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, SET);
//  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, SET);
//  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, SET);
  HAL_UART_Receive_IT(&huart1, &rxData, 1);
  HAL_TIM_Base_Start_IT(&htim6);
  ledStatus('R');//xanh la
  turnOnA76XX();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(!isConnectSimcomA76xx){
		  isConnectSimcomA76xx = connectSimcomA76xx();
	  }
	  if(!isConnectMQTT){
		  isConnectMQTT = connectMQTT();
	  }
	  if(sendPayloadStatusToServer == 1){
		  sendStatusPayloadToMQTT();
		  sendPayloadStatusToServer= 0;
	  }
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00707CBB;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim6.Init.Period = 9999;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART4_UART_Init(void)
{

  /* USER CODE BEGIN USART4_Init 0 */

  /* USER CODE END USART4_Init 0 */

  /* USER CODE BEGIN USART4_Init 1 */

  /* USER CODE END USART4_Init 1 */
  huart4.Instance = USART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART4_Init 2 */

  /* USER CODE END USART4_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SPI1_CS_SDCARD_Pin|LED_BLUE_Pin
                          |PAYLOAD_2_Pin|PAYLOAD_3_Pin|PAYLOAD_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ENABLE_A76XX_Pin|GPIO_PIN_12|LED_GREEN_Pin|LED_RED_Pin|GPIO_PIN_14|PAYLOAD_6_Pin
                          |PAYLOAD_7_Pin|PAYLOAD_8_Pin|PAYLOAD_9_Pin|PAYLOAD_10_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PAYLOAD_1_GPIO_Port, PAYLOAD_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PAYLOAD_5_GPIO_Port, PAYLOAD_5_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SPI1_CS_SDCARD_Pin LED_GREEN_Pin LED_RED_Pin LED_BLUE_Pin
                           LED_STATUS_Pin PAYLOAD_2_Pin PAYLOAD_3_Pin PAYLOAD_4_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_SDCARD_Pin|LED_BLUE_Pin
                          |PAYLOAD_2_Pin|PAYLOAD_3_Pin|PAYLOAD_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin =   LED_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  /*Configure GPIO pins : ENABLE_A76XX_Pin PB12 PB14 PAYLOAD_6_Pin
                           PAYLOAD_7_Pin PAYLOAD_8_Pin PAYLOAD_9_Pin PAYLOAD_10_Pin */
  GPIO_InitStruct.Pin = ENABLE_A76XX_Pin|LED_GREEN_Pin|LED_RED_Pin|GPIO_PIN_12|GPIO_PIN_14|PAYLOAD_6_Pin
                          |PAYLOAD_7_Pin|PAYLOAD_8_Pin|PAYLOAD_9_Pin|PAYLOAD_10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PAYLOAD_1_Pin */
  GPIO_InitStruct.Pin = PAYLOAD_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PAYLOAD_1_GPIO_Port, &GPIO_InitStruct);

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



void turnOnA76XX(void){
	HAL_GPIO_WritePin(ENABLE_A76XX_GPIO_Port, ENABLE_A76XX_Pin, RESET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(ENABLE_A76XX_GPIO_Port, ENABLE_A76XX_Pin, SET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(ENABLE_A76XX_GPIO_Port, ENABLE_A76XX_Pin, RESET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(ENABLE_A76XX_GPIO_Port, ENABLE_A76XX_Pin, SET);
	HAL_Delay(10000);

}

int connectSimcomA76xx(){
	previousTick =  HAL_GetTick();
	while(isConnectSimcomA76xx == 0 && previousTick  + timeOutConnectA76XX >  HAL_GetTick()){
		if(strstr((char *)simcomRxBuffer,"PB DONE")){
			isPBDONE = 1;
		}
		if(isPBDONE==1){
			memset(simcomRxBuffer,'0',100);
			HAL_Delay(200);
			sendingToSimcomA76xx(AT_CHECK_A76XX);
			HAL_Delay(200);
			if(strstr((char *)simcomRxBuffer,"OK")){
				isATOK = 1;
			}
		}
		if(isATOK==1){
			memset(simcomRxBuffer,'0',100);
			HAL_Delay(200);
			sendingToSimcomA76xx(AT_CHECK_ESIM);
			HAL_Delay(200);
			if(strstr((char *)simcomRxBuffer,"OKGREG: 0,1")){
				isConnectSimcomA76xx = 1;
				ledStatus('B');
			}
		}
	}
	if(isConnectSimcomA76xx==0){
		NVIC_SystemReset();
	}

	return isConnectSimcomA76xx;
}

int connectMQTT(void){
	sendingToSimcomA76xx(AT_START_MQTT);
	HAL_Delay(200);
	sprintf(AT_COMMAND,AT_ACQUIRE_CLIENT,MQTT_CLIENT_ID);
	sendingToSimcomA76xx(AT_COMMAND);
	HAL_Delay(200);
	sprintf(AT_COMMAND,AT_CONNECT_MQTT,MQTT_HOST,MQTT_PORT,MQTT_USER,MQTT_PASS);
	sendingToSimcomA76xx(AT_COMMAND);
	HAL_Delay(200);
	for(int i=1;i<NUMBER_LOADS+1;i++){
		isConnectMQTT = 0;
		previousTick =  HAL_GetTick();
		if(i>9){
			sprintf(AT_COMMAND,AT_SET_SUBCRIBE_10_18_TOPIC,strlen(FARM)+17);
			sendingToSimcomA76xx(AT_COMMAND);
			HAL_Delay(200);
		}
		else{
			sprintf(AT_COMMAND,AT_SET_SUBCRIBE_0_9_TOPIC,strlen(FARM)+16);//
			sendingToSimcomA76xx(AT_COMMAND);
		}
		HAL_Delay(200);
		sprintf(AT_COMMAND,AT_SUBCRIBE_TOPIC,MQTT_TOPIC_ACTUATOR_CONTROL,i);
		sendingToSimcomA76xx(AT_COMMAND);
		HAL_Delay(200);
		memset(simcomRxBuffer,'0',100);
		sendingToSimcomA76xx(AT_SUBCRIBE);
		HAL_Delay(200);
		while(isConnectMQTT == 0 && previousTick  + timeOutConnectMQTT >  HAL_GetTick()){

			if(strstr((char *)simcomRxBuffer,"CMQTTSUB: 0,0")){
					isConnectMQTT=1;
			}
		}
		if(isConnectMQTT==0){
			NVIC_SystemReset();;
		}
	}
	if(isConnectMQTT==1){
		ledStatus('G');
		MX_IWDG_Init();
	}
	return isConnectMQTT;
}

void ledStatus(char cmd){
	if(cmd == 'R'){
		HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, SET);
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, SET);
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, RESET);
	}
	else if(cmd == 'B'){//chua sua
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

void informPayloadToServer(void){
//	sprintf(AT_COMMAND,AT_SET_PUBLISH_TOPIC,strlen(MQTT_TOPIC_ACTUATOR_STATUS)); // Set the topic for publish message
//	sendingToSimcomA76xx(AT_COMMAND);
//	sprintf(AT_COMMAND,"%s\r\n",MQTT_TOPIC_ACTUATOR_STATUS);
//	sendingToSimcomA76xx(AT_COMMAND);
//
//	sprintf(AT_COMMAND,AT_INFORM_PAYLOAD,payLoadPin,payLoadStatus);
//
////	sendingToSimcomA76xx(AT_COMMAND,"{,payLoadPin,payLoadStatus);
//
//	int lengthOfInformPayload = strlen(AT_COMMAND);
//	sprintf(AT_COMMAND,AT_SET_PUBLISH_PAYLOAD,lengthOfInformPayload-2);
//	sendingToSimcomA76xx(AT_COMMAND);
//	sprintf(AT_COMMAND,AT_INFORM_PAYLOAD,payLoadPin,payLoadStatus);
//	sendingToSimcomA76xx(AT_COMMAND); // Set the payload
//	sendingToSimcomA76xx(AT_PUBLISH);
}// Publish
int sendStatusPayloadToMQTT(){
	//lengthOfStatusPayloadArray = (7*NUMBER_LOADS) +(NUMBER_LOADS-1)+2;
	//STATUS_PAYLOAD_ARRAY[lengthOfStatusPayloadArray];
	if(NUMBER_LOADS<10){
		memcpy(STATUS_PAYLOAD_ARRAY_0_9,STATUS_PAYLOAD_ARRAY_TOTAL,LENGTH_STATUS_PAYLOAD_0_9-1);
		STATUS_PAYLOAD_ARRAY_0_9[LENGTH_STATUS_PAYLOAD_0_9-1] = '}';
		for(int i=1;i<NUMBER_LOADS+1;i++){
				statusOfLoad = HAL_GPIO_ReadPin(GPIO_LOAD_PORT[i-1], GPIO_LOAD_PIN[i-1]);
				STATUS_PAYLOAD_ARRAY_0_9[i*6-1] = statusOfLoad+48;
		}
		sprintf(AT_COMMAND,AT_SET_PUBLISH_TOPIC,strlen(MQTT_TOPIC_ACTUATOR_STATUS)); // Set the topic for publish message
		sendingToSimcomA76xx(AT_COMMAND);
		HAL_Delay(200);
		sprintf(AT_COMMAND,"%s\r\n",MQTT_TOPIC_ACTUATOR_STATUS);
		sendingToSimcomA76xx(AT_COMMAND);
		HAL_Delay(200);

		//sprintf(AT_COMMAND,STATUS_PAYLOAD_ARRAY_0_9,payLoadPin,payLoadStatus);
		int lengthOfInformPayload = strlen(STATUS_PAYLOAD_ARRAY_0_9);

		sprintf(AT_COMMAND,AT_SET_PUBLISH_PAYLOAD,lengthOfInformPayload);
		sendingToSimcomA76xx(AT_COMMAND);
		HAL_Delay(200);
		//sprintf(AT_COMMAND,AT_INFORM_PAYLOAD,payLoadPin,payLoadStatus);
		sendingToSimcomA76xx(STATUS_PAYLOAD_ARRAY_0_9);
		HAL_Delay(200);// Set the payload
		sendingToSimcomA76xx(AT_PUBLISH);
		HAL_Delay(200);
	}
	else{
		memcpy(STATUS_PAYLOAD_ARRAY_10_18,STATUS_PAYLOAD_ARRAY_TOTAL,LENGTH_STATUS_PAYLOAD_10_18-1);
		STATUS_PAYLOAD_ARRAY_10_18[LENGTH_STATUS_PAYLOAD_10_18-1] = '}';
		for(int i=1;i<10;i++){
			statusOfLoad = HAL_GPIO_ReadPin(GPIO_LOAD_PORT[i-1], GPIO_LOAD_PIN[i-1]);
			STATUS_PAYLOAD_ARRAY_10_18[i*6-1] = statusOfLoad+48;

		}
		int j=0;
		for(int i=10;i<NUMBER_LOADS+1;i++){
			j++;
			statusOfLoad = HAL_GPIO_ReadPin(GPIO_LOAD_PORT[i-1], GPIO_LOAD_PIN[i-1]);
			STATUS_PAYLOAD_ARRAY_10_18[i*6+j-1] = statusOfLoad+48;
		}

		sprintf(AT_COMMAND,AT_SET_PUBLISH_TOPIC,strlen(MQTT_TOPIC_ACTUATOR_STATUS)); // Set the topic for publish message
		sendingToSimcomA76xx(AT_COMMAND);
		HAL_Delay(200);
		sprintf(AT_COMMAND,"%s\r\n",MQTT_TOPIC_ACTUATOR_STATUS);
		sendingToSimcomA76xx(AT_COMMAND);
		HAL_Delay(200);

		//sprintf(AT_COMMAND,STATUS_PAYLOAD_ARRAY_0_9,payLoadPin,payLoadStatus);
		int lengthOfInformPayload = strlen(STATUS_PAYLOAD_ARRAY_10_18);

		sprintf(AT_COMMAND,AT_SET_PUBLISH_PAYLOAD,lengthOfInformPayload);
		sendingToSimcomA76xx(AT_COMMAND);
		HAL_Delay(200);
		//sprintf(AT_COMMAND,AT_INFORM_PAYLOAD,payLoadPin,payLoadStatus);
		sendingToSimcomA76xx(STATUS_PAYLOAD_ARRAY_10_18);
		HAL_Delay(200);// Set the payload
		sendingToSimcomA76xx(AT_PUBLISH);
		HAL_Delay(200);

	}

	if(ledStatusSendTopic == 1){
		ledStatus('W');
		HAL_Delay(500);
		ledStatus('G');
		ledStatusSendTopic= 0;
	}
	//sendPayloadStatusToServer = 0;
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
  while (1)
  {
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
