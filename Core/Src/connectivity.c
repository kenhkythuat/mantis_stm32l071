/*
 * connectivity.c
 *
 *  Created on: Aug 27, 2024
 *      Author: thuanphat7
 */
#include "cJSON.h"
#include "config.h"
#include "main.h"
#include "stdbool.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <math.h>
extern UART_HandleTypeDef huart1;
char array_json[200];
// float Percentage_battery;

void send_to_simcom_a76xx(char *cmd) {
  printf("STM32 Write: %s", cmd);
  HAL_UART_Transmit(&huart1, (uint8_t *)cmd, strlen(cmd), 1000);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart->Instance == USART1) {
    printf("\r\nSIMCOM Response:");
    printf(rx_buffer);
    static int times;
    times = strlen(rx_buffer);
    for (int i = 0; i < times; i++) {
      rx_data_sim[i] = rx_buffer[i];
      if ((char)rx_buffer[i] == (char)SERIAL_NUMBER[5] &&
          (char)rx_buffer[i + 1] == (char)SERIAL_NUMBER[6] &&
          (char)rx_buffer[i + 2] == (char)SERIAL_NUMBER[7]) {
        payLoadPin = (rx_buffer[i + 4] - 48);
#if SIMCOM_MODEL == a7672
        if (rx_buffer[(i + 29)] == 49 && is_pb_done == true)
#elif SIMCOM_MODEL == a7670
        if (rx_buffer[(i + 31)] == 49 && is_pb_done == true)
#elif SIMCOM_MODEL == a7677
        if (rx_buffer[(i + 29)] == 49 && is_pb_done == true)
#endif
        {
          printf("-----------ON RELAY %d -----------\r\n", payLoadPin);
          HAL_GPIO_WritePin(GPIO_LOAD_PORT[payLoadPin - 1],
                            GPIO_LOAD_PIN[payLoadPin - 1], 1);
          on_relay++;
          if (on_relay >= NUMBER_LOADS) {
            on_relay = NUMBER_LOADS;
          }
        }
#if SIMCOM_MODEL == a7672
        if (rx_buffer[(i + 29)] == 48 && is_pb_done == true)
#elif SIMCOM_MODEL == a7670
        if (rx_buffer[(i + 31)] == 48 && is_pb_done == true)
#elif SIMCOM_MODEL == a7677
        if (rx_buffer[(i + 29)] == 48 && is_pb_done == true)
#endif
        {
          printf("-----------OFF RELAY %d -----------\r\n", payLoadPin);
          HAL_GPIO_WritePin(GPIO_LOAD_PORT[payLoadPin - 1],
                            GPIO_LOAD_PIN[payLoadPin - 1], 0);
          on_relay--;
          if (on_relay <= 0) {
            on_relay = 0;
          }
        }
      }
    }
    if ((strstr((char *)rx_buffer, "+CMQTTCONNLOST") != NULL) &&
        is_pb_done == true) {
      printf(
          "-----------------Client Disconnect passively!------------------\n");
      check_error_mqtt_via_gsm();
    }
    memset(rx_buffer, '\0', 1024);
  }
  HAL_UARTEx_ReceiveToIdle_IT(&huart1, (uint8_t *)rx_buffer, 1024);
}

void create_JSON(void) {
  cJSON *json = cJSON_CreateObject();
  for (int i = 1; i < NUMBER_LOADS + 1; i++) {
    int statusOfLoad;
    statusOfLoad =
        HAL_GPIO_ReadPin(GPIO_LOAD_PORT[i - 1], GPIO_LOAD_PIN[i - 1]);
    static char payload1[2];
    sprintf(payload1, "%d", i);
    cJSON_AddNumberToObject(json, payload1, statusOfLoad);
  }
  //  Data_Percentage_pin = Level_Pin();
  //  rssi = read_signal_quality();
  cJSON_AddNumberToObject(json, "_gsm_signal_strength", rssi);
  //  cJSON_AddNumberToObject(json, "_battery_level", Data_Percentage_pin);
  char *json_string = cJSON_Print(json);
  if (json_string == NULL) {
    printf("New create error JSON\n");
    cJSON_Delete(json);
    return;
  }
  sprintf(array_json, "%s", json_string);
  // decompress memory
  free(json_string);
  cJSON_Delete(json);
}
