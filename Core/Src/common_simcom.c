/*
 * common_simcom.c
 *
 *  Created on: Aug 28, 2024
 *      Author: thuanphat7
 */
#include "cJSON.h"
#include "config.h"
#include "stdbool.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <main.h>
#include <stdbool.h>

bool at_connect_mqtt = false;
bool at_acquier_mqtt = false;
bool at_subcribe_topic_mqtt = false;
bool at_subcribe_mqtt = false;
bool at_topic_puplish_mqtt = false;
bool at_data_puplish_mqtt = false;
bool at_puplish_mqtt = false;
bool at_check_dis_mqtt = false;
bool at_disconnect_mqtt = false;
bool at_rel_mqtt = false;
bool at_stop_mqtt = false;
bool inital_check = false;
uint16_t count_errors = 0;
int timeout_pb_done = 40000;

int read_signal_quality(void) {
  sendingToSimcomA76xx("AT+CSQ\r\n");
  HAL_Delay(200);
  signal_strength = (rx_data_sim[8] - 48) * 10 + (rx_data_sim[9] - 48);
  if (signal_strength >= 31) {
    rssi = -51;
  } else if (signal_strength <= 0) {
    rssi = -113;
  } else
    rssi = (signal_strength * 2 - 113);

  return rssi;
}
float fn_check_signal_simcom(void) {
  printf("-----------------fn_check_signal_simcom------------------\n");
  sendingToSimcomA76xx("ATE0\r\n");
  HAL_Delay(200);
  sendingToSimcomA76xx("AT+CSQ\r\n");
  HAL_Delay(200);
  signal_strength = (rx_data_sim[8] - 48) * 10 + (rx_data_sim[9] - 48);
  if (signal_strength >= 31) {
    rssi = -51;
  } else
    rssi = (signal_strength * 2 - 113);
  is_connect_simcom = 1;
  HAL_Delay(200);
  sendingToSimcomA76xx("AT+CPIN?\r\n");
  HAL_Delay(200);
  if (strstr((char *)rx_data_sim, "+CPIN: READY")) {
    printf("-----------------SIM OK !------------------\n");
  } else
    return 0;
  //  sendingToSimcomA76xx("AT+CREG=2\r\n");
  //  HAL_Delay(5000);
  HAL_Delay(200);
  sendingToSimcomA76xx("AT+CREG?\r\n");
  HAL_Delay(200);
  if (strstr((char *)rx_data_sim, "+CREG: 0,1") ||
      strstr((char *)rx_data_sim, "+CREG: 0,6") ||
      strstr((char *)rx_data_sim, "+CREG: 2,6")) {
    printf("-----------------Network registration OK!------------------\n");
  } else
    return 0;

  sendingToSimcomA76xx("ATI\r\n");
  HAL_Delay(200);
  sendingToSimcomA76xx("AT+CICCID\r\n");
  HAL_Delay(200);
  sendingToSimcomA76xx("AT+CGREG?\r\n");
  HAL_Delay(200);

  if (strstr((char *)rx_data_sim, "+CGREG: 0,1")) {
    printf("-----------------Network registration OK!------------------\n");
  } else
    return 0;
  return 1;
}
int enable_mqtt_on_gsm_modem(void) {
  sendingToSimcomA76xx("AT+CMQTTSTART\r\n");
  HAL_Delay(400);
  if ((strstr((char *)rx_data_sim, "+CMQTTSTART: 0") != NULL) ||
      (strstr((char *)rx_data_sim, "ERROR") != NULL)) {
    printf("-----------------Service have started "
           "successfully------------------\n");
    return 1;
  } else {
    printf("----------------- Start MQTT service fail------------------\n");
    return 0;
  }
  return 0;
}
int acquire_gsm_mqtt_client(void) {
  printf("-----------------acquire_gsm_mqtt_client------------------\n");
  sprintf(AT_COMMAND, "+CMQTTACCQ: 0,\"%s\",0\r\n", MQTT_CLIENT_ID);
  sendingToSimcomA76xx("AT+CMQTTACCQ?\r\n");
  HAL_Delay(400);
  if (strstr((char *)rx_data_sim, AT_COMMAND) != NULL) {
    printf("-----------------Had acquired------------------\n");
    return 1;
  } else {
    printf("-----------------Haven't got acquier yet------------------\n");
    at_acquier_mqtt = false;
  }
  if (at_acquier_mqtt == false) {
    sprintf(AT_COMMAND, "AT+CMQTTACCQ=0,\"%s\",0\r\n", MQTT_CLIENT_ID);
    sendingToSimcomA76xx(AT_COMMAND);
    HAL_Delay(200);
    sprintf(AT_COMMAND, "+CMQTTACCQ: 0,\"%s\",0", MQTT_CLIENT_ID);
    HAL_Delay(200);
    if (strstr((char *)rx_data_sim, "OK") != NULL) {
      printf("-----------------Acquier Successfully------------------\n");
      at_acquier_mqtt = true;
      return 1;
    } else {
      printf("-----------------Acquier Fail------------------\n");
    }
  }
  return 0;
}
int connect_mqtt_server_by_gsm(void) {
  sprintf(AT_COMMAND, "+CMQTTCONNECT: 0,\"%s:%d\",20,1,\"%s\",\"%s\"\r\n",
          MQTT_HOST, MQTT_PORT, MQTT_USER, MQTT_PASS);
  HAL_Delay(200);
  sendingToSimcomA76xx("AT+CMQTTCONNECT?\r\n");
  HAL_Delay(200);
  if (strstr((char *)rx_data_sim, AT_COMMAND) != NULL) {
    printf("-----------------Connected------------------\n");
    at_connect_mqtt = true;
    return 1;
  } else {
    printf("-----------------Not connect yet !------------------\n");
    at_connect_mqtt = false;
  }
  if (at_connect_mqtt == false) {
    sprintf(AT_COMMAND, "AT+CMQTTCONNECT=0,\"%s:%d\",20,1,\"%s\",\"%s\"\r\n",
            MQTT_HOST, MQTT_PORT, MQTT_USER, MQTT_PASS);
    sendingToSimcomA76xx(AT_COMMAND);
    HAL_Delay(500);
    if (strstr((char *)rx_data_sim, "+CMQTTCONNECT: 0,0") != NULL) {
      printf("-----------------Connected MQTT Success------------------\n");
      return 1;
    } else {
      printf("-----------------Connect fail------------------\n");
    }
  }
  return 0;
}
int subscribe_mqtt_via_gsm(void) {
  for (int i = 1; i < NUMBER_LOADS + 1; i++) {
    sprintf(AT_COMMAND, "%s/snac/%s/%d", FARM, SERIAL_NUMBER, i);
    sprintf(AT_COMMAND, "AT+CMQTTSUBTOPIC=0,%d,1\r\n", (int)strlen(AT_COMMAND));
    sendingToSimcomA76xx(AT_COMMAND);
    HAL_Delay(500);
    sprintf(AT_COMMAND, "%s/snac/%s/%d", FARM, SERIAL_NUMBER, i);
    sendingToSimcomA76xx(AT_COMMAND);
    HAL_Delay(500);
    if (strstr((char *)rx_data_sim, "OK") != NULL) {
      printf("-----------------Subscribe Topic Success------------------\n");
      at_subcribe_topic_mqtt = true;
    } else {
      printf("-----------------Subscribe Topic Fail------------------\n");
      at_subcribe_topic_mqtt = false;
    }
    if (at_subcribe_topic_mqtt == true) {
      sendingToSimcomA76xx("AT+CMQTTSUB=0\r\n");
      HAL_Delay(500);
      if (strstr((char *)rx_data_sim, "+CMQTTSUB: 0,0") != NULL) {
        printf("-----------------Subscribe Success !------------------\n");
        at_subcribe_mqtt = true;
      } else {
        printf("-----------------Subscribe Fail !------------------\n");
        at_subcribe_mqtt = false;
        return 0;
      }
    }
  }
  return 1;
}
int publish_mqtt_via_gsm(void) {
  //  is used to input the topic of a publish message
  led_status('W');
  create_JSON();
  sprintf(AT_COMMAND, "AT+CMQTTTOPIC=0,%d\r\n",
          strlen(MQTT_TOPIC_ACTUATOR_STATUS));
  sendingToSimcomA76xx(AT_COMMAND);
  HAL_Delay(200);
  sprintf(AT_COMMAND, "%s\r\n", MQTT_TOPIC_ACTUATOR_STATUS);
  sendingToSimcomA76xx(AT_COMMAND);
  HAL_Delay(200);
  if (strstr((char *)rx_data_sim, "OK") != NULL) {
    printf("----------------- Sent input the topic of a publish message "
           "success ! ------------------\n");
    at_topic_puplish_mqtt = true;
  } else {
    printf("----------------- Sent input the topic of a publish message fail "
           "!------------------\n");
    at_topic_puplish_mqtt = false;
  }
  if (at_topic_puplish_mqtt) {
    // is used to input the message body of a publish message.
    int lengthOfInformPayload = strlen(array_json);
    sprintf(AT_COMMAND, "AT+CMQTTPAYLOAD=0,%d\r\n", lengthOfInformPayload);
    sendingToSimcomA76xx(AT_COMMAND);
    HAL_Delay(200);
    sendingToSimcomA76xx(array_json);
    HAL_Delay(200);
    if (strstr((char *)rx_data_sim, "OK") != NULL) {
      printf("----------------- Sent input the message body of a publish "
             "message ! ------------------\n");
      at_data_puplish_mqtt = true;
    } else {
      printf("----------------- Sent input the message body of a publish fail! "
             "------------------\n");
      at_data_puplish_mqtt = false;
    }
    if (at_data_puplish_mqtt) {
      sendingToSimcomA76xx("AT+CMQTTPUB=0,1,60\r\n");
      HAL_Delay(200);
      if (strstr((char *)rx_data_sim, "+CMQTTPUB: 0,0") != NULL) {
        printf("-----------------Publish Success !------------------\n");
        at_puplish_mqtt = true;
        led_status('G');
        return 1;
      } else {
        printf("-----------------Publish fail !------------------\n");
        at_puplish_mqtt = false;
      }
    }
  }
  return 0;
}
int check_error_mqtt_via_gsm(void) {
  fn_enable_mqtt = false;
  fn_connect_mqtt = false;
  fn_check_sim = false;
  fn_acquier_mqtt = false;
  if (!fn_check_sim) {
    fn_check_sim = fn_check_signal_simcom();
  } else {
    return 0;
  }

  if (fn_check_sim) {
    fn_enable_mqtt = enable_mqtt_on_gsm_modem();
  } else {
    return 0;
  }

  if (fn_enable_mqtt) {
    fn_acquier_mqtt = acquire_gsm_mqtt_client();
  } else {
    return 0;
  }
  if (fn_acquier_mqtt) {
    for (int i = 0; i <= 5; i++) {
      fn_connect_mqtt = connect_mqtt_server_by_gsm();
      if (fn_connect_mqtt) {
        break;
      }
    }
  } else {
    return 0;
  }
  if (fn_connect_mqtt) {
    for (int i = 0; i <= 3; i++) {
      fn_subcribe_mqtt = subscribe_mqtt_via_gsm();
      if (fn_subcribe_mqtt) {
        break;
      }
    }
    if (fn_subcribe_mqtt)
      return 1;
    else
      stop_mqtt_via_gsm();
  }
  if (!fn_connect_mqtt) {
    stop_mqtt_via_gsm();
  }
  return 0;
}
int stop_mqtt_via_gsm(void) {
  sendingToSimcomA76xx("AT+CMQTTDISC?\r\n");
  HAL_Delay(500);
  if (strstr((char *)rx_data_sim, "+CMQTTDISC: 0,0") != NULL) {
    printf("----------------- Connection! ------------------\n");
    at_check_dis_mqtt = true;
  } else {
    printf("----------------- Disconnect! ------------------\n");
    at_check_dis_mqtt = false;
    at_disconnect_mqtt = true;
  }
  if (at_check_dis_mqtt) {
    sendingToSimcomA76xx("AT+CMQTTDISC=0,120\r\n");
    HAL_Delay(500);
    if (strstr((char *)rx_data_sim, "+CMQTTDISC: 0,0") != NULL) {
      printf("----------------- Disconnect successfully! ------------------\n");
      at_disconnect_mqtt = true;
    } else
      restart_stm32();
  }
  if (at_disconnect_mqtt) {
    sendingToSimcomA76xx("AT+CMQTTREL=0\r\n");
    HAL_Delay(500);
    if (strstr((char *)rx_data_sim, "OK") != NULL) {
      printf("----------------- Release a MQTT client successfully! "
             "------------------\n");
      at_rel_mqtt = true;
    } else
      restart_stm32();
  }
  if (at_rel_mqtt) {
    sendingToSimcomA76xx("AT+CMQTTSTOP\r\n");
    HAL_Delay(500);
    if (strstr((char *)rx_data_sim, "OK") != NULL) {
      printf("----------------- Stop MQTT service successfully! "
             "------------------\n");
      at_stop_mqtt = true;
      return 1;
    } else
      restart_stm32();
  }
  return 0;
}
int update_status(void) {
  for (int i = 1; i <= 10; i++) {
    fn_publish_mqtt = publish_mqtt_via_gsm();
    if (fn_publish_mqtt) {
      return 1;
    }
  }
  if (!fn_publish_mqtt) {
    int temp = 0;
    count_errors=0;
    for (int i = 1; i <= 5; i++) {
      temp = check_error_mqtt_via_gsm();
      if (!temp) {
        count_errors++;
        printf("-----------------UPDATE FAIL %d!------------------\n",
               count_errors);
        if (count_errors >= 4) {
          restart_stm32();
        }
      } else {
        count_errors = 0;
        break;
      }
    }
  }
  return 1;
}
void restart_stm32(void) {
  printf("\r\n-----------------Restart STM32------------------\r\n");
  printf("\r\n-----------------GOOD BYE !------------------\r\n");
  stop_mqtt_via_gsm();
#if SAVE_LOAD
  read_statusload();
#endif
  NVIC_SystemReset();
}
int init_cricket(void) {
  for (int i = 0; i <= 3; i++) {
    printf("\r\n-----------------INIT MANTIS !------------------\r\n");
    if (is_pb_done == true) {
      if (!fn_check_sim) {
        fn_check_sim = fn_check_signal_simcom();
      } else
        restart_stm32();
      if (fn_check_sim) {
        fn_enable_mqtt = enable_mqtt_on_gsm_modem();
      } else
        restart_stm32();
      if (fn_enable_mqtt) {
        fn_acquier_mqtt = acquire_gsm_mqtt_client();
      }
      if (fn_acquier_mqtt) {
        fn_connect_mqtt = connect_mqtt_server_by_gsm();
      }
      if (fn_connect_mqtt) {
        fn_subcribe_mqtt = subscribe_mqtt_via_gsm();
        if (fn_subcribe_mqtt) {
          HAL_GPIO_WritePin(GPIOB, LED_STATUS_Pin, GPIO_PIN_SET);
          led_status('G');
          is_connect_mqtt = true;
          inital_check = true;
          return 1;
        } else {
          check_error_mqtt_via_gsm();
          // isConnectedMQTT = false;
        }
      }
      printf("-----------------Complete initial check ------------------");
    }
  }
  restart_stm32();
  return 0;
}
int event_wait_function(void) {
  previousTick = HAL_GetTick();
  while (inital_check == 0 && previousTick + timeout_pb_done > HAL_GetTick()) {
    if (strstr((char *)rx_data_sim, "PB DONE")) {
      // is_pb_done = 1;
      led_status('B');
      return 1;
    }
#if SIMCOM_MODEL == a7677
    if (strstr((char *)rx_data_sim, "EPS PDN ACT 1")) {
      is_pb_done = 1;
      led_status('B');
      HAL_Delay(8000);
      return 1;
    }
#endif
  }
  if (is_connect_simcom == 0) {
    NVIC_SystemReset();
  }

  return 0;
}
int check_active_payload(void) {
  for (int i = 1; i <= 4; i++) {
    static int temp = 0;
    temp = HAL_GPIO_ReadPin(GPIO_LOAD_PORT[i - 1], GPIO_LOAD_PIN[i - 1]);
    if (temp == 1) {
      onReay++;
    } else
      onReay--;
  }
  if (onReay >= NUMBER_LOADS) {
    onReay = NUMBER_LOADS;
  }
  if (onReay <= 0) {
    // HAL_GPIO_WritePin(ON_OFF_PWM_GPIO_Port, ON_OFF_PWM_Pin, 0);
    onReay = 0;
  }
  return onReay;
}
