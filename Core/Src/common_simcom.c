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

bool is_at_connect_mqtt = false;
bool is_at_acquier_mqtt = false;
bool is_at_subcribe_topic_mqtt = false;
bool is_at_subcribe_mqtt = false;
bool is_at_topic_puplish_mqtt = false;
bool is_at_data_puplish_mqtt = false;
bool is_at_puplish_mqtt = false;
bool is_at_check_dis_mqtt = false;
bool is_at_disconnect_mqtt = false;
bool is_at_rel_mqtt = false;
bool is_at_stop_mqtt = false;
bool is_inital_check = false;
uint16_t count_errors = 0;
int timeout_pb_done = 40000;

int read_signal_quality(void) {
  send_to_simcom_a76xx("AT+CSQ\r\n");
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
  send_to_simcom_a76xx("ATE0\r\n");
  HAL_Delay(200);
  send_to_simcom_a76xx("AT+CSQ\r\n");
  HAL_Delay(200);
  signal_strength = (rx_data_sim[8] - 48) * 10 + (rx_data_sim[9] - 48);
  if (signal_strength >= 31) {
    rssi = -51;
  } else
    rssi = (signal_strength * 2 - 113);
  is_connect_simcom = 1;
  HAL_Delay(200);
  send_to_simcom_a76xx("AT+CPIN?\r\n");
  HAL_Delay(200);
  if (strstr((char *)rx_data_sim, "+CPIN: READY")) {
    printf("-----------------SIM OK !------------------\n");
  } else
    return 0;
  HAL_Delay(200);
  send_to_simcom_a76xx("AT+CREG?\r\n");
  HAL_Delay(200);
  if (strstr((char *)rx_data_sim, "+CREG: 0,1") ||
      strstr((char *)rx_data_sim, "+CREG: 0,6") ||
      strstr((char *)rx_data_sim, "+CREG: 2,6")) {
    printf("-----------------Network registration OK!------------------\n");
  } else
    return 0;
  send_to_simcom_a76xx("ATI\r\n");
  HAL_Delay(200);
  send_to_simcom_a76xx("AT+CICCID\r\n");
  HAL_Delay(200);
  send_to_simcom_a76xx("AT+CGREG?\r\n");
  HAL_Delay(200);

  if (strstr((char *)rx_data_sim, "+CGREG: 0,1")) {
    printf("-----------------Network registration OK!------------------\n");
  } else
    return 0;
  return 1;
}

int enable_mqtt_on_gsm_modem(void) {
  send_to_simcom_a76xx("AT+CMQTTSTART\r\n");
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
  sprintf(array_at_command, "+CMQTTACCQ: 0,\"%s\",0\r\n", MQTT_CLIENT_ID);
  send_to_simcom_a76xx("AT+CMQTTACCQ?\r\n");
  HAL_Delay(400);
  if (strstr((char *)rx_data_sim, array_at_command) != NULL) {
    printf("-----------------Had acquired------------------\n");
    return 1;
  } else {
    printf("-----------------Haven't got acquire yet------------------\n");
    is_at_acquier_mqtt = false;
  }
  if (is_at_acquier_mqtt == false) {
    sprintf(array_at_command, "AT+CMQTTACCQ=0,\"%s\",0\r\n", MQTT_CLIENT_ID);
    send_to_simcom_a76xx(array_at_command);
    HAL_Delay(200);
    sprintf(array_at_command, "+CMQTTACCQ: 0,\"%s\",0", MQTT_CLIENT_ID);
    HAL_Delay(200);
    if (strstr((char *)rx_data_sim, "OK") != NULL) {
      printf("-----------------Acquire Successfully------------------\n");
      is_at_acquier_mqtt = true;
      return 1;
    } else {
      printf("-----------------Acquire Fail------------------\n");
    }
  }
  return 0;
}

int connect_mqtt_server_by_gsm(void) {
  sprintf(array_at_command, "+CMQTTCONNECT: 0,\"%s:%d\",20,1,\"%s\",\"%s\"\r\n",
          MQTT_HOST, MQTT_PORT, MQTT_USER, MQTT_PASS);
  HAL_Delay(200);
  send_to_simcom_a76xx("AT+CMQTTCONNECT?\r\n");
  HAL_Delay(200);
  if (strstr((char *)rx_data_sim, array_at_command) != NULL) {
    printf("-----------------Connected------------------\n");
    is_at_connect_mqtt = true;
    return 1;
  } else {
    printf("-----------------Not connect yet !------------------\n");
    is_at_connect_mqtt = false;
  }
  if (is_at_connect_mqtt == false) {
    sprintf(array_at_command,
            "AT+CMQTTCONNECT=0,\"%s:%d\",20,1,\"%s\",\"%s\"\r\n", MQTT_HOST,
            MQTT_PORT, MQTT_USER, MQTT_PASS);
    send_to_simcom_a76xx(array_at_command);
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
  sprintf(array_at_command, "%s/snac/%s/#", FARM, SERIAL_NUMBER);
  sprintf(array_at_command, "AT+CMQTTSUBTOPIC=0,%d,1\r\n",
          (int)strlen(array_at_command));
  send_to_simcom_a76xx(array_at_command);
  HAL_Delay(500);
  sprintf(array_at_command, "%s/snac/%s/#", FARM, SERIAL_NUMBER);
  send_to_simcom_a76xx(array_at_command);
  HAL_Delay(500);
  if (strstr((char *)rx_data_sim, "OK") != NULL) {
    printf("-----------------Subscribe Topic Success------------------\n");
    is_at_subcribe_topic_mqtt = true;
  } else {
    printf("-----------------Subscribe Topic Fail------------------\n");
    is_at_subcribe_topic_mqtt = false;
  }
  if (is_at_subcribe_topic_mqtt == true) {
    send_to_simcom_a76xx("AT+CMQTTSUB=0\r\n");
    HAL_Delay(500);
    if (strstr((char *)rx_data_sim, "+CMQTTSUB: 0,0") != NULL) {
      printf("-----------------Subscribe Success !------------------\n");
      is_at_subcribe_mqtt = true;
    } else {
      printf("-----------------Subscribe Fail !------------------\n");
      is_at_subcribe_mqtt = false;
      return 0;
    }
  }
  return 1;
}

int publish_mqtt_via_gsm(void) {
  //  is used to input the topic of a publish message
  led_status('W');
  create_JSON();
  sprintf(array_at_command, "AT+CMQTTTOPIC=0,%d\r\n",
          strlen(MQTT_TOPIC_ACTUATOR_STATUS));
  send_to_simcom_a76xx(array_at_command);
  HAL_Delay(200);
  sprintf(array_at_command, "%s\r\n", MQTT_TOPIC_ACTUATOR_STATUS);
  send_to_simcom_a76xx(array_at_command);
  HAL_Delay(200);
  if (strstr((char *)rx_data_sim, "OK") != NULL) {
    printf("----------------- Sent input the topic of a publish message "
           "success ! ------------------\n");
    is_at_topic_puplish_mqtt = true;
  } else {
    printf("----------------- Sent input the topic of a publish message fail "
           "!------------------\n");
    is_at_topic_puplish_mqtt = false;
  }
  if (is_at_topic_puplish_mqtt) {
    // is used to input the message body of a publish message.
    int lengthOfInformPayload = strlen(array_json);
    sprintf(array_at_command, "AT+CMQTTPAYLOAD=0,%d\r\n",
            lengthOfInformPayload);
    send_to_simcom_a76xx(array_at_command);
    HAL_Delay(200);
    send_to_simcom_a76xx(array_json);
    HAL_Delay(200);
    if (strstr((char *)rx_data_sim, "OK") != NULL) {
      printf("----------------- Sent input the message body of a publish "
             "message ! ------------------\n");
      is_at_data_puplish_mqtt = true;
    } else {
      printf("--- Sent input the message body of a publish fail! "
             "--------\n");
      is_at_data_puplish_mqtt = false;
    }
    if (is_at_data_puplish_mqtt) {
      send_to_simcom_a76xx("AT+CMQTTPUB=0,1,60\r\n");
      HAL_Delay(200);
      if (strstr((char *)rx_data_sim, "+CMQTTPUB: 0,0") != NULL) {
        printf("-----------------Publish Success !------------------\n");
        is_at_puplish_mqtt = true;
        led_status('G');
        return 1;
      } else {
        printf("-----------------Publish fail !------------------\n");
        is_at_puplish_mqtt = false;
      }
    }
  }
  return 0;
}

int check_error_mqtt_via_gsm(void) {
  is_fn_enable_mqtt = false;
  is_fn_connect_mqtt = false;
  is_fn_check_sim = false;
  is_fn_acquier_mqtt = false;
  if (!is_fn_check_sim) {
    is_fn_check_sim = fn_check_signal_simcom();
  } else {
    return 0;
  }

  if (is_fn_check_sim) {
    is_fn_enable_mqtt = enable_mqtt_on_gsm_modem();
  } else {
    return 0;
  }

  if (is_fn_enable_mqtt) {
    is_fn_acquier_mqtt = acquire_gsm_mqtt_client();
  } else {
    return 0;
  }
  if (is_fn_acquier_mqtt) {
    for (int i = 0; i <= 5; i++) {
      is_fn_connect_mqtt = connect_mqtt_server_by_gsm();
      if (is_fn_connect_mqtt) {
        break;
      }
    }
  } else {
    return 0;
  }
  if (is_fn_connect_mqtt) {
    for (int i = 0; i <= 3; i++) {
      is_fn_subcribe_mqtt = subscribe_mqtt_via_gsm();
      if (is_fn_subcribe_mqtt) {
        break;
      }
    }
    if (is_fn_subcribe_mqtt)
      return 1;
    else
      stop_mqtt_via_gsm();
  }
  if (!is_fn_connect_mqtt) {
    stop_mqtt_via_gsm();
  }
  return 0;
}

int stop_mqtt_via_gsm(void) {
  send_to_simcom_a76xx("AT+CMQTTDISC?\r\n");
  HAL_Delay(500);
  if (strstr((char *)rx_data_sim, "+CMQTTDISC: 0,0") != NULL) {
    printf("----------------- Connection! ------------------\n");
    is_at_check_dis_mqtt = true;
  } else {
    printf("----------------- Disconnect! ------------------\n");
    is_at_check_dis_mqtt = false;
    is_at_disconnect_mqtt = true;
  }
  if (is_at_check_dis_mqtt) {
    send_to_simcom_a76xx("AT+CMQTTDISC=0,120\r\n");
    HAL_Delay(500);
    if (strstr((char *)rx_data_sim, "+CMQTTDISC: 0,0") != NULL) {
      printf("----------------- Disconnect successfully! ------------------\n");
      is_at_disconnect_mqtt = true;
    } else
      restart_stm32();
  }
  if (is_at_disconnect_mqtt) {
    send_to_simcom_a76xx("AT+CMQTTREL=0\r\n");
    HAL_Delay(500);
    if (strstr((char *)rx_data_sim, "OK") != NULL) {
      printf("----------------- Release a MQTT client successfully! "
             "------------------\n");
      is_at_rel_mqtt = true;
    } else
      restart_stm32();
  }
  if (is_at_rel_mqtt) {
    send_to_simcom_a76xx("AT+CMQTTSTOP\r\n");
    HAL_Delay(500);
    if (strstr((char *)rx_data_sim, "OK") != NULL) {
      printf("----------------- Stop MQTT service successfully! "
             "------------------\n");
      is_at_stop_mqtt = true;
      return 1;
    } else
      restart_stm32();
  }
  return 0;
}

int update_status(void) {
  for (int i = 1; i <= 10; i++) {
    is_fn_publish_mqtt = publish_mqtt_via_gsm();
    if (is_fn_publish_mqtt) {
      return 1;
    }
  }
  if (!is_fn_publish_mqtt) {
    int temp = 0;
    count_errors = 0;
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
  write_load_statues();
#endif
  NVIC_SystemReset();
}

int init_cricket(void) {
  for (int i = 0; i <= 3; i++) {
    printf("\r\n-----------------INIT MANTIS !------------------\r\n");
    if (is_pb_done == true) {
      if (!is_fn_check_sim) {
        is_fn_check_sim = fn_check_signal_simcom();
      } else
        restart_stm32();
      if (is_fn_check_sim) {
        is_fn_enable_mqtt = enable_mqtt_on_gsm_modem();
      } else
        restart_stm32();
      if (is_fn_enable_mqtt) {
        is_fn_acquier_mqtt = acquire_gsm_mqtt_client();
      }
      if (is_fn_acquier_mqtt) {
        is_fn_connect_mqtt = connect_mqtt_server_by_gsm();
      }
      if (is_fn_connect_mqtt) {
        is_fn_subcribe_mqtt = subscribe_mqtt_via_gsm();
        if (is_fn_subcribe_mqtt) {
          HAL_GPIO_WritePin(GPIOB, LED_STATUS_Pin, GPIO_PIN_SET);
          led_status('G');
          is_connect_mqtt = true;
          is_inital_check = true;
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
  while (is_inital_check == 0 &&
         previousTick + timeout_pb_done > HAL_GetTick()) {
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
      on_relay++;
    } else
      on_relay--;
  }
  if (on_relay >= NUMBER_LOADS) {
    on_relay = NUMBER_LOADS;
  }
  if (on_relay <= 0) {
    // HAL_GPIO_WritePin(ON_OFF_PWM_GPIO_Port, ON_OFF_PWM_Pin, 0);
    on_relay = 0;
  }
  return on_relay;
}
