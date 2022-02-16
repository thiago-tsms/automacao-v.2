#ifndef WIFI_CONECTION
#define WIFI_CONECTION

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <esp_wifi.h>
//#include <esp_event.h>
#include <esp_wifi_types.h>

//#include <nvs_flash.h>

//#include "esp_log.h"

//#include "lwip/sockets.h"

/*#define xTime_close_conect pdMS_TO_TICKS(15000)*/

/*const char *TAG_SERVER_WIFI = "log-server-wifi";
const char *TAG_WIFI_STATUS = "log-status-wifi";*/


/* VARIAVEIS */
char *wifi_connection_sta_ssid, *wifi_connection_sta_password;
tcpip_adapter_ip_info_t tcp_server_adapter_ip;
uint16_t tcp_server_adapter_port;

/*TaskHandle_t control_tcp_server_wifi_task;
TaskHandle_t control_tcp_server_wifi_send_msg;
TaskHandle_t control_tcp_server_wifi_recv_msg;
TimerHandle_t time_recv_msg;*/


/* ESCOPO */
void wifi_sta_connection_login(char *ssid, char *password);
void wifi_sta_tcp_server_address(uint32_t *ip, uint32_t *gw, uint32_t *netmask, uint16_t *port);
/*void tcp_server_wifi_nvs();
void tcp_server_start();
static void tcp_server_on_wifi();
static void tcp_server_on_ip();
void tcp_server_wifi_task(void *params);
void tcp_server_wifi_send_msg(void *params);
void tcp_server_wifi_recv_msg(void *params);
void tcp_server_wifi_recv_msg_time();
void cnv_send(params_send_recv_t *params_send_recv, char *dt_string_send);
int cnv_recv(params_send_recv_t *dt_params_send_recv, char *dt_string_send, int len);
void cnv_sdrv_to_lgpl(params_send_recv_t *params_send_recv, lighting_states_t *lighting_states, params_led_t *params_led);
void cnv_lgpl_to_sdrv(params_send_recv_t *params_send_recv, lighting_states_t *lighting_states, params_led_t *params_led);
*/

/* FUNÇÕES */
void wifi_sta_connection_login(char *ssid, char *password){
  wifi_connection_sta_ssid = ssid;
  wifi_connection_sta_password = password;
}

void wifi_sta_tcp_server_address(uint32_t *ip, uint32_t *gw, uint32_t *netmask, uint16_t *port){
  IP4_ADDR(&tcp_server_adapter_ip.ip, ((*ip & 0xFF000000) >> 24), ((*ip & 0x00FF0000) >> 16), ((*ip & 0x0000FF00) >> 8), (*ip & 0x000000FF));
  IP4_ADDR(&tcp_server_adapter_ip.gw, ((*gw & 0xFF000000) >> 24), ((*gw & 0x00FF0000) >> 16), ((*gw & 0x0000FF00) >> 8), (*gw & 0x000000FF));
  IP4_ADDR(&tcp_server_adapter_ip.netmask, ((*netmask & 0xFF000000) >> 24), ((*netmask & 0x00FF0000) >> 16), ((*netmask & 0x0000FF00) >> 8), (*netmask & 0x000000FF));
  tcp_server_adapter_port = *port;
}


/*void tcp_server_wifi_nvs(){
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
    ESP_ERROR_CHECK( nvs_flash_erase() );
    ESP_ERROR_CHECK( nvs_flash_init() );
  }
}*/

/*void tcp_server_start(){
  tcpip_adapter_init();

  ESP_ERROR_CHECK( tcpip_adapter_dhcpc_stop(TCPIP_ADAPTER_IF_STA) );
  ESP_ERROR_CHECK( tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_STA, &tcp_server_adapter_ip) );
  ESP_ERROR_CHECK( esp_event_loop_create_default() );

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
  ESP_ERROR_CHECK( esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &tcp_server_on_wifi, NULL) );
  ESP_ERROR_CHECK( esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &tcp_server_on_ip, NULL) );
  ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
  
  wifi_config_t wifi_config = {.sta = {}};
  for(int i = 0; i < 32; i++) wifi_config.sta.ssid[i] = tcp_server_wifi_ssid[i];
  for(int i = 0; i < 32; i++) wifi_config.sta.password[i] = tcp_server_wifi_password[i];
  
  ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
  ESP_ERROR_CHECK( esp_wifi_start() );

  ESP_ERROR_CHECK( tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, "Wifi Esp8266") );

  ESP_ERROR_CHECK( esp_wifi_connect() );
}*/

/*static void tcp_server_on_wifi(void* arg, esp_event_base_t base, int32_t id, void* data) {
  switch(id){
    case WIFI_EVENT_WIFI_READY:
      ESP_LOGI(TAG_WIFI_STATUS, "Wifi Pronto");
    break;

    case WIFI_EVENT_STA_START:
      xTaskCreate(tcp_server_wifi_task, "tcp_server_wifi_task", 2048, NULL, 2, &control_tcp_server_wifi_task);
      ESP_LOGI(TAG_WIFI_STATUS, "STA Start");
    break;

    case WIFI_EVENT_STA_STOP:
      vTaskDelete(control_tcp_server_wifi_task);
      vTaskDelete(control_tcp_server_wifi_send_msg);
      vTaskDelete(control_tcp_server_wifi_recv_msg);
      xTimerStop(time_recv_msg, 0);

      ESP_LOGI(TAG_WIFI_STATUS, "STA Stop");
    break;

    case WIFI_EVENT_STA_CONNECTED:
      ESP_LOGI(TAG_WIFI_STATUS, "Wifi Conectado");
    break;

    case WIFI_EVENT_STA_DISCONNECTED:
      ESP_LOGI(TAG_WIFI_STATUS, "Wifi Desconectado");
      vTaskDelay(pdMS_TO_TICKS(15000));
      esp_wifi_connect();
    break;
  }
}*/

/*static void tcp_server_on_ip(void *arg, esp_event_base_t base, int32_t id, void *data) {
  ip_event_got_ip_t *d;

  switch(id){
    case IP_EVENT_STA_GOT_IP:
      d = (ip_event_got_ip_t*) data;
      ESP_LOGI(TAG_SERVER_WIFI,"- Got IP %s (event)", ip4addr_ntoa(&d->ip_info.ip));
    break;

    case IP_EVENT_STA_LOST_IP:
      ESP_LOGI(TAG_SERVER_WIFI, "- Perdeu IP");
    break;
  }
}*/

/*void tcp_server_wifi_task(void *params){
  struct sockaddr_in destAddr;
  struct sockaddr_in sourceAddr;
  int listen_sock, sock, err;
  uint addrLen;

    // Adiciona o endereço em que o socket será criado
  destAddr.sin_addr.s_addr = inet_addr(ip4addr_ntoa(&tcp_server_adapter_ip.ip));
  destAddr.sin_family = AF_INET;
  destAddr.sin_port = htons(tcp_server_port);

  ESP_LOGI(TAG_SERVER_WIFI, "TCP Server Iniciado");

  while (1){
      // Cria socket
    listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) {
      ESP_LOGE(TAG_SERVER_WIFI, "Unable to create socket: errno %d", errno);
      break;
    }
    ESP_LOGI(TAG_SERVER_WIFI, "Socket created");

      // Liga socket
    err = bind(listen_sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
    if (err != 0) {
      ESP_LOGE(TAG_SERVER_WIFI, "Socket unable to bind: errno %d", errno);
      break;
    }
    ESP_LOGI(TAG_SERVER_WIFI, "Socket binded");

      // Começa a escutar
    err = listen(listen_sock, 1);
    if (err != 0) {
      ESP_LOGE(TAG_SERVER_WIFI, "Error occured during listen: errno %d", errno);
      break;
    }
    ESP_LOGI(TAG_SERVER_WIFI, "Socket listening");

    while(1){
      addrLen = sizeof(sourceAddr);
      sock = accept(listen_sock, (struct sockaddr *)&sourceAddr, &addrLen);
      if (sock < 0) {
        ESP_LOGE(TAG_SERVER_WIFI, "Unable to accept connection: errno %d", errno);
        break;
      }
      ESP_LOGI(TAG_SERVER_WIFI, "Socket accepted");

        // Inicia task de troca de dados e o contador de close conection
      xTaskCreate(tcp_server_wifi_send_msg, "tcp-server-wifi-send_msg", 1500, &sock, 2, &control_tcp_server_wifi_send_msg);
      xTaskCreate(tcp_server_wifi_recv_msg, "tcp-server-wifi-recv-msg", 2048, &sock, 2, &control_tcp_server_wifi_recv_msg);
      time_recv_msg = xTimerCreate("time_recv_msg", xTime_close_conect, pdFALSE, NULL, tcp_server_wifi_recv_msg_time);
      while(control_tcp_server_wifi_send_msg == NULL || control_tcp_server_wifi_recv_msg == NULL) vTaskDelay(pdMS_TO_TICKS(10));

        // Para a task durante a troca de mensages
      vTaskSuspend(control_tcp_server_wifi_task);

        // Retorna as tasks de send e recv e para o timer
      vTaskDelete(control_tcp_server_wifi_send_msg);
      vTaskDelete(control_tcp_server_wifi_recv_msg);
      xTimerStop(time_recv_msg, 0);

        // Fecha a conexão
      if (sock != -1) {
        ESP_LOGI(TAG_SERVER_WIFI, "Connection closed");
        close(sock);
      }
    }

    close(sock);
    shutdown(sock, 0);
    ESP_LOGE(TAG_SERVER_WIFI, "Shutting down socket and restarting...");
  }
}*/

/*void tcp_server_wifi_recv_msg_time(){
  vTaskResume(control_tcp_server_wifi_task);
}*/

/*void tcp_server_wifi_recv_msg(void *params){
  int *sock = (int*)params, len;
  char rx_buffer[512];
  params_send_recv_t params_send_recv;

  while(1){
    if(xTimerIsTimerActive(time_recv_msg) == pdFALSE) xTimerStart(time_recv_msg, 0);
    else xTimerReset(time_recv_msg, 0);

    len = recv(*sock, rx_buffer, sizeof(rx_buffer) - 1, 0);

      // Error occured during receiving
    if (len < 0) {
      ESP_LOGE(TAG_SERVER_WIFI, "recv failed: errno %d", errno);
      break;

      // Connection closed
    } else if (len == 0) {
      ESP_LOGI(TAG_SERVER_WIFI, "Connection closed");
      break;

      // Data received
    } else {
      int ret = cnv_recv(&params_send_recv, rx_buffer, len);
      if(ret == 0) xQueueSend(recv_data_queue, &params_send_recv, 0);
    }
  }

  vTaskResume(control_tcp_server_wifi_task);
}*/

/*void tcp_server_wifi_send_msg(void *params){
  int *sock = (int*)params;
  bool send_data = false, new_conection = true;
  char dt_string[128];
  params_send_recv_t params_send_recv;

    // Notifica que uma nova conecção foi estabelecida
  xQueueSend(new_conection_queue, &new_conection, 0);

  while(*sock != -1 && sock != NULL){

    while(uxQueueMessagesWaiting(send_data_queue) > 0){
      send_data = true;
      xQueueReceive(send_data_queue, &params_send_recv, 0);
    }

    if(send_data){
      send_data = false;
      cnv_send(&params_send_recv, dt_string);
      //ESP_LOGI("%s \n", dt_string);

      int err = send(*sock, dt_string, strlen(dt_string), 0);
      if (err < 0) {
        ESP_LOGE(TAG_SERVER_WIFI, "Error occured during sending: errno %d", errno);
        //break;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(150));
  }

  vTaskDelete(NULL);
}*/

/*void cnv_send(params_send_recv_t *dt_params_send_recv, char *dt_string_send){
  sprintf(dt_string_send, "[%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d]", 
    dt_params_send_recv->lighting_states.l1, dt_params_send_recv->lighting_states.l2, dt_params_send_recv->lighting_states.led,
    dt_params_send_recv->params_led.effect,
    dt_params_send_recv->lighting_states.mode,
    dt_params_send_recv->params_led.R, dt_params_send_recv->params_led.G, dt_params_send_recv->params_led.B,
    dt_params_send_recv->params_led.amp_r, dt_params_send_recv->params_led.amp_g, dt_params_send_recv->params_led.amp_b,
    dt_params_send_recv->params_led.per_r, dt_params_send_recv->params_led.per_g, dt_params_send_recv->params_led.per_b,
    dt_params_send_recv->params_led.des_r, dt_params_send_recv->params_led.des_g, dt_params_send_recv->params_led.des_b
  );
}*/

/*int cnv_recv(params_send_recv_t *params_send_recv, char *dt_string_send, int len){
  char *txt, extract[128];
  int count_0 = len-1, count_1 = 0, init, end;
  bool ok = false;
  while(count_0 >= 0 && !ok){
    switch(dt_string_send[count_0]){
      case ']':
        count_1 = 0;
        end = count_0;
      break;

      case '|':
        count_1++;
      break;

      case '[':
        if(count_1 == 16){
          init = count_0;
          ok = true;

        } else {
          count_1 = 0;
        }
      break;
    }
    count_0--;
  }

  if(ok){
    int i, j;
    for(i = init+1, j = 0; i < end; i++, j++) extract[j] = dt_string_send[i];
    extract[j] = 0;

    count_0 = 0;
    txt = strtok(extract, "|");
    while(extract != NULL && txt != NULL){
      switch(count_0){
        case 0: params_send_recv->lighting_states.l1 = atoi(txt); break;
        case 1: params_send_recv->lighting_states.l2 = atoi(txt); break;
        case 2: params_send_recv->lighting_states.led = atoi(txt); break;
        case 3: params_send_recv->params_led.effect = atoi(txt); break;
        case 4: params_send_recv->lighting_states.mode = atoi(txt); break;
        case 5: params_send_recv->params_led.R = atoi(txt); break;
        case 6: params_send_recv->params_led.G = atoi(txt); break;
        case 7: params_send_recv->params_led.B = atoi(txt); break;
        case 8: params_send_recv->params_led.amp_r = atoi(txt); break;
        case 9: params_send_recv->params_led.amp_g = atoi(txt); break;
        case 10: params_send_recv->params_led.amp_b = atoi(txt); break;
        case 11: params_send_recv->params_led.per_r = atoi(txt); break;
        case 12: params_send_recv->params_led.per_g = atoi(txt); break;
        case 13: params_send_recv->params_led.per_b = atoi(txt); break;
        case 14: params_send_recv->params_led.des_r = atoi(txt); break;
        case 15: params_send_recv->params_led.des_g = atoi(txt); break;
        case 16: params_send_recv->params_led.des_b = atoi(txt); break;
      }

      txt = strtok(NULL, "|");
      count_0++;
    }

    return 0;
  
  } else {
    return -1;
  }
}*/

/*void cnv_sdrv_to_lgpl(params_send_recv_t *params_send_recv, lighting_states_t *lighting_states, params_led_t *params_led){
  lighting_states->l1 = params_send_recv->lighting_states.l1;
  lighting_states->l2 = params_send_recv->lighting_states.l2;
  lighting_states->led = params_send_recv->lighting_states.led;
  lighting_states->mode = params_send_recv->lighting_states.mode;

  params_led->effect = params_send_recv->params_led.effect;
  params_led->R = params_send_recv->params_led.R;
  params_led->amp_r = params_send_recv->params_led.amp_r;
  params_led->per_r = params_send_recv->params_led.per_r;
  params_led->des_r = params_send_recv->params_led.des_r;
  params_led->G = params_send_recv->params_led.G;
  params_led->amp_g = params_send_recv->params_led.amp_g;
  params_led->per_g = params_send_recv->params_led.per_g;
  params_led->des_g = params_send_recv->params_led.des_g;
  params_led->B = params_send_recv->params_led.B;
  params_led->amp_b = params_send_recv->params_led.amp_b;
  params_led->per_b = params_send_recv->params_led.per_b;
  params_led->des_b = params_send_recv->params_led.des_b;
}*/

/*void cnv_lgpl_to_sdrv(params_send_recv_t *params_send_recv, lighting_states_t *lighting_states, params_led_t *params_led){
  params_send_recv->lighting_states.l1 = lighting_states->l1;
  params_send_recv->lighting_states.l2 = lighting_states->l2;
  params_send_recv->lighting_states.led = lighting_states->led;
  params_send_recv->lighting_states.mode = lighting_states->mode;

  params_send_recv->params_led.effect = params_led->effect;
  params_send_recv->params_led.R = params_led->R;
  params_send_recv->params_led.amp_r = params_led->amp_r;
  params_send_recv->params_led.per_r = params_led->per_r;
  params_send_recv->params_led.des_r = params_led->des_r;
  params_send_recv->params_led.G = params_led->G;
  params_send_recv->params_led.amp_g = params_led->amp_g;
  params_send_recv->params_led.per_g = params_led->per_g;
  params_send_recv->params_led.des_g = params_led->des_g;
  params_send_recv->params_led.B = params_led->B;
  params_send_recv->params_led.amp_b = params_led->amp_b;
  params_send_recv->params_led.per_b = params_led->per_b;
  params_send_recv->params_led.des_b = params_led->des_b;
}*/

#endif