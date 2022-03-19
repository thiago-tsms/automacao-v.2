#ifndef WIFI_STA_SOCKET_SERVER
#define WIFI_STA_SOCKET_SERVER

#include <string.h>

#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_wifi_types.h>

#include "esp_log.h"
#include <errno.h>

#include "lwip/sockets.h"


  /* Task Delay */
#define xDelay_Wifi_STA_Socket_Server_Control_Task pdMS_TO_TICKS(250)
#define xDelay_Wifi_STA_Socket_Server_Send_msg_Task pdMS_TO_TICKS(200)
#define xDelay_Wifi_STA_Socket_Server_Recv_msg_Task pdMS_TO_TICKS(180)


  /* Task Stack */
#define usStackDepth_Wifi_STA_Socket_Server_Control_Task 1250
#define usStackDepth_Wifi_STA_Socket_Server_Send_msg_Task 1250
#define usStackDepth_Wifi_STA_Socket_Server_Recv_msg_Task 2250


  /* Parametros Software Timer */
#define xTimer_wifi_sta_socket_server_time_limit pdMS_TO_TICKS(2000)
TimerHandle_t xHandleTimer_wifi_sta_socket_server_time_limit;


  // Número máximo de conexões aquardando para poder se conectar
#define WIFI_STA_SOCKET_SERVER_MAX_CONECTION 1


  /* Nome da conexão */
#define TAG_WIFI_STA_SOCKET_SERVER_NAME "WiFi ESP8266"


  /* TAG de Log */
const char *TAG_WIFI_STA_SOCKET_SERVER = "log-wifi_sta-socket-server";


  /* Parâmetros de Socket */
int sock;


  /* Variáveis */
char *wifi_sta_socket_server_ssid, *wifi_sta_socket_server_password;
tcpip_adapter_ip_info_t wifi_sta_socket_server_adapter_ip;
uint16_t wifi_sta_socket_server_adapter_port;


  /* ESCOPO */
void wifi_sta_socket_server_login(char *ssid, char *password);
void wifi_sta_socket_server_address(uint32_t *ip, uint32_t *gw, uint32_t *netmask, uint16_t *port);
void wifi_sta_socket_server_start();
void wifi_sta_socket_server_stop();
void wifi_sta_socket_server_event_handler_on_wifi(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
void wifi_sta_socket_server_event_handler_on_ip(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void wifi_sta_socket_server_control_task(void *params);
void wifi_sta_socket_server_send_msg_task(void *params);
void wifi_sta_socket_server_recv_msg_task(void *params);
void wifi_sta_socket_server_time_limit(TimerHandle_t xTimer);


  /* Parametros Task */
TaskHandle_t xHandleTask_wifi_sta_socket_server_control;
TaskHandle_t xHandleTask_wifi_sta_socket_server_send_msg;
TaskHandle_t xHandleTask_wifi_sta_socket_server_recv_msg;


  /* FUNÇÕES */

  //Informações para login na rede Wifi
void wifi_sta_socket_server_login(char *ssid, char *password){
  wifi_sta_socket_server_ssid = ssid;
  wifi_sta_socket_server_password = password;

  ESP_LOGV(TAG_WIFI_STA_SOCKET_SERVER, "SSID e Password inseridos");
}

  // Informações para o servidor
void wifi_sta_socket_server_address(uint32_t *ip, uint32_t *gw, uint32_t *netmask, uint16_t *port){
  IP4_ADDR(&wifi_sta_socket_server_adapter_ip.ip, ((*ip & 0xFF000000) >> 24), ((*ip & 0x00FF0000) >> 16), ((*ip & 0x0000FF00) >> 8), (*ip & 0x000000FF));
  IP4_ADDR(&wifi_sta_socket_server_adapter_ip.gw, ((*gw & 0xFF000000) >> 24), ((*gw & 0x00FF0000) >> 16), ((*gw & 0x0000FF00) >> 8), (*gw & 0x000000FF));
  IP4_ADDR(&wifi_sta_socket_server_adapter_ip.netmask, ((*netmask & 0xFF000000) >> 24), ((*netmask & 0x00FF0000) >> 16), ((*netmask & 0x0000FF00) >> 8), (*netmask & 0x000000FF));
  wifi_sta_socket_server_adapter_port = *port;

  ESP_LOGV(TAG_WIFI_STA_SOCKET_SERVER, "IP, GATEWAY, NETMASK e PORT inseridos");
}

  // Inicia STA Socket Server
void wifi_sta_socket_server_start(){

  tcpip_adapter_init();

  ESP_ERROR_CHECK( tcpip_adapter_dhcpc_stop(TCPIP_ADAPTER_IF_STA) );
  ESP_ERROR_CHECK( tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_STA, &wifi_sta_socket_server_adapter_ip) );
  ESP_ERROR_CHECK( esp_event_loop_create_default() );

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_sta_socket_server_event_handler_on_wifi, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_sta_socket_server_event_handler_on_ip, NULL));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  
  wifi_config_t wifi_config = {.sta = {}};
  for(int i = 0; i < strlen(wifi_sta_socket_server_ssid); i++) wifi_config.sta.ssid[i] = wifi_sta_socket_server_ssid[i];
  for(int i = 0; i < strlen(wifi_sta_socket_server_password); i++) wifi_config.sta.password[i] = wifi_sta_socket_server_password[i];
  
  ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
  ESP_ERROR_CHECK( esp_wifi_start() );

  ESP_ERROR_CHECK( tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, TAG_WIFI_STA_SOCKET_SERVER_NAME) );

  ESP_ERROR_CHECK( esp_wifi_connect() );

  ESP_LOGI(TAG_WIFI_STA_SOCKET_SERVER, "STA Socket Server Iniciado");
}

  // Para o STA Socker Server
void wifi_sta_socket_server_stop(){
  esp_wifi_stop();
}

  // Handler IP (Conexão dos Clientes)
void wifi_sta_socket_server_event_handler_on_ip(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data){
  ip_event_got_ip_t *event_data_cast = (ip_event_got_ip_t*) event_data;

  switch(event_id){
    case IP_EVENT_STA_GOT_IP:
      ESP_LOGI(TAG_WIFI_STA_SOCKET_SERVER,"Conectado a rede com IP: %s", ip4addr_ntoa(&event_data_cast->ip_info.ip));
    break;

    case IP_EVENT_STA_LOST_IP:
      ESP_LOGI(TAG_WIFI_STA_SOCKET_SERVER, "Desconectado da Rede");
    break;
  }
}

  // Handler WIFI (Conexão da Rede)
void wifi_sta_socket_server_event_handler_on_wifi(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data){
  switch(event_id){
    case WIFI_EVENT_WIFI_READY:
      ESP_LOGI(TAG_WIFI_STA_SOCKET_SERVER, "Wifi Pronto");
    break;

    case WIFI_EVENT_STA_START:
      xTaskCreate(wifi_sta_socket_server_control_task, "wifi-sta-socket-server-control-task", usStackDepth_Wifi_STA_Socket_Server_Control_Task, NULL, 2, &xHandleTask_wifi_sta_socket_server_control);
      ESP_LOGI(TAG_WIFI_STA_SOCKET_SERVER, "STA Start");
    break;

    case WIFI_EVENT_STA_STOP:
      vTaskDelete(xHandleTask_wifi_sta_socket_server_control);
      vTaskDelete(xHandleTask_wifi_sta_socket_server_send_msg);
      vTaskDelete(xHandleTask_wifi_sta_socket_server_recv_msg);
      xTimerStop(xHandleTimer_wifi_sta_socket_server_time_limit, 0);
      xTaskNotify(xHandleTask_central_control, NOTIFY_WIFI_DISCONNECT, eSetBits);

      ESP_LOGI(TAG_WIFI_STA_SOCKET_SERVER, "STA Stop");
    break;

    case WIFI_EVENT_STA_CONNECTED:
      ESP_LOGI(TAG_WIFI_STA_SOCKET_SERVER, "Wifi Conectado");
    break;

    case WIFI_EVENT_STA_DISCONNECTED:
      ESP_LOGI(TAG_WIFI_STA_SOCKET_SERVER, "Wifi Desconectado");
      vTaskDelay(pdMS_TO_TICKS(8000));
      esp_wifi_connect();
    break;
  }
}

  // Task de controle do socket [Estabelece a conexão via socket e inicia as taks de envio e recepção de mensagens]
void wifi_sta_socket_server_control_task(void *params){
  //UBaseType_t uxHighWaterMark;

  struct sockaddr_in addr_socket;
  struct sockaddr_in source_addr;
  int listen_sock, err;
  uint addr_len;

    // Adiciona o endereço em que o socket será criado
  addr_socket.sin_addr.s_addr = inet_addr(ip4addr_ntoa(&wifi_sta_socket_server_adapter_ip.ip));
  addr_socket.sin_family = AF_INET;
  addr_socket.sin_port = htons(wifi_sta_socket_server_adapter_port);

  ESP_LOGI(TAG_WIFI_STA_SOCKET_SERVER, "Iniciando Socket");

  while (1){
      // Cria socket
    listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if(listen_sock < 0) {
      ESP_LOGE(TAG_WIFI_STA_SOCKET_SERVER, "Incapaz de criar socket: errno %d", errno);
      break;
    }
    ESP_LOGI(TAG_WIFI_STA_SOCKET_SERVER, "Socket criado (created)");

      // Liga socket
    err = bind(listen_sock, (struct sockaddr *)&addr_socket, sizeof(addr_socket));
    if(err != 0) {
      ESP_LOGE(TAG_WIFI_STA_SOCKET_SERVER, "Incapaz de vincular (bind) socket: errno %d", errno);
      break;
    }
    ESP_LOGI(TAG_WIFI_STA_SOCKET_SERVER, "Socket vinculado (binded)");

      // Começa a escutar
    err = listen(listen_sock, WIFI_STA_SOCKET_SERVER_MAX_CONECTION);
    if(err != 0) {
      ESP_LOGE(TAG_WIFI_STA_SOCKET_SERVER, "Erro ocorrido durante a escuta (listen): errno %d", errno);
      break;
    }
    ESP_LOGI(TAG_WIFI_STA_SOCKET_SERVER, "Socket escutando (listening)");

    while(1){
      addr_len = sizeof(source_addr);
      sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
      if (sock < 0) {
        ESP_LOGE(TAG_WIFI_STA_SOCKET_SERVER, "Incapas de aceitar conexão: errno %d", errno);
        break;
      }
      ESP_LOGI(TAG_WIFI_STA_SOCKET_SERVER, "Socket aceito");

        // Inicia task de troca de dados e o contador de close conection
      xTaskCreate(wifi_sta_socket_server_send_msg_task, "wifi-sta-socket-server-send-msg-task", usStackDepth_Wifi_STA_Socket_Server_Send_msg_Task, &sock, 2, &xHandleTask_wifi_sta_socket_server_send_msg);
      xTaskCreate(wifi_sta_socket_server_recv_msg_task, "wifi-sta-socket-server-recv-msg-task", usStackDepth_Wifi_STA_Socket_Server_Recv_msg_Task, &sock, 2, &xHandleTask_wifi_sta_socket_server_recv_msg);
      xHandleTimer_wifi_sta_socket_server_time_limit = xTimerCreate("wifi-sta-socket-server-time-limit", xTimer_wifi_sta_socket_server_time_limit, pdFALSE, NULL, wifi_sta_socket_server_time_limit);
      while(xHandleTask_wifi_sta_socket_server_recv_msg == NULL || xHandleTask_wifi_sta_socket_server_recv_msg == NULL) vTaskDelay(pdMS_TO_TICKS(50));
      xTaskNotify(xHandleTask_central_control, NOTIFY_WIFI_CONECT, eSetBits);

        // Para a task durante a troca de mensages
      vTaskSuspend(xHandleTask_wifi_sta_socket_server_control);

        // Retorna as tasks de send e recv e para o timer
      vTaskDelete(xHandleTask_wifi_sta_socket_server_send_msg);
      vTaskDelete(xHandleTask_wifi_sta_socket_server_recv_msg);
      xTimerStop(xHandleTimer_wifi_sta_socket_server_time_limit, 0);
      xTaskNotify(xHandleTask_central_control, NOTIFY_WIFI_DISCONNECT, eSetBits);

        // Fecha a conexão
      if (sock != -1) {
        ESP_LOGI(TAG_WIFI_STA_SOCKET_SERVER, "Conexão fechada (closed)");
        close(sock);
      }

      /*uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
      printf("STACK wifi-sta-socket-server-control-task -- alocado: %d || livre %d \n", usStackDepth_Wifi_STA_Socket_Server_Control_Task, uxHighWaterMark);*/

      vTaskDelay(xDelay_Wifi_STA_Socket_Server_Control_Task);
    }

    close(sock);
    shutdown(sock, 0);
    ESP_LOGE(TAG_WIFI_STA_SOCKET_SERVER, "Socker desligado e reiniciando...");

    vTaskDelay(xDelay_Wifi_STA_Socket_Server_Control_Task);
  }
}

  // Task de recebimento de mensagens
void wifi_sta_socket_server_send_msg_task(void *params){
  //UBaseType_t uxHighWaterMark;

  data_json_t json_data;
  char *json_string;
  int err;

  while(1){
    while(uxQueueMessagesWaiting(queue_wifi_send) > 0){
      xQueueReceive(queue_wifi_send, &json_data, 0);
      json_string = json_serialize(&json_data);

      err = send(sock, json_string, strlen(json_string), 0);
      if (err < 0) ESP_LOGE(TAG_WIFI_STA_SOCKET_SERVER, "Erro dorante o envio da mensagem: errno %d", errno);
      else {
        if(xTimerIsTimerActive(xHandleTimer_wifi_sta_socket_server_time_limit) == pdFALSE) xTimerStart(xHandleTimer_wifi_sta_socket_server_time_limit, 0);
        else xTimerReset(xHandleTimer_wifi_sta_socket_server_time_limit, 0);
      }

      json_string_free(json_string);
    }

    /*uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    printf("STACK wifi-sta-socket-server-send-msg-task -- alocado: %d || livre %d \n", usStackDepth_Wifi_STA_Socket_Server_Send_msg_Task, uxHighWaterMark);*/

    vTaskDelay(xDelay_Wifi_STA_Socket_Server_Send_msg_Task);
  }
}

  // Task de envio de mensagens
void wifi_sta_socket_server_recv_msg_task(void *params){
  //UBaseType_t uxHighWaterMark;

  data_json_t json_data;
  char json_string[512];
  size_t json_string_len = sizeof(json_string);
  int len;

  while(1){
    len = recv(sock, json_string, json_string_len, 0);
    if (len < 0) {
      ESP_LOGE(TAG_WIFI_STA_SOCKET_SERVER, "Erro na recepção dos dados (recv failed): errno %d", errno);
      break;

      // Connection closed
    } else if (len == 0) {
      ESP_LOGI(TAG_WIFI_STA_SOCKET_SERVER, "Conexão fechada (Connection closed)");
      break;

      // Data received
    } else {
      json_data = json_deserialize(json_string);
      xQueueSend(queue_wifi_recv, &json_data, 0);

      if(xTimerIsTimerActive(xHandleTimer_wifi_sta_socket_server_time_limit) == pdFALSE) xTimerStart(xHandleTimer_wifi_sta_socket_server_time_limit, 0);
      else xTimerReset(xHandleTimer_wifi_sta_socket_server_time_limit, 0);
    }

    /*uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    printf("STACK wifi-sta-socket-server-recv-msg-task -- alocado: %d || livre %d \n", usStackDepth_Wifi_STA_Socket_Server_Recv_msg_Task, uxHighWaterMark);*/

    vTaskDelay(xDelay_Wifi_STA_Socket_Server_Recv_msg_Task);
  }

  vTaskResume(xHandleTask_wifi_sta_socket_server_control);
}

  // Fecha Socket com limite de tempo
void wifi_sta_socket_server_time_limit(TimerHandle_t xTimer){
  vTaskResume(xHandleTask_wifi_sta_socket_server_control);
}
#endif