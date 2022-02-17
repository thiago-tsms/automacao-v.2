#ifndef WIFI_MANAGER
#define WIFI_MANAGER

#include <stdlib.h>
#include <string.h>

#include "esp_wifi.h"
#include <esp_http_server.h>

#include "esp_log.h"

#include "html-page.h"


  /* Nome da Rede */
#define WIFI_MANAGER_SSID_SERVER "WiFi Manager"


  /* TAG de Log */
const char *TAG_WIFI_MANAGER = "wifi-manager";


  /* Variáveis */
httpd_handle_t wifi_manager_server = NULL;
char wifi_manager_server_ssid[32];
char wifi_manager_server_password[32];
uint32_t wifi_manager_server_ip;
uint32_t wifi_manager_server_gateway;
uint32_t wifi_manager_server_netmask;
uint16_t wifi_manager_server_port;
bool wifi_manager_server_new_data = false;


  /* Escopo de Funções */
void wifi_manager_init();
void wifi_manager_start();
void wifi_manager_stop();
esp_err_t wifi_manager_GET_handler(httpd_req_t *req);
esp_err_t wifi_manager_POST_handler(httpd_req_t *req);
void wifi_manager_get_data(char *ssid, char *password, uint32_t *ip, uint32_t *gateway, uint32_t *netmask, uint16_t *port);
bool wifi_manager_get_data_available();
uint32_t wifi_manager_convert_data(char *param);


  // Inicia Wifi manager
void wifi_manager_init(){
  tcpip_adapter_init();
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

  wifi_config_t wifi_config;
  for(int i = 0; i < strlen(WIFI_MANAGER_SSID_SERVER); i++) wifi_config.ap.ssid[i] = WIFI_MANAGER_SSID_SERVER[i];
  wifi_config.ap.ssid_len = strlen(WIFI_MANAGER_SSID_SERVER);
  wifi_config.ap.password[0] = '\0';
  wifi_config.ap.max_connection = 1;
  wifi_config.ap.authmode = WIFI_AUTH_OPEN;
  wifi_config.ap.beacon_interval = 500;

  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG_WIFI_MANAGER, "Wifi Manager Iniciado");
}

  /* Iniciando o Webserver */
void wifi_manager_start(){  
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  httpd_uri_t wifi_manager_handler_GET = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = wifi_manager_GET_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t wifi_manager_handler_POST = {
    .uri       = "/",
    .method    = HTTP_POST,
    .handler   = wifi_manager_POST_handler,
    .user_ctx  = NULL
  };

  ESP_LOGI(TAG_WIFI_MANAGER, "Iniciando server na porta: '%d'", config.server_port);
  if (httpd_start(&wifi_manager_server, &config) == ESP_OK) {
    ESP_LOGI(TAG_WIFI_MANAGER, "Iniciando URI handlers");
    httpd_register_uri_handler(wifi_manager_server, &wifi_manager_handler_GET);
    httpd_register_uri_handler(wifi_manager_server, &wifi_manager_handler_POST);
  
  } else {
    ESP_LOGI(TAG_WIFI_MANAGER, "Error ao iniciar o server!");
  }
}

  // Para o Wifi Manager (Server)
void wifi_manager_stop(){
  if(wifi_manager_server){
    httpd_stop(wifi_manager_server);
  }
}

  // Wifi Manager GET (callback)
esp_err_t wifi_manager_GET_handler(httpd_req_t *req){

  httpd_resp_send(req, html_page, strlen(html_page));

  if(httpd_req_get_hdr_value_len(req, "") == 0) {
    ESP_LOGI(TAG_WIFI_MANAGER, "Página enviada.");
  }

  return ESP_OK;
}

  // Wifi Manager POST (callback)
esp_err_t wifi_manager_POST_handler(httpd_req_t *req){
  char*  buf;
  size_t buf_len;
  char param[32];

  ESP_LOGI(TAG_WIFI_MANAGER, "POST - URL: %s", req->uri);
  ESP_LOGI(TAG_WIFI_MANAGER, "POST - Tamanho da requisição: %d", req->content_len);

  buf_len = req->content_len + 4;
  if(buf_len > 1){
    buf = (char*)malloc(buf_len);
    for(int i = 0; i < buf_len; i++) buf[i] = 0;

    httpd_req_recv(req, buf, buf_len);
    if(buf_len > 0) {
      ESP_LOGV(TAG_WIFI_MANAGER, "Dados: %s", buf);

      if (httpd_query_key_value(buf, "ssid", param, sizeof(param)) == ESP_OK){
        strcpy(wifi_manager_server_ssid, param);
        ESP_LOGV(TAG_WIFI_MANAGER, "SSID: %s", param);
      }

      if (httpd_query_key_value(buf, "password", param, sizeof(param)) == ESP_OK){
        strcpy(wifi_manager_server_password, param);
        ESP_LOGV(TAG_WIFI_MANAGER, "Password: %s", param);
      }

      if (httpd_query_key_value(buf, "ip", param, sizeof(param)) == ESP_OK){
        wifi_manager_server_ip = wifi_manager_convert_data(param);
        ESP_LOGV(TAG_WIFI_MANAGER, "IP: %s", param);
      }

      if (httpd_query_key_value(buf, "gateway", param, sizeof(param)) == ESP_OK){
        wifi_manager_server_gateway = wifi_manager_convert_data(param);
        ESP_LOGV(TAG_WIFI_MANAGER, "Gateway: %s", param);
      }

      if (httpd_query_key_value(buf, "netmask", param, sizeof(param)) == ESP_OK){
        wifi_manager_server_netmask = wifi_manager_convert_data(param);
        ESP_LOGV(TAG_WIFI_MANAGER, "Netmask: %s", param);
      }

      if (httpd_query_key_value(buf, "port", param, sizeof(param)) == ESP_OK){
        wifi_manager_server_port = atoi(param);
        ESP_LOGV(TAG_WIFI_MANAGER, "Port: %s", param);
      }

      free(buf);
      wifi_manager_server_new_data = true;
    }
  }

  httpd_resp_send(req, "1", 1);    

  return ESP_OK;
}

  // Obtem o SSID, Senha, IP, Gateway, Mascara de Subrede, Porta
void wifi_manager_get_data(char *ssid, char *password, uint32_t *ip, uint32_t *gateway, uint32_t *netmask, uint16_t *port){
  wifi_manager_server_new_data = false;

  strcpy(ssid, wifi_manager_server_ssid);
  strcpy(password, wifi_manager_server_password);
  *ip = wifi_manager_server_ip;
  *gateway = wifi_manager_server_gateway;
  *netmask = wifi_manager_server_netmask;
  *port = wifi_manager_server_port;
}

  // Verifica se há novos dados
bool wifi_manager_get_data_available(){
  return wifi_manager_server_new_data;
}

  // Converte os dados de rede de String para 32 bits
uint32_t wifi_manager_convert_data(char *param){

  char *str;
  uint8_t aux[4];
  uint32_t data = 0;

  str = strtok(param, ".");
  for(int i = 0; str != NULL && i < 4; i++){
    aux[i] = (u_int8_t)atoi(str);
    str = strtok(NULL, ".");
  }

  data += aux[0] << 24;
  data += aux[1] << 16;
  data += aux[2] << 8;
  data += aux[3];

  return data;
}

#endif
