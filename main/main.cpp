#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

#include "driver/gpio.h"
#include "driver/hw_timer.h"
#include "driver/pwm.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "esp_log.h"
#include "esp_system.h"

#include "data-types.h"
#include "json-parse.h"
#include "wifi-manager.h"
#include "wifi-sta-socket-server.h"


  /* Task Delay */
#define xDelay_Central_Control_Task pdMS_TO_TICKS(100)
#define xDelay_Lighting_Control_Task pdMS_TO_TICKS(100)
#define xDelay_Wifi_Control_Task pdMS_TO_TICKS(120)

#define xDelay_Task_Start pdMS_TO_TICKS(50)
#define xDelay_Config_Wifi_Connection pdMS_TO_TICKS(100)


  /* TAG de Log */
const char *TAG_MAIN = "log-main";
const char *TAG_NVS = "log-nvs";
const char *TAG_TASK_CONTROL = "log-task-control";
const char *TAG_LD_CONTROL = "log-ld-control";
const char *TAG_WIFI_CONTROL = "log-wifi-control";


  /* Mapeamento IO */
#define BT1 GPIO_NUM_5
#define BT2 GPIO_NUM_13
#define BT3 GPIO_NUM_16
#define LD1 GPIO_NUM_15
#define LD2 GPIO_NUM_0
#define LDR GPIO_NUM_4
#define LDG GPIO_NUM_12
#define LDB GPIO_NUM_14


  /* Parametros do PWM */
#define PWM_PERIOD (1000) //(1Khz)
uint32_t pwm_chanel[3] = {LDR, LDG, LDB};
uint32_t pwm_duties[3] = {0, 0, 0};
float pwm_phase[3] = {0, 0, 0};


  /* Configuração Hardware Timer */
#define TIME_INTERRUPT 90 //(ms)


  /* Parametros Software Timer */
#define xTimer_nvs_storage_lighting_states pdMS_TO_TICKS(10000)
#define xTimer_nvs_nvs_storage_led_states_rgb pdMS_TO_TICKS(10000)
TimerHandle_t xHandleTimer_nvs_storage_lighting_states;
TimerHandle_t xHandleTimer_nvs_storage_led_states_rgb;


  /* Parametros Queue e Set */
#define QUEUE_LENGHT_INTERRUPT_TIMER 6
#define QUEUE_LENGHT_WIFI_SEND 6
#define QUEUE_LENGHT_WIFI_RECV 8
#define QUEUE_SIZE_INTERRUPT_TIMER sizeof(action_interrupt_timer_t)
#define QUEUE_SIZE_WIFI_SEND sizeof(data_json_t)
#define QUEUE_SIZE_WIFI_RECV sizeof(data_json_t)
#define QUEUESET_LENGHT_RECV (QUEUE_LENGHT_INTERRUPT_TIMER + QUEUE_LENGHT_WIFI_RECV + 0)
QueueHandle_t queue_interrupt_timer;
QueueSetHandle_t queueSet_control_recv;


  /* Parametros Semaphore */
#define xTicksToWait_semaphore_lighting_states pdMS_TO_TICKS(150)
#define xTicksToWait_semaphore_led_states pdMS_TO_TICKS(150)
SemaphoreHandle_t semaphore_lighting_states;
SemaphoreHandle_t semaphore_led_states;


  /* Parametros Notify */
#define NOTIFY_LED_ON_OFF 0x1
#define NOTIFY_LED_STATE 0x2
#define NOTIFY_LED_ALL 0x3


  /* Parametros NVS */
#define NVS_PARTITION_NAME_WIFI "WIFI"
#define NVS_KEY_WIFI_SSID "SSID"
#define NVS_KEY_WIFI_PASSWORD "PASS"
#define NVS_KEY_WIFI_IP "IP"
#define NVS_KEY_WIFI_GATEWAY "GATW"
#define NVS_KEY_WIFI_NETMASK "NETM"
#define NVS_KEY_WIFI_PORT "PORT"

#define NVS_PARTITION_NAME_LIGHTING_STATES "LG_ST"
#define NVS_KEY_L1 "L1"
#define NVS_KEY_L2 "L2"
#define NVS_KEY_LED "LED"
#define NVS_KEY_LED_MODO "MODO"

#define NVS_PARTITION_NAME_RGB_STATES "RGB_ST"
#define NVS_KEY_RGB_0 "RGB_0"
#define NVS_KEY_RGB_1 "RGB_1"
#define NVS_KEY_RGB_2 "RGB_2"
#define NVS_KEY_RGB_3 "RGB_3"


  /* Parametros WiFi */
char wifi_ssid[32];
char wifi_password[32];
uint32_t wifi_ip;
uint32_t wifi_gateway;
uint32_t wifi_netmask;
uint16_t wifi_port;
bool wifi_status = false;

  /* Parâmetros Configuração de conexão */
#define TIME_TO_SEQUENCE pdMS_TO_TICKS(4000)


  /* Variáveis Gerais */
lighting_states_t lighting_states = DEFAULT_LIGHTING_STATES;
led_states_rgb_t led_states_rgb[4] = DEFAULT_LED_STATES_RGB;


  /* Escopo de funções */
void nvs_start();
void peripherals_config();
void config_wifi_connection();
static void sweep_switches(void *params);

void nvs_load_data();
void nvs_storage_config_wifi_connection();
void nvs_storage_lighting_states(TimerHandle_t xTimer);
void nvs_storage_led_states_rgb(TimerHandle_t xTimer);

void central_control_task(void *params);
void lighting_control_task(void *params);

actions_enum select_button_action(action_interrupt_timer_t *btn_action);
actions_enum select_wifi_action(data_json_t *data_json);
void update_states(actions_enum action, data_json_t *data_json);
void update_lighting(actions_enum action);
void update_storage_nvs(actions_enum action);
void update_wifi(actions_enum action);

bool compare_storage_rgb(led_states_rgb_t *rgb_0, led_states_rgb_t *rgb_1);


extern "C" {
  void app_main(void);
}


void app_main(){

  ESP_LOGI(TAG_MAIN, "Inicializando...");


    // Cria semáforo
  ESP_LOGI(TAG_MAIN, "Criando Semáforo");
  semaphore_lighting_states = xSemaphoreCreateBinary();
  xSemaphoreGive(semaphore_lighting_states);
  semaphore_led_states = xSemaphoreCreateBinary();
  xSemaphoreGive(semaphore_led_states);


    // Cria Queue
  ESP_LOGI(TAG_MAIN, "Criando Queue");
  queue_interrupt_timer = xQueueCreate(QUEUE_LENGHT_INTERRUPT_TIMER, QUEUE_SIZE_INTERRUPT_TIMER);
  vQueueAddToRegistry(queue_interrupt_timer, "queue-interrupt-timer");
  queue_wifi_recv = xQueueCreate(QUEUE_LENGHT_WIFI_RECV, QUEUE_SIZE_WIFI_RECV);
  vQueueAddToRegistry(queue_wifi_recv, "queue-wifi-recv");

  queue_wifi_send = xQueueCreate(QUEUE_LENGHT_WIFI_SEND, QUEUE_SIZE_WIFI_SEND);
  vQueueAddToRegistry(queue_wifi_send, "queue-wifi-send");


    // Cria Set
  ESP_LOGI(TAG_MAIN, "Criando Set");
  queueSet_control_recv = xQueueCreateSet(QUEUESET_LENGHT_RECV);
  xQueueAddToSet(queue_interrupt_timer, queueSet_control_recv);
  xQueueAddToSet(queue_wifi_recv, queueSet_control_recv);


    // Inicia NVS
  ESP_LOGI(TAG_MAIN, "Iniciando NVS");
  nvs_start();
  nvs_load_data();


    // Configura periféricos
  ESP_LOGI(TAG_MAIN, "Configurando IO - PWM - Hardware Timer");
  peripherals_config();


    // Teste para modo de configuração de rede
  config_wifi_connection();


    // Inicia tarefas (task)
  ESP_LOGI(TAG_MAIN, "Iniciando Tasks");
  xTaskCreate(central_control_task, "central-control-task", 5000, NULL, 2, &xHandleTask_central_control);
  xTaskCreate(lighting_control_task, "lighting-control-task", 2000, NULL, 1, &xHandleTask_lighting_control);


      // Inicialização API Software Timer
  xHandleTimer_nvs_storage_lighting_states = xTimerCreate("nvs-storage-lighting-states", xTimer_nvs_storage_lighting_states, pdFALSE, NULL, nvs_storage_lighting_states);
  xHandleTimer_nvs_storage_led_states_rgb = xTimerCreate("nvs-storage-led-states-rgb", xTimer_nvs_nvs_storage_led_states_rgb, pdFALSE, NULL, nvs_storage_led_states_rgb);


    // Inicialização Wifi
  wifi_sta_socket_server_login(wifi_ssid, wifi_password);
  wifi_sta_socket_server_address(&wifi_ip, &wifi_gateway, &wifi_netmask, &wifi_port);
  wifi_sta_socket_server_start();


  ESP_LOGI(TAG_MAIN, "Inicialização Finalizada");
  ESP_LOGI(TAG_MAIN, "Sistema em execução");
}


/* -- -- -- Inicializações e Configurações -- -- -- */

  // Inicia o NVS
void nvs_start(){
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());
  }
  ESP_ERROR_CHECK(err);
}

  // Carrega informações da memória (NVS)
void nvs_load_data(){
  nvs_handle nvs_partition;
  esp_err_t err;
  size_t nvs_size;

    // Carrega dados para o WiFi
  err = nvs_open(NVS_PARTITION_NAME_WIFI, NVS_READONLY, &nvs_partition);
  if(err == ESP_OK){
    ESP_LOGV(TAG_MAIN, "Carregando dados NVS WiFi");

    nvs_size = sizeof(wifi_ssid);
    nvs_get_str(nvs_partition, NVS_KEY_WIFI_SSID, wifi_ssid, &nvs_size);

    nvs_size = sizeof(wifi_password);
    nvs_get_str(nvs_partition, NVS_KEY_WIFI_PASSWORD, wifi_password, &nvs_size);

    nvs_get_u32(nvs_partition, NVS_KEY_WIFI_IP, &wifi_ip);
    nvs_get_u32(nvs_partition, NVS_KEY_WIFI_GATEWAY, &wifi_gateway);
    nvs_get_u32(nvs_partition, NVS_KEY_WIFI_NETMASK, &wifi_netmask);
    nvs_get_u16(nvs_partition, NVS_KEY_WIFI_PORT, &wifi_port);

    // Inicializa partição WiFi
  } else if(err == ESP_ERR_NVS_NOT_INITIALIZED){
    nvs_flash_init_partition(NVS_PARTITION_NAME_WIFI);
  }
  nvs_close(nvs_partition);


    // Carrega dados da Iluminação
  err = nvs_open(NVS_PARTITION_NAME_LIGHTING_STATES, NVS_READONLY, &nvs_partition);
  if(err == ESP_OK){
    uint8_t nvs_uint8_data;
    
    ESP_LOGV(TAG_MAIN, "Carregando dados NVS Iluminação");

    if(nvs_get_u8(nvs_partition, NVS_KEY_L1, &nvs_uint8_data) == ESP_OK) lighting_states.l1 = nvs_uint8_data;
    if(nvs_get_u8(nvs_partition, NVS_KEY_L2, &nvs_uint8_data) == ESP_OK) lighting_states.l2 = nvs_uint8_data;
    if(nvs_get_u8(nvs_partition, NVS_KEY_LED, &nvs_uint8_data) == ESP_OK) lighting_states.led = nvs_uint8_data;
    nvs_get_u8(nvs_partition, NVS_KEY_LED_MODO, &(lighting_states.mode));

    // Inicializa partição Iluminação
  } else if(err == ESP_ERR_NVS_NOT_INITIALIZED){
    nvs_flash_init_partition(NVS_PARTITION_NAME_LIGHTING_STATES);
  }
  nvs_close(nvs_partition);


    // Carrega dados de RGB
  err = nvs_open(NVS_PARTITION_NAME_RGB_STATES, NVS_READONLY, &nvs_partition);
  if(err == ESP_OK){

    ESP_LOGV(TAG_MAIN, "Carregando dados RGB");

    nvs_size = sizeof(led_states_rgb_t);
    nvs_get_blob(nvs_partition, NVS_KEY_RGB_0, &(led_states_rgb[0]), &nvs_size);
    nvs_get_blob(nvs_partition, NVS_KEY_RGB_1, &(led_states_rgb[1]), &nvs_size);
    nvs_get_blob(nvs_partition, NVS_KEY_RGB_2, &(led_states_rgb[2]), &nvs_size);
    nvs_get_blob(nvs_partition, NVS_KEY_RGB_3, &(led_states_rgb[3]), &nvs_size);

    // Inicializa partição RGB
  } else if(err == ESP_ERR_NVS_NOT_INITIALIZED){
    nvs_flash_init_partition(NVS_PARTITION_NAME_RGB_STATES);
  }
  nvs_close(nvs_partition);
}

  // Configuração de entrada, saída, interrupção e PWM
void peripherals_config(){

    // Configuração do GPIO
  gpio_config_t io_conf;
  uint32_t pin_mask;

    // Saidas IO
  pin_mask = ((1ULL << LD1) | (1ULL << LD2));
  io_conf.pin_bit_mask = pin_mask;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&io_conf);

    // Entradas IO
  pin_mask = ((1ULL << BT1) | (1ULL << BT2) | (1ULL << BT3));
  io_conf.pin_bit_mask = pin_mask;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&io_conf);

    // Configura PWM
  pwm_init(PWM_PERIOD, pwm_duties, 3, pwm_chanel);
  pwm_set_phases(pwm_phase);
  pwm_stop(0);

    // Configuração Interrupção de Timer
  hw_timer_init(sweep_switches, NULL);
  hw_timer_alarm_us((TIME_INTERRUPT * 1000), true);
}


/* -- -- -- Função de configuração de rede -- -- -- */

  // Configura informações de rede
void config_wifi_connection(){
  if(gpio_get_level(BT1) && gpio_get_level(BT2) && gpio_get_level(BT3)){
    vTaskDelay(TIME_TO_SEQUENCE);

      if(gpio_get_level(BT1) && !gpio_get_level(BT2) && gpio_get_level(BT3)){
        ESP_LOGI(TAG_MAIN, "Iniciando modo de configuração de rede");

        wifi_manager_init();
        wifi_manager_start();

        while(1){
          vTaskDelay(xDelay_Config_Wifi_Connection);

          if(wifi_manager_get_data_available()){
            wifi_manager_get_data(wifi_ssid, wifi_password, &wifi_ip, &wifi_gateway, &wifi_netmask, &wifi_port);
            nvs_storage_config_wifi_connection();
          }
        }

        wifi_manager_stop();
    }
  }
}

  // Salva dados para Wifi e Socket na memoria (NVS)
void nvs_storage_config_wifi_connection(){
  nvs_handle nvs_partition;
  bool success = true;

  if(nvs_open(NVS_PARTITION_NAME_WIFI, NVS_READWRITE, &nvs_partition) == ESP_OK){
    if(nvs_set_str(nvs_partition, NVS_KEY_WIFI_SSID, wifi_ssid) != ESP_OK) success = false;
    if(nvs_set_str(nvs_partition, NVS_KEY_WIFI_PASSWORD, wifi_password) != ESP_OK) success = false;
    if(nvs_set_u32(nvs_partition, NVS_KEY_WIFI_IP, wifi_ip) != ESP_OK) success = false;
    if(nvs_set_u32(nvs_partition, NVS_KEY_WIFI_GATEWAY, wifi_gateway) != ESP_OK) success = false;
    if(nvs_set_u32(nvs_partition, NVS_KEY_WIFI_NETMASK, wifi_netmask) != ESP_OK) success = false;
    if(nvs_set_u16(nvs_partition, NVS_KEY_WIFI_PORT, wifi_port) != ESP_OK) success = false;

    if(success){
      nvs_commit(nvs_partition);
      ESP_LOGV(TAG_MAIN, "Dados Wifi e Socket salvos com sucesso");
    }
  }
  nvs_close(nvs_partition);
}


/* -- -- -- Funções de ISR e Software Timer API -- -- -- */

  // Funções de interrupção de timer para varredura de botões (ISR)
static void sweep_switches(void *params){
  static bool bt1 = false, bt2 = false, bt3 = false;
  static uint16_t time_bt1 = 0, time_bt2 = 0, time_bt3 = 0;
  action_interrupt_timer_t queue_data;

    // Leitura do botão 1
  if(gpio_get_level(BT1)){
    bt1 = true;
    time_bt1 += TIME_INTERRUPT;
  
  } else if(bt1){
    queue_data.id = btn_id_enum::BT_1;
    queue_data.time = time_bt1;
    xQueueSendToBackFromISR(queue_interrupt_timer, &queue_data, 0);

    bt1 = false;
    time_bt1 = 0;
  }

    // Leitura do botão 2
  if(gpio_get_level(BT2)){
    bt2 = true;
    time_bt2 += TIME_INTERRUPT;
  
  } else if(bt2){
    queue_data.id = btn_id_enum::BT_2;
    queue_data.time = time_bt2;
    xQueueSendToBackFromISR(queue_interrupt_timer, &queue_data, 0);

    bt2 = false;
    time_bt2 = 0;
  }

    // Leitura do botão 3
  if(gpio_get_level(BT3)){
    bt3 = true;
    time_bt3 += TIME_INTERRUPT;
  
  } else if(bt3){
    queue_data.id = btn_id_enum::BT_3;
    queue_data.time = time_bt3;
    xQueueSendToBackFromISR(queue_interrupt_timer, &queue_data, 0);

    bt3 = false;
    time_bt3 = 0;
  }
}

  // Salva estados na memória NVS (Software Timer API)
void nvs_storage_lighting_states(TimerHandle_t xTimer){
  nvs_handle nvs_partition;
  esp_err_t err;

  ESP_LOGV(TAG_LD_CONTROL, "Salvando estados na memória (NVS)");

    // Abre partição
  if(nvs_open(NVS_PARTITION_NAME_LIGHTING_STATES, NVS_READWRITE, &nvs_partition) == ESP_OK){
    uint8_t nvs_uint8_data;
    
      // Verifica e salva L1
    err = nvs_get_u8(nvs_partition, NVS_KEY_L1, &nvs_uint8_data);
    if(err == ESP_ERR_NVS_NOT_FOUND || lighting_states.l1 != nvs_uint8_data){
      nvs_uint8_data = lighting_states.l1;
      if(nvs_set_u8(nvs_partition, NVS_KEY_L1, nvs_uint8_data) == ESP_OK) nvs_commit(nvs_partition);
    }

      // Verifica e salva L2
    err = nvs_get_u8(nvs_partition, NVS_KEY_L2, &nvs_uint8_data);
    if(err == ESP_ERR_NVS_NOT_FOUND || lighting_states.l2 != nvs_uint8_data){
      nvs_uint8_data = lighting_states.l2;
      if(nvs_set_u8(nvs_partition, NVS_KEY_L2, nvs_uint8_data) == ESP_OK) nvs_commit(nvs_partition);
    }

      // Verifica e salva L3
    err = nvs_get_u8(nvs_partition, NVS_KEY_LED, &nvs_uint8_data);
    if(err == ESP_ERR_NVS_NOT_FOUND || lighting_states.led != nvs_uint8_data){
      nvs_uint8_data = lighting_states.led;
      if(nvs_set_u8(nvs_partition, NVS_KEY_LED, nvs_uint8_data) == ESP_OK) nvs_commit(nvs_partition);
    }

      // Verifica e salva MODO
    err = nvs_get_u8(nvs_partition, NVS_KEY_LED_MODO, &nvs_uint8_data);
    if(err == ESP_ERR_NVS_NOT_FOUND || lighting_states.mode != nvs_uint8_data){
      nvs_uint8_data = lighting_states.mode;
      if(nvs_set_u8(nvs_partition, NVS_KEY_LED_MODO, nvs_uint8_data) == ESP_OK) nvs_commit(nvs_partition);
    }
  }
  nvs_close(nvs_partition);
}

  // Salva parâmetros dos efeitos do RGB na memória (NVS)
void nvs_storage_led_states_rgb(TimerHandle_t xTimer){
  nvs_handle nvs_partition;
  size_t nvs_size;
  esp_err_t err;
  led_states_rgb_t led_states;

  ESP_LOGV(TAG_LD_CONTROL, "Salvando parâmetros RGB na memória (NVS)");

  // Abre partição
  if(nvs_open(NVS_PARTITION_NAME_RGB_STATES, NVS_READWRITE, &nvs_partition) == ESP_OK){

      // RGB 0
    err = nvs_get_blob(nvs_partition, NVS_KEY_RGB_0, &led_states, &nvs_size);
    if(err == ESP_ERR_NVS_NOT_FOUND || compare_storage_rgb(&led_states, &(led_states_rgb[0]))){
      if(nvs_set_blob(nvs_partition, NVS_KEY_RGB_0, &(led_states_rgb[0]), sizeof(led_states_rgb_t)) == ESP_OK) nvs_commit(nvs_partition);
    }

      // RGB 1
    err = nvs_get_blob(nvs_partition, NVS_KEY_RGB_1, &led_states, &nvs_size);
    if(err == ESP_ERR_NVS_NOT_FOUND || compare_storage_rgb(&led_states, &(led_states_rgb[1]))){
      if(nvs_set_blob(nvs_partition, NVS_KEY_RGB_1, &(led_states_rgb[1]), sizeof(led_states_rgb_t)) == ESP_OK) nvs_commit(nvs_partition);
    }

      // RGB 2
    err = nvs_get_blob(nvs_partition, NVS_KEY_RGB_2, &led_states, &nvs_size);
    if(err == ESP_ERR_NVS_NOT_FOUND || compare_storage_rgb(&led_states, &(led_states_rgb[2]))){
      if(nvs_set_blob(nvs_partition, NVS_KEY_RGB_2, &(led_states_rgb[2]), sizeof(led_states_rgb_t)) == ESP_OK) nvs_commit(nvs_partition);
    }

      // RGB 3
    err = nvs_get_blob(nvs_partition, NVS_KEY_RGB_3, &led_states, &nvs_size);
    if(err == ESP_ERR_NVS_NOT_FOUND || compare_storage_rgb(&led_states, &(led_states_rgb[3]))){
      if(nvs_set_blob(nvs_partition, NVS_KEY_RGB_3, &(led_states_rgb[3]), sizeof(led_states_rgb_t)) == ESP_OK) nvs_commit(nvs_partition);
    }
  }
  nvs_close(nvs_partition);
}


/* -- -- --  Tarefas (Task) -- -- -- */

  // Task de controle geral
void central_control_task(void *params){
  vTaskDelay(xDelay_Task_Start);

  QueueSetMemberHandle_t set_recv;

  action_interrupt_timer_t btn_action;
  data_json_t data_json;
  actions_enum action;

  uint32_t notify_data;
  

    // Atializa estados iniciais
  update_lighting(actions_enum::UPDATE_ALL_STATES);

  TickType_t reference_time_delay;
  while(true){

    if(xTaskNotifyWait(0, 0xffffffff, &notify_data, 0) == pdTRUE){
      if(notify_data == NOTIFY_WIFI_CONECT) wifi_status = true;
      else if(notify_data == NOTIFY_WIFI_DISCONNECT) wifi_status = false;
    }

      // Efetua a leitura da queue
    while((set_recv = xQueueSelectFromSet(queueSet_control_recv, 0)) != NULL){
      action = actions_enum::NOTHING;

        // Dados da interrupção
      if(set_recv == (QueueSetMemberHandle_t)queue_interrupt_timer){
        xQueueReceive(queue_interrupt_timer, &btn_action, 0);
        action = select_button_action(&btn_action);
        
        // Dados Wifi
      } else if(set_recv == (QueueSetMemberHandle_t)queue_wifi_recv){
        xQueueReceive(queue_wifi_recv, &data_json, 0);
        action = select_wifi_action(&data_json);
      }

      if(action != actions_enum::NOTHING){
        update_states(action, &data_json);
        update_storage_nvs(action);
        update_lighting(action);
        update_wifi(action);
      }
    }

    vTaskDelayUntil(&reference_time_delay, xDelay_Central_Control_Task);
  }
}

  // Controla os periféricos
void lighting_control_task(void *params){
  vTaskDelay(xDelay_Task_Start);
  
  float time = 0;
  uint32_t notify_data;
  uint16_t R = 0, G = 0, B = 0;
  uint8_t mode = 0;
  bool status_led = false;  
  led_states_rgb_t led_states;

  while(true){

      // Recebe uma notificação
    if(xTaskNotifyWait(0, 0xffffffff, &notify_data, 0) == pdTRUE){

      if(notify_data == NOTIFY_LED_ON_OFF || notify_data == NOTIFY_LED_ALL){
        status_led = lighting_states.led;
      }
      
      if(notify_data == NOTIFY_LED_STATE || notify_data == NOTIFY_LED_ALL){
        mode = lighting_states.mode;
        led_states.fx = led_states_rgb[mode].fx;
        led_states.r.amp = led_states_rgb[mode].r.amp;
        led_states.r.per = led_states_rgb[mode].r.per;
        led_states.r.des = led_states_rgb[mode].r.des;
        led_states.g.amp = led_states_rgb[mode].g.amp;
        led_states.g.per = led_states_rgb[mode].g.per;
        led_states.g.des = led_states_rgb[mode].g.des;
        led_states.b.amp = led_states_rgb[mode].b.amp;
        led_states.b.per = led_states_rgb[mode].b.per;
        led_states.b.des = led_states_rgb[mode].b.des;
      }

      if(!status_led){
        pwm_stop(0);

      } else if(!led_states.fx){
        pwm_set_duty(0, led_states.r.amp);
        pwm_set_duty(1, led_states.g.amp);
        pwm_set_duty(2, led_states.b.amp);
        pwm_start();
      }
    }

    if(status_led && led_states.fx){
      time += xDelay_Lighting_Control_Task;
        
      R = (led_states.r.amp/2) + (led_states.r.amp/2) * sin((led_states.r.per * time / 10000) + led_states.r.des / 1000);
      G = (led_states.g.amp/2) + (led_states.g.amp/2) * sin((led_states.g.per * time / 10000) + led_states.g.des / 1000);
      B = (led_states.b.amp/2) + (led_states.b.amp/2) * sin((led_states.b.per * time / 10000) + led_states.b.des / 1000);
      
      pwm_set_duty(0, R);
      pwm_set_duty(1, G);
      pwm_set_duty(2, B);
      pwm_start();
    }

    vTaskDelay(xDelay_Lighting_Control_Task);
  }
}


  /* -- -- -- Função de Identificação de ação -- -- -- */

  // Interpreta a ação dos botões
actions_enum select_button_action(action_interrupt_timer_t *btn_action){
  static bool flag = false;

    // BT 1
  if(btn_action->id == btn_id_enum::BT_1){
    ESP_LOGV(TAG_LD_CONTROL, "Botão 1 precionado");

    if(!flag) return actions_enum::ISR_L1;
    else return actions_enum::ISR_MODO_UP;

    // BT 2
  } else if(btn_action->id == btn_id_enum::BT_2){
    ESP_LOGV(TAG_LD_CONTROL, "Botão 2 precionado");

    if(!flag) return actions_enum::ISR_L2;
    else return actions_enum::ISR_MODO_DOWN;

    // BT 3
  } else if(btn_action->id == btn_id_enum::BT_3){
    ESP_LOGV(TAG_LD_CONTROL, "Botão 3 precionado");

    if(btn_action->time > 20000) esp_restart();
    else if(btn_action->time > 2000) flag = !flag;
    else if(!flag) return actions_enum::ISR_LED;
    else flag = !flag;
  }
    
  return actions_enum::NOTHING;
}

  // Interpreta ações do WIFI
actions_enum select_wifi_action(data_json_t *data_json){
  switch(data_json->id){   
    case id_json_t::CONNECTION_STATUS_REQUEST: return actions_enum::WIFI_CONNECTION_STATUS_OK;
    case id_json_t::REQUEST_ALL_STATES: return actions_enum::WIFI_REQUEST_ALL_STATES;
    case id_json_t::DATA_TRANSACTION:
      switch(data_json->mask & ~(mask_json_t::ID)){
        case mask_json_t::L1: return actions_enum::WIFI_L1;
        case mask_json_t::L2: return actions_enum::WIFI_L2;
        case mask_json_t::LED: return actions_enum::WIFI_LED;
        case mask_json_t::MODE: return actions_enum::WIFI_MODO;
        case mask_json_t::FX: return actions_enum::WIFI_FX;
        case mask_json_t::AMP_R: return actions_enum::WIFI_AMP_R;
        case mask_json_t::PER_R: return actions_enum::WIFI_PER_R;
        case mask_json_t::DES_R: return actions_enum::WIFI_DES_R;
        case mask_json_t::AMP_G: return actions_enum::WIFI_AMP_G;
        case mask_json_t::PER_G: return actions_enum::WIFI_PER_G;
        case mask_json_t::DES_G: return actions_enum::WIFI_DES_G;
        case mask_json_t::AMP_B: return actions_enum::WIFI_AMP_B;
        case mask_json_t::PER_B: return actions_enum::WIFI_PER_B;
        case mask_json_t::DES_B: return actions_enum::WIFI_DES_B;
        case mask_json_t::RGB: return actions_enum::WIFI_RGB;
        case mask_json_t::ALL: return actions_enum::UPDATE_ALL_STATES;
        default: return actions_enum::NOTHING;
      }
    default: return actions_enum::NOTHING;
  }
}


  /* -- -- -- Função de Realização de ação -- -- -- */

  // Atualiza os estados com base nas ações (Iluminação e RGB)
void update_states(actions_enum action, data_json_t *data_json){

    // L1 (ISR)
  if(action == actions_enum::ISR_L1){
    if(xSemaphoreTake(semaphore_lighting_states, xTicksToWait_semaphore_lighting_states) == pdPASS)
      lighting_states.l1 = !lighting_states.l1;
    xSemaphoreGive(semaphore_lighting_states);
    ESP_LOGV(TAG_LD_CONTROL, "Modifica estado de L1 para: %d", lighting_states.l1);
  

    // L2 (ISR)
  } else if(action == actions_enum::ISR_L2){
    if(xSemaphoreTake(semaphore_lighting_states, xTicksToWait_semaphore_lighting_states) == pdPASS)
      lighting_states.l2 = !lighting_states.l2;
    xSemaphoreGive(semaphore_lighting_states);
    ESP_LOGV(TAG_LD_CONTROL, "Modifica estado de L2 para: %d", lighting_states.l2);
  

    // LED (ISR)
  } else if(action == actions_enum::ISR_LED){
    if(xSemaphoreTake(semaphore_lighting_states, xTicksToWait_semaphore_lighting_states) == pdPASS)
      lighting_states.led = !lighting_states.led;
    xSemaphoreGive(semaphore_lighting_states);
    ESP_LOGV(TAG_LD_CONTROL, "Modifica estado de LED para: %d", lighting_states.led);
  

    //LED - MODE UP (ISR)
  } else if(action == actions_enum::ISR_MODO_UP){
    if(xSemaphoreTake(semaphore_lighting_states, xTicksToWait_semaphore_lighting_states) == pdPASS){
      lighting_states.mode++;
      if(lighting_states.mode > 3){
        if(lighting_states.mode > 100) lighting_states.mode = 3;
        else lighting_states.mode = 0;
      }
    }
    xSemaphoreGive(semaphore_lighting_states);
    ESP_LOGV(TAG_LD_CONTROL, "Modifica MODE para: %d", lighting_states.mode);
  

    //LED - MODE DOWN (ISR)
  } else if(action == actions_enum::ISR_MODO_DOWN){
    if(xSemaphoreTake(semaphore_lighting_states, xTicksToWait_semaphore_lighting_states) == pdPASS){
      lighting_states.mode--;
      if(lighting_states.mode > 3){
        if(lighting_states.mode > 100) lighting_states.mode = 3;
        else lighting_states.mode = 0;
      }
    }
    xSemaphoreGive(semaphore_lighting_states);
    ESP_LOGV(TAG_LD_CONTROL, "Modifica MODE para: %d", lighting_states.l1);
  

    // L1 (WIFI)
  } else if(action == actions_enum::WIFI_L1){
    if(xSemaphoreTake(semaphore_lighting_states, xTicksToWait_semaphore_lighting_states) == pdPASS){
      lighting_states.l1 = data_json->ls.l1;
    }
    xSemaphoreGive(semaphore_lighting_states);
    ESP_LOGV(TAG_LD_CONTROL, "Modifica L1 para: %d", lighting_states.l2);


    // L2 (WIFI)
  } else if(action == actions_enum::WIFI_L2){
    if(xSemaphoreTake(semaphore_lighting_states, xTicksToWait_semaphore_lighting_states) == pdPASS){
      lighting_states.l2 = data_json->ls.l2;
    }
    xSemaphoreGive(semaphore_lighting_states);
    ESP_LOGV(TAG_LD_CONTROL, "Modifica L2 para: %d", lighting_states.led);


    // LED (WIFI)
  } else if(action == actions_enum::WIFI_LED){
    if(xSemaphoreTake(semaphore_lighting_states, xTicksToWait_semaphore_lighting_states) == pdPASS){
      lighting_states.led = data_json->ls.led;
    }
    xSemaphoreGive(semaphore_lighting_states);
    ESP_LOGV(TAG_LD_CONTROL, "Modifica LED para: %d", lighting_states.mode);


    //LED - MODE (WIFI)
  } else if(action == actions_enum::WIFI_MODO){
    if(xSemaphoreTake(semaphore_lighting_states, xTicksToWait_semaphore_lighting_states) == pdPASS){
      if(data_json->ls.mode < 4 && data_json->ls.mode >= 0)
        lighting_states.mode = data_json->ls.mode;
    }
    xSemaphoreGive(semaphore_lighting_states);
    ESP_LOGV(TAG_LD_CONTROL, "Modifica MODE para: %d", lighting_states.mode);
  
  
    // FX (WIFI)
  } else if(action == actions_enum::WIFI_FX){
    if(xSemaphoreTake(semaphore_led_states, xTicksToWait_semaphore_led_states) == pdPASS){
      led_states_rgb[lighting_states.mode].fx = data_json->rgb.fx;
    }
    xSemaphoreGive(semaphore_led_states);
    ESP_LOGV(TAG_LD_CONTROL, "Modifica FX para: %d", led_states_rgb[lighting_states.mode].fx);


    // AMP R
  } else if(action == actions_enum::WIFI_AMP_R){
    if(xSemaphoreTake(semaphore_led_states, xTicksToWait_semaphore_led_states) == pdPASS){
      if(data_json->rgb.r.amp < 1024)
        led_states_rgb[lighting_states.mode].r.amp = data_json->rgb.r.amp;
    }
    xSemaphoreGive(semaphore_led_states);
    ESP_LOGV(TAG_LD_CONTROL, "Modifica AMP R para: %d", led_states_rgb[lighting_states.mode].r.amp);


    // PER R (WIFI)
  } else if(action == actions_enum::WIFI_PER_R){
    if(xSemaphoreTake(semaphore_led_states, xTicksToWait_semaphore_led_states) == pdPASS){
      led_states_rgb[lighting_states.mode].r.per = data_json->rgb.r.per;
    }
    xSemaphoreGive(semaphore_led_states);
    ESP_LOGV(TAG_LD_CONTROL, "Modifica PER R para: %d", led_states_rgb[lighting_states.mode].r.per);


    // DES R (WIFI)
  } else if(action == actions_enum::WIFI_DES_R){
    if(xSemaphoreTake(semaphore_led_states, xTicksToWait_semaphore_led_states) == pdPASS){
      led_states_rgb[lighting_states.mode].r.des = data_json->rgb.r.des;
    }
    xSemaphoreGive(semaphore_led_states);
    ESP_LOGV(TAG_LD_CONTROL, "Modifica DES R para: %d", led_states_rgb[lighting_states.mode].r.des);


    // AMP G (WIFI)
  } else if(action == actions_enum::WIFI_AMP_G){
    if(xSemaphoreTake(semaphore_led_states, xTicksToWait_semaphore_led_states) == pdPASS){
      if(data_json->rgb.g.amp < 1024)
        led_states_rgb[lighting_states.mode].g.amp = data_json->rgb.g.amp;
      
    }
    xSemaphoreGive(semaphore_led_states);
    ESP_LOGV(TAG_LD_CONTROL, "Modifica AMP G para: %d", led_states_rgb[lighting_states.mode].g.amp);


    // PER G (WIFI)
  } else if(action == actions_enum::WIFI_PER_G){
    if(xSemaphoreTake(semaphore_led_states, xTicksToWait_semaphore_led_states) == pdPASS){
      led_states_rgb[lighting_states.mode].g.per = data_json->rgb.g.per;
    }
    xSemaphoreGive(semaphore_led_states);
    ESP_LOGV(TAG_LD_CONTROL, "Modifica PER G para: %d", led_states_rgb[lighting_states.mode].g.per);


    // DES G (WIFI)
  } else if(action == actions_enum::WIFI_DES_G){
    if(xSemaphoreTake(semaphore_led_states, xTicksToWait_semaphore_led_states) == pdPASS){
      led_states_rgb[lighting_states.mode].g.des = data_json->rgb.g.des;
    }
    xSemaphoreGive(semaphore_led_states);
    ESP_LOGV(TAG_LD_CONTROL, "Modifica DES G para: %d", led_states_rgb[lighting_states.mode].g.des);


    // AMP B (WIFI)
  } else if(action == actions_enum::WIFI_AMP_B){
    if(xSemaphoreTake(semaphore_led_states, xTicksToWait_semaphore_led_states) == pdPASS){
      if(data_json->rgb.b.amp < 1024)
        led_states_rgb[lighting_states.mode].b.amp = data_json->rgb.b.amp;
    }
    xSemaphoreGive(semaphore_led_states);
    ESP_LOGV(TAG_LD_CONTROL, "Modifica AMP B para: %d", led_states_rgb[lighting_states.mode].b.amp);


    // PER B (WIFI)
  } else if(action == actions_enum::WIFI_PER_B){
    if(xSemaphoreTake(semaphore_led_states, xTicksToWait_semaphore_led_states) == pdPASS){
      led_states_rgb[lighting_states.mode].b.per = data_json->rgb.b.per;
    }
    xSemaphoreGive(semaphore_led_states);
    ESP_LOGV(TAG_LD_CONTROL, "Modifica PER B para: %d", led_states_rgb[lighting_states.mode].b.per);


    // DES B (WIFI)
  } else if(action == actions_enum::WIFI_DES_B){
    if(xSemaphoreTake(semaphore_led_states, xTicksToWait_semaphore_led_states) == pdPASS){
      led_states_rgb[lighting_states.mode].b.des = data_json->rgb.b.des;
    }
    xSemaphoreGive(semaphore_led_states);
    ESP_LOGV(TAG_LD_CONTROL, "Modifica DES B para: %d", led_states_rgb[lighting_states.mode].b.des);
  }
}

  // Atualiza os periféricos com base nos estados
void update_lighting(actions_enum action){

    // L1 - ON/OFF
  if((action == actions_enum::ISR_L1) || (action == actions_enum::WIFI_L1)){
    gpio_set_level(LD1, lighting_states.l1);


    // L2 - ON/OFF
  } else if((action == actions_enum::ISR_L2) || (action == actions_enum::WIFI_L2)){
    gpio_set_level(LD2, !lighting_states.l2);


    // LED - ON/OFF
  } else if((action == actions_enum::ISR_LED) || (action == actions_enum::WIFI_LED)){

      //Notifica alteração do estado led
    xTaskNotify(xHandleTask_lighting_control, NOTIFY_LED_ON_OFF, eSetBits);


    // MODE, FX, AMP, PER, DES
  } else if(
            (action == actions_enum::ISR_MODO_UP) || (action == actions_enum::ISR_MODO_DOWN) || (action == actions_enum::WIFI_MODO) ||
            (action == actions_enum::WIFI_AMP_R) || (action == actions_enum::WIFI_PER_R) || (action == actions_enum::WIFI_DES_R) ||
            (action == actions_enum::WIFI_AMP_G) || (action == actions_enum::WIFI_PER_G) || (action == actions_enum::WIFI_DES_G) ||
            (action == actions_enum::WIFI_AMP_B) || (action == actions_enum::WIFI_PER_B) || (action == actions_enum::WIFI_DES_B)
          ){
      //Notifica alteração do modoled
    xTaskNotify(xHandleTask_lighting_control, NOTIFY_LED_STATE, eSetBits);
  

    // Todos
  } else if(action == actions_enum::UPDATE_ALL_STATES){
    gpio_set_level(LD1, lighting_states.l1);
    gpio_set_level(LD2, !lighting_states.l2);
    xTaskNotify(xHandleTask_lighting_control, NOTIFY_LED_ALL, eSetBits);
  }
}

  // Inicia o contador para salvar os estados na memória (NVS) [L1, L2, LED, MODO]
void update_storage_nvs(actions_enum action){

  bool ls = (action == actions_enum::ISR_L1) ||
            (action == actions_enum::ISR_L2) ||
            (action == actions_enum::ISR_LED) ||
            (action == actions_enum::ISR_MODO_UP) ||
            (action == actions_enum::ISR_MODO_DOWN) ||
            (action == actions_enum::WIFI_L1) ||
            (action == actions_enum::WIFI_L2) ||
            (action == actions_enum::WIFI_LED) ||
            (action == actions_enum::WIFI_MODO);

  bool rgb =  (action == actions_enum::WIFI_FX) ||
              (action == actions_enum::WIFI_AMP_R) ||
              (action == actions_enum::WIFI_PER_R) ||
              (action == actions_enum::WIFI_DES_R) ||
              (action == actions_enum::WIFI_AMP_G) ||
              (action == actions_enum::WIFI_PER_G) ||
              (action == actions_enum::WIFI_DES_G) ||
              (action == actions_enum::WIFI_AMP_B) ||
              (action == actions_enum::WIFI_PER_B) ||
              (action == actions_enum::WIFI_DES_B);


    // Atualiza estados de Iluminação
  if(ls){
    if(xTimerIsTimerActive(xHandleTimer_nvs_storage_lighting_states) == pdFALSE) 
      xTimerStart(xHandleTimer_nvs_storage_lighting_states, 0);
    else
      xTimerReset(xHandleTimer_nvs_storage_lighting_states, 0);

    ESP_LOGV(TAG_LD_CONTROL, "Contador iniciado para salvamento de estados LS");
  }

    // Atualiza estador do RGB
  if(rgb){
    if(xTimerIsTimerActive(xHandleTimer_nvs_storage_led_states_rgb) == pdFALSE) 
      xTimerStart(xHandleTimer_nvs_storage_led_states_rgb, 0);
    else
      xTimerReset(xHandleTimer_nvs_storage_led_states_rgb, 0);

    ESP_LOGV(TAG_LD_CONTROL, "Contador iniciado para salvamento de estados RGB");
  }
}

  // Prepara os dados e os envia para o WIFI
void update_wifi(actions_enum action){
  data_json_t json_data;

  if(!wifi_status) return;

    // Status da conexão (WIFI)
  if(action == actions_enum::WIFI_CONNECTION_STATUS_OK){
    json_data.mask = (mask_json_t::ID);
    json_data.id = id_json_t::CONNECTION_STATUS_OK;

    xQueueSendToBack(queue_wifi_send, &json_data, 0);
  

    // LD 1 - ON/OFF
  } else if((action == actions_enum::ISR_L1) || (action == actions_enum::WIFI_L1)) {
    json_data.mask = (mask_json_t::ID) + (mask_json_t::L1);
    json_data.id = id_json_t::DATA_TRANSACTION;
    json_data.ls.l1 = lighting_states.l1;

    xQueueSendToBack(queue_wifi_send, &json_data, 0);


    // LD 2 - ON/OFF
  } else if((action == actions_enum::ISR_L2) || (action == actions_enum::WIFI_L2)){
    json_data.mask = (mask_json_t::ID) + (mask_json_t::L2);
    json_data.id = id_json_t::DATA_TRANSACTION;
    json_data.ls.l2 = lighting_states.l2;

    xQueueSendToBack(queue_wifi_send, &json_data, 0);


    // LD 3 (LED) - ON/OFF
  } else if((action == actions_enum::ISR_LED) || (action == actions_enum::WIFI_LED)){
    json_data.mask = (mask_json_t::ID) + (mask_json_t::LED);
    json_data.id = id_json_t::DATA_TRANSACTION;
    json_data.ls.led = lighting_states.led;

    xQueueSendToBack(queue_wifi_send, &json_data, 0);


    // MODO
  } else if((action == actions_enum::ISR_MODO_UP) || (action == actions_enum::ISR_MODO_DOWN) || (action == actions_enum::WIFI_MODO)){
    json_data.mask =  (mask_json_t::RGB);
    json_data.id = id_json_t::DATA_TRANSACTION;
    json_data.ls.mode = lighting_states.mode;
    json_data.rgb.fx = led_states_rgb[json_data.ls.mode].fx;
    json_data.rgb.r.amp = led_states_rgb[json_data.ls.mode].r.amp;
    json_data.rgb.g.amp = led_states_rgb[json_data.ls.mode].g.amp;
    json_data.rgb.b.amp = led_states_rgb[json_data.ls.mode].b.amp;
    json_data.rgb.r.per = led_states_rgb[json_data.ls.mode].r.per;
    json_data.rgb.g.per = led_states_rgb[json_data.ls.mode].g.per;
    json_data.rgb.b.per = led_states_rgb[json_data.ls.mode].b.per;
    json_data.rgb.r.des = led_states_rgb[json_data.ls.mode].r.des;
    json_data.rgb.g.des = led_states_rgb[json_data.ls.mode].g.des;
    json_data.rgb.b.des = led_states_rgb[json_data.ls.mode].b.des;

    xQueueSendToBack(queue_wifi_send, &json_data, 0);


    // WIFI - Solicita todos os States
  } else if(action == actions_enum::WIFI_REQUEST_ALL_STATES){
    json_data.mask =  (mask_json_t::ALL);
    json_data.id = id_json_t::DATA_TRANSACTION;
    json_data.ls.l1 = lighting_states.l1;
    json_data.ls.l2 = lighting_states.l2;
    json_data.ls.led = lighting_states.led;
    json_data.ls.mode = lighting_states.mode;
    json_data.rgb.fx = led_states_rgb[json_data.ls.mode].fx;
    json_data.rgb.r.amp = led_states_rgb[json_data.ls.mode].r.amp;
    json_data.rgb.g.amp = led_states_rgb[json_data.ls.mode].g.amp;
    json_data.rgb.b.amp = led_states_rgb[json_data.ls.mode].b.amp;
    json_data.rgb.r.per = led_states_rgb[json_data.ls.mode].r.per;
    json_data.rgb.g.per = led_states_rgb[json_data.ls.mode].g.per;
    json_data.rgb.b.per = led_states_rgb[json_data.ls.mode].b.per;
    json_data.rgb.r.des = led_states_rgb[json_data.ls.mode].r.des;
    json_data.rgb.g.des = led_states_rgb[json_data.ls.mode].g.des;
    json_data.rgb.b.des = led_states_rgb[json_data.ls.mode].b.des;

    xQueueSendToBack(queue_wifi_send, &json_data, 0);
  
  
    // Nenhuma ação
  } else {
    ESP_LOGV(TAG_LD_CONTROL, "Nenhuma ação válida");
  }
}


  /* -- -- -- Função Geral -- -- -- */

  // Compara se há alterações nos parâmetros do RGB com os valores em memória
bool compare_storage_rgb(led_states_rgb_t *rgb_0, led_states_rgb_t *rgb_1){
  if(
    (rgb_0->fx != rgb_1->fx) ||

    (rgb_0->r.amp != rgb_1->r.amp) ||
    (rgb_0->g.amp != rgb_1->g.amp) ||
    (rgb_0->b.amp != rgb_1->b.amp) ||

    (rgb_0->r.per != rgb_1->r.per) ||
    (rgb_0->g.per != rgb_1->g.per) ||
    (rgb_0->b.per != rgb_1->b.per) ||

    (rgb_0->r.des != rgb_1->r.des) ||
    (rgb_0->g.des != rgb_1->g.des) ||
    (rgb_0->b.des != rgb_1->b.des)
  
  ) return true;
  else return false;
}