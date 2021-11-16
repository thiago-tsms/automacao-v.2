#include <stdio.h>
#include <string.h>
#include <stdlib.h>
//#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
//#include "freertos/timers.h"

#include "driver/gpio.h"
#include "driver/hw_timer.h"
#include "driver/pwm.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "esp_log.h"
//#include "esp_system.h"

#include "data-types.h"


  /* Task Delay */
#define xDelay_Central_Control_Task pdMS_TO_TICKS(100)
#define xDelay_Lighting_Control_Task pdMS_TO_TICKS(100)


  /* TAG de Log */
const char *TAG_MAIN = "log-main";
const char *TAG_TASK_CONTROL = "log-task-control";
const char *TAG_LD_CONTROL = "log-ld-control";


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


/* Parametros Queue e Set*/
#define QUEUE_LENGHT_INTERRUPT_TIMER 6
#define QUEUE_SIZE_INTERRUPT_TIMER sizeof(action_interrupt_timer_t)
#define QUEUESET_LENGHT_RECV (QUEUE_LENGHT_INTERRUPT_TIMER + 0)
QueueHandle_t queue_interrupt_timer;
QueueSetHandle_t queueSet_control_recv;


  /* Variáveis */
lighting_states_t lighting_states = {false, false, false};


/*#define xDelay_start pdMS_TO_TICKS(100)

#define xDelay_storage_lighting_states pdMS_TO_TICKS(15000)
#define xDelay_storage_led_states pdMS_TO_TICKS(15000)

#define nvs_partition_name_lighting_states "light_states"
#define nvs_key_lighting_states "key_ligh"
#define nvs_partition_name_effect_led "led_params"
#define nvs_key_effect_led "key_led_prm"

#define default_lighting_states {false,false,false,1}
#define default_effect_led {{false,70,0,0,0,4,0,0,0,2,0,0,0}, {false,255,0,0,0,27,0,0,0,207,0,0,0}, {true,0,80,6,20,0,30,5,8,0,60,1,10}, {true,0,200,2,10,0,30,40,50,0,20,55,40}}

const char *TAG_CONTROLE = "log-controle";
const char *TAG_NVS = "log-nvs";*/



  /* Estados da iluminação */
/*
TimerHandle_t storage_lighting_states_timer;
QueueHandle_t storage_lighting_states_queue;*/

  /* Controle RGB*/
/*typedef struct {
  bool effect;
  uint8_t R;
  uint8_t amp_r;
  uint16_t per_r;
  uint16_t des_r;
  uint8_t G;
  uint8_t amp_g;
  uint16_t per_g;
  uint16_t des_g;
  uint8_t B;
  uint8_t amp_b;
  uint16_t per_b;
  uint16_t des_b;
} params_led_t;
TaskHandle_t control_effect_led_task;
QueueHandle_t effect_led_queue;
TimerHandle_t storage_params_led_timer;
QueueHandle_t storage_params_led_queue;*/

  /* WiFi */
/*typedef struct {
  lighting_states_t lighting_states;
  params_led_t params_led;
} params_send_recv_t;*/

/*QueueHandle_t send_data_queue;
QueueHandle_t recv_data_queue;
QueueHandle_t new_conection_queue;*/

//#include "tcp_server_wifi.h"

/*char *ssid_wifi = "Alex";
char *ssid_password = "9025al32";
uint32_t ip[4] = {192, 168, 1, 100};
uint32_t gateway[4] = {192, 168, 1, 1};
uint32_t netmask[4] = {255, 255, 255, 0};
int port = 6000;*/


  /* Escopo de funções */
void nvs_start();
void peripherals_config();
static void sweep_switches(void *params);

//void storage_lighting_states();
//void storage_effect_led_states();

void central_control_task(void *params);
void lighting_control_task(void *params);

actions_t select_button_action(action_interrupt_timer_t *btn_action);
void update_lighting(actions_t action);


/*void load_lighting_states(lighting_states_t *lighting_states);
void load_effect_led_states(params_led_t *params_mode_led);
bool compare_storage_effect_led_states(params_led_t *new_effect_led, params_led_t *old_effect_led);
*/

extern "C" {
  void app_main(void);
}

void app_main(){

  ESP_LOGI(TAG_MAIN, "Inicializando...");

    // Cria Queue
  ESP_LOGI(TAG_MAIN, "Criando Queue");
  queue_interrupt_timer = xQueueCreate(QUEUE_LENGHT_INTERRUPT_TIMER, QUEUE_SIZE_INTERRUPT_TIMER);
  vQueueAddToRegistry(queue_interrupt_timer, "queue-interrupt-timer");

    // Cria Set
  ESP_LOGI(TAG_MAIN, "Criando Set");
  queueSet_control_recv = xQueueCreateSet(QUEUESET_LENGHT_RECV);
  xQueueAddToSet(queue_interrupt_timer, queueSet_control_recv);

    // Inicia NVS
  ESP_LOGI(TAG_MAIN, "Iniciando NVS");
  nvs_start();

    // Configura periféricos
  ESP_LOGI(TAG_MAIN, "Configurando IO - PWM - Hardware Timer");
  peripherals_config();

  /*storage_lighting_states_queue = xQueueCreate(1, sizeof(lighting_states_t));
  storage_params_led_queue = xQueueCreate(1, sizeof(params_send_recv_t));
  effect_led_queue = xQueueCreate(4, sizeof(params_led_t));*/

      // Inicialização API Software Timer
  /*storage_lighting_states_timer =  xTimerCreate("storage-lighting-states", xDelay_storage_lighting_states, pdFALSE, NULL, storage_lighting_states);
  storage_params_led_timer = xTimerCreate("storage_params_led_timer", xDelay_storage_led_states, pdFALSE, NULL, storage_effect_led_states);*/

    // Inicia tarefas (task)
  ESP_LOGI(TAG_MAIN, "Iniciando Tasks");
  xTaskCreate(central_control_task, "central-control-task", 5000, NULL, 2, NULL);
  xTaskCreate(lighting_control_task, "lighting-control-task", 2000, NULL, 1, NULL);

    // Queue Wifi
  /*send_data_queue = xQueueCreate(6, sizeof(params_send_recv_t));
  recv_data_queue = xQueueCreate(6, sizeof(params_send_recv_t));
  new_conection_queue = xQueueCreate(1, sizeof(bool));

    // Inicialização Wifi
  tcp_server_wifi_login(ssid_wifi, ssid_password);
  tcp_server_wifi_sta(ip, gateway, netmask);
  tcp_server_wifi_port(port);
  tcp_server_start();

  vTaskDelay(xDelay_start);*/

  ESP_LOGI(TAG_MAIN, "Inicialização Finalizada");
  ESP_LOGI(TAG_MAIN, "Sistema em execução");
}


/* -- -- Inicializações e configurações -- -- */

  // Inicia o NVS
void nvs_start(){
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());
  }
  ESP_ERROR_CHECK(err);
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




/* -- -- Funções de interrupção e timer -- -- */

  // Funções de interrupção de timer para varredura de botões
static void sweep_switches(void *params){
  static bool bt1 = false, bt2 = false, bt3 = false;
  static uint16_t time_bt1 = 0, time_bt2 = 0, time_bt3 = 0;
  action_interrupt_timer_t queue_data;

    // Leitura do botão 1
  if(gpio_get_level(BT1)){
    bt1 = true;
    time_bt1 += TIME_INTERRUPT;
  
  } else if(bt1){
    queue_data.id = btn_id_t::BT_1;
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
    queue_data.id = btn_id_t::BT_2;
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
    queue_data.id = btn_id_t::BT_3;
    queue_data.time = time_bt3;
    xQueueSendToBackFromISR(queue_interrupt_timer, &queue_data, 0);

    bt3 = false;
    time_bt3 = 0;
  }
}


/* -- -- Funções de Software Timer API -- -- */

  // Salvar da memória (nvs)
/*void storage_lighting_states(){
  lighting_states_t lighting_states_update;
  esp_err_t err;

  xQueueReceive(storage_lighting_states_queue, &lighting_states_update, 0);

  uint8_t storage_data = 0, old_storage_data;
  storage_data += lighting_states_update.l1 << 7;
  storage_data += lighting_states_update.l2 << 6;
  storage_data += lighting_states_update.led << 5;
  storage_data += lighting_states_update.mode;

  nvs_handle storage_lighting_states_nvs;
  ESP_ERROR_CHECK( nvs_open(nvs_partition_name_lighting_states, NVS_READWRITE, &storage_lighting_states_nvs) );
  
  err = nvs_get_u8(storage_lighting_states_nvs, nvs_key_lighting_states, &old_storage_data);
  if( (err != ESP_OK) || (old_storage_data != storage_data)){
    ESP_ERROR_CHECK( nvs_set_u8(storage_lighting_states_nvs, nvs_key_lighting_states, storage_data) );
    ESP_ERROR_CHECK( nvs_commit(storage_lighting_states_nvs) );
  }
  
  nvs_close(storage_lighting_states_nvs);

  ESP_LOGI(TAG_CONTROLE, "Salvar lighting_states");
}*/

/*void storage_effect_led_states(){
  params_send_recv_t params_led_update;
  params_led_t old_params_led;
  esp_err_t err;
  size_t lenght;
  char key_ready[20];
  bool compare_storage = false;
  
  xQueueReceive(storage_params_led_queue, &params_led_update, 0);
  
  nvs_handle storage_effect_led_nvs;
  sprintf(key_ready, "%s_%d", nvs_key_effect_led, params_led_update.lighting_states.mode);
  ESP_ERROR_CHECK( nvs_open(nvs_partition_name_effect_led, NVS_READWRITE, &storage_effect_led_nvs) );
  
  err = nvs_get_blob(storage_effect_led_nvs, key_ready, &old_params_led, &lenght);
  if(err == ESP_OK) compare_storage = compare_storage_effect_led_states(&(params_led_update.params_led), &old_params_led);
  if(!compare_storage){
    ESP_ERROR_CHECK( nvs_set_blob(storage_effect_led_nvs, key_ready, &(params_led_update.params_led), sizeof(params_led_t)) );
    ESP_ERROR_CHECK( nvs_commit(storage_effect_led_nvs) );

    ESP_LOGI(TAG_CONTROLE, "Salvar effect-led");
  }
  
  nvs_close(storage_effect_led_nvs);
}*/


/* -- -- Tarefas (Task) -- -- */

  // Task de controle geral
void central_control_task(void *params){
  QueueSetMemberHandle_t set_recv;

  action_interrupt_timer_t btn_action;
  actions_t action;

  TickType_t reference_time_delay;
  while(true){

      // Efetua a leitura da queue
    while((set_recv = xQueueSelectFromSet(queueSet_control_recv, 0)) != NULL){

      if(set_recv == (QueueSetMemberHandle_t)queue_interrupt_timer){
        xQueueReceive(queue_interrupt_timer, &btn_action, 0);

        action = select_button_action(&btn_action);
        update_lighting(action);
        //update_wifi(action);
      }

    }

    vTaskDelayUntil(&reference_time_delay, xDelay_Central_Control_Task);
  }


  
  /*state_button_action_t action_button;

  params_send_recv_t params_send_recv;
  lighting_states_t lighting_states = default_lighting_states;
  params_led_t params_mode_led[4] = default_effect_led;

  bool update_btn = true, update_wifi = false;

  while(control_effect_led_task == NULL) vTaskDelay(pdMS_TO_TICKS(50));
  load_lighting_states(&lighting_states);
  load_effect_led_states(params_mode_led);

  TickType_t time_count_delay;
  while(true){

      // Recebe as ações dos botões
    while(uxQueueMessagesWaiting(action_button_queue) > 0){
      update_btn = true;
      data_interrupt_timer_t
      action_button_select(&action_button, &lighting_states);
    }

      // Recebe do Wifi
    while(uxQueueMessagesWaiting(recv_data_queue) > 0){
      update_wifi = true;
      xQueueReceive(recv_data_queue, &params_send_recv, 0);
      if(params_send_recv.lighting_states.mode != lighting_states.mode){ 
        update_btn = true;
        lighting_states.mode = params_send_recv.lighting_states.mode;
      } else {
        cnv_sdrv_to_lgpl(&params_send_recv, &lighting_states, &params_mode_led[params_send_recv.lighting_states.mode-1]);
      }
    }

      // Recebe nova conexão
    while(uxQueueMessagesWaiting(new_conection_queue) > 0){
      xQueueReset(new_conection_queue);

      if(uxQueueMessagesWaiting(send_data_queue) > 0) xQueueReset(send_data_queue);
      cnv_lgpl_to_sdrv(&params_send_recv, &lighting_states, &(params_mode_led[lighting_states.mode-1]));
      xQueueSend(send_data_queue, &params_send_recv, 0);

      ESP_LOGI(TAG_CONTROLE, "Envia para o Wifi");
    }

      // Atualiza os estados
    if(update_btn || update_wifi){
  
        // Atualiza l1 e l2
      update_lighting(&lighting_states);

        // Atualiza led
      if(lighting_states.led){
        vTaskResume(control_effect_led_task);
        xQueueSend(effect_led_queue, &(params_mode_led[lighting_states.mode-1]), 0);
        ESP_LOGI(TAG_CONTROLE, "LED Rodando");

      } else {
        vTaskSuspend(control_effect_led_task);
        pwm_stop(0);
        ESP_LOGI(TAG_CONTROLE, "LED Parado");
      }

        // Storage lighting states
      xQueueReset(storage_lighting_states_queue);
      xQueueSend(storage_lighting_states_queue, &lighting_states, 0);
      if(xTimerIsTimerActive(storage_lighting_states_timer) == pdFALSE) xTimerStart(storage_lighting_states_timer, 0);
      else xTimerReset(storage_lighting_states_timer, 0);

        // Atualiza o Wifi
      if(update_btn){
        if(uxQueueMessagesWaiting(send_data_queue) > 0) xQueueReset(send_data_queue);
        cnv_lgpl_to_sdrv(&params_send_recv, &lighting_states, &(params_mode_led[lighting_states.mode-1]));
        xQueueSend(send_data_queue, &params_send_recv, 0);
      }

        // Storage params led
      if(update_wifi){
        xQueueReset(storage_params_led_queue);
        xQueueSend(storage_params_led_queue, &params_send_recv, 0);
        if(xTimerIsTimerActive(storage_params_led_timer) == pdFALSE) xTimerStart(storage_params_led_timer, 0);
        else xTimerReset(storage_params_led_timer, 0);
      }

      update_btn = false;
      update_wifi = false;
    }

    vTaskDelayUntil(&time_count_delay, xDelay_central_control_task);
  }*/  
}

void lighting_control_task(void *params){
  /*params_led_t params_mode_effect_led;
  uint16_t R = 0, G = 0, B = 0;
  float time = 0;*/

  while(true){
    /*while(uxQueueMessagesWaiting(effect_led_queue) > 0){
      xQueueReceive(effect_led_queue, &params_mode_effect_led, 0);
    }

    time += xDelay_effect_led_task;
  
    if(!params_mode_effect_led.effect){
      R = params_mode_effect_led.R;
      G = params_mode_effect_led.G;
      B = params_mode_effect_led.B;

    } else {
      R = (params_mode_effect_led.amp_r/2) + (params_mode_effect_led.amp_r/2) * sin((params_mode_effect_led.per_r * time / 1000) + params_mode_effect_led.des_r / 1000);
      G = (params_mode_effect_led.amp_g/2) + (params_mode_effect_led.amp_g/2) * sin((params_mode_effect_led.per_g * time / 1000) + params_mode_effect_led.des_g / 1000);
      B = (params_mode_effect_led.amp_b/2) + (params_mode_effect_led.amp_b/2) * sin((params_mode_effect_led.per_b * time / 1000) + params_mode_effect_led.des_b / 1000);
    }

    R = (R == 0) ? 0 : ((R * 4) + 3);
    G = (G == 0) ? 0 : ((G * 4) + 3);
    B = (B == 0) ? 0 : ((B * 4) + 3);

    pwm_set_duty(0, R);
    pwm_set_duty(1, G);
    pwm_set_duty(2, B);
    pwm_start();*/

    vTaskDelay(xDelay_Lighting_Control_Task);
  }
}


/* -- -- Funções -- -- */

  // Carrega os estados salvos na memória
/*void load_lighting_states(lighting_states_t *lighting_states){
  nvs_handle storage_lighting_states_nvs;
  uint8_t load_data;
  esp_err_t err;

  nvs_open(nvs_partition_name_lighting_states, NVS_READONLY, &storage_lighting_states_nvs);
  err = nvs_get_u8(storage_lighting_states_nvs, nvs_key_lighting_states, &load_data);
  nvs_close(storage_lighting_states_nvs);

  if(err == ESP_OK){
    lighting_states->l1 = load_data & 0b10000000;
    lighting_states->l2 = load_data & 0b01000000;
    lighting_states->led = load_data & 0b00100000;
    lighting_states->mode = load_data & 0b00001111;
  
  } else {
    ESP_LOGE(TAG_NVS, "Erro load_lighting_states: %d", err);
  }
}*/

/*void load_effect_led_states(params_led_t *params_mode_led){
  char key_ready[20];
  size_t lenght;

  nvs_handle storage_effect_led_nvs;
  nvs_open(nvs_partition_name_effect_led, NVS_READONLY, &storage_effect_led_nvs);
  for(uint8_t i = 0; i < 4; i++){
    sprintf(key_ready, "%s_%d", nvs_key_effect_led, i+1);
    nvs_get_blob(storage_effect_led_nvs, key_ready, &(params_mode_led[i]), &lenght);
  }
  nvs_close(storage_effect_led_nvs);
}*/

/*bool compare_storage_effect_led_states(params_led_t *new_effect_led, params_led_t *old_effect_led){
  return (new_effect_led->effect == old_effect_led->effect) &&
  (new_effect_led->R  == old_effect_led->R) && (new_effect_led->G == old_effect_led->G) && (new_effect_led->B == old_effect_led->B) &&
  (new_effect_led->amp_r == old_effect_led->amp_r) && (new_effect_led->per_r == old_effect_led->per_r) && (new_effect_led->des_r == old_effect_led->des_r) &&
  (new_effect_led->amp_g == old_effect_led->amp_g) && (new_effect_led->per_g == old_effect_led->per_g) && (new_effect_led->des_g == old_effect_led->des_g) &&
  (new_effect_led->amp_b == old_effect_led->amp_b) && (new_effect_led->per_b == old_effect_led->per_b) && (new_effect_led->des_b == old_effect_led->des_b);
}*/


  // Interpreta a ação dos botões
actions_t select_button_action(action_interrupt_timer_t *btn_action){
  //static bool adjustment = false;

  if(btn_action->id == btn_id_t::BT_1){
    ESP_LOGI(TAG_LD_CONTROL, "Botão 1 precionado");
    //printf("Interrupção ID %d - Time: %d \n", btn_action->id, btn_action->time);

    return actions_t::UPDATE_LD_1;

  }else if(btn_action->id == btn_id_t::BT_2){
    ESP_LOGI(TAG_LD_CONTROL, "Botão 2 precionado");
    //printf("Interrupção ID %d - Time: %d \n", btn_action->id, btn_action->time);

    return actions_t::UPDATE_LD_2;

  }else if(btn_action->id == btn_id_t::BT_3){
    ESP_LOGI(TAG_LD_CONTROL, "Botão 3 precionado");
    //printf("Interrupção ID %d - Time: %d \n", btn_action->id, btn_action->time);
  }

  return actions_t::NOTHING;
  

 /* static bool select_mode_led = false, reset = false;

  bool aux = (action_button->cnt_bt1 >= 10000) && (action_button->cnt_bt2 >= 10000) && (action_button->cnt_bt3 >= 10000);
  if(aux || reset){
    reset = true;

    if(!action_button->press_bt1 && !action_button->press_bt2 && !action_button->press_bt3){
      reset = false;
      esp_restart();
    }

  } else {

      // Ação do botão 1
    if(action_button->act_bt1){
      if(select_mode_led){
        lighting_states->mode++;
        if(lighting_states->mode > 4) lighting_states->mode = 1;
        if(lighting_states->mode < 1) lighting_states->mode = 4;

        ESP_LOGI(TAG_CONTROLE, "UP %d", lighting_states->mode);

      } else {
        lighting_states->l1 = !lighting_states->l1;

        ESP_LOGI(TAG_CONTROLE, "LUZ 1 %d", lighting_states->mode);
      }
    }

      // Ação do botão 2
    if(action_button->act_bt2){
      if(select_mode_led){
        lighting_states->mode--;
        if(lighting_states->mode > 4) lighting_states->mode = 1;
        if(lighting_states->mode < 1) lighting_states->mode = 4;

        ESP_LOGI(TAG_CONTROLE, "DOWN %d \n", lighting_states->mode);
      
      } else {
        lighting_states->l2 = !lighting_states->l2;
        ESP_LOGI(TAG_CONTROLE, "BT 2");
      }
    }

      // Ação do botão 3
    if(action_button->act_bt3){
      if(select_mode_led) select_mode_led = false;
      else if((action_button->cnt_bt3 > 2000) && lighting_states->led) select_mode_led = true;
      else {
        lighting_states->led = !lighting_states->led;
        ESP_LOGI(TAG_CONTROLE, "BT 3");
      }
    }
  }*/
}

  /* Executa as ações a serem tomadas */
void update_lighting(actions_t action){
  switch(action){
    case actions_t::UPDATE_LD_1:
      lighting_states.l1 = !lighting_states.l1;
      ESP_LOGI(TAG_LD_CONTROL, "Atualizando LD1: %d", lighting_states.l1);
      gpio_set_level(LD1, lighting_states.l1);
    break;

    case actions_t::UPDATE_LD_2:
      lighting_states.l2 = !lighting_states.l2;
      ESP_LOGI(TAG_LD_CONTROL, "Atualizando LD2: %d", lighting_states.l2);
      gpio_set_level(LD2, !lighting_states.l2);
    break;

    default:
    break;
  }
}
