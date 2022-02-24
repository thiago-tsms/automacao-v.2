#ifndef DATA_TYPES_CUSTON
#define DATA_TYPES_CUSTON

#include <stdlib.h>

#include "freertos/task.h"
#include "freertos/queue.h"


  // Inicialização de variáveis
#define DEFAULT_LIGHTING_STATES {false, false, false, 1}
#define DEFAULT_LED_STATES_RGB {{false, {1023, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {false, {0, 0, 0}, {1023, 0, 0}, {0, 0, 0}}, {false, {0, 0, 0}, {0, 0, 0}, {1023, 0, 0}}, {false, {1023, 0, 0}, {1023, 0, 0}, {1023, 0, 0}}}


  /* Parametros Notify */
#define NOTIFY_WIFI_CONECT 0x4
#define NOTIFY_WIFI_DISCONNECT 0x5


  /* Parametros Task */
TaskHandle_t xHandleTask_central_control;
TaskHandle_t xHandleTask_lighting_control;


  /* Parametros Queue */
QueueHandle_t queue_wifi_send;
QueueHandle_t queue_wifi_recv;


  /* Identificador dos botões precionados */
typedef enum {
  BT_1,
  BT_2,
  BT_3
} btn_id_enum;

  /* Dados queue Interrupção de Timer */
typedef struct {
  btn_id_enum id;
  uint16_t time;
} action_interrupt_timer_t;

  /* Identificador das ações a serem tomadas */
typedef enum {
  NOTHING,
  UPDATE_ALL_STATES,
  ISR_L1,
  ISR_L2,
  ISR_LED,
  ISR_MODO_UP,
  ISR_MODO_DOWN,
  WIFI_L1,
  WIFI_L2,
  WIFI_LED,
  WIFI_MODO,
  WIFI_FX,
  WIFI_AMP_R,
  WIFI_PER_R,
  WIFI_DES_R,
  WIFI_AMP_G,
  WIFI_PER_G,
  WIFI_DES_G,
  WIFI_AMP_B,
  WIFI_PER_B,
  WIFI_DES_B,
  WIFI_RGB,
  WIFI_CONNECTION_STATUS_OK,
  WIFI_REQUEST_ALL_STATES
} actions_enum;

  /* Dados queue Iluminação */
typedef struct {
  bool l1;
  bool l2;
  bool led;
  uint8_t mode;
} lighting_states_t;

typedef struct {
  uint16_t amp;
  uint16_t per;
  uint16_t des;
} led_params_t;

typedef struct {
  bool fx;
  led_params_t r;
  led_params_t g;
  led_params_t b;
} led_states_rgb_t;

  /* Mascara para JSON */
typedef enum {
  CLEAR = 0b0000000000000000,
  ID    = 0b0000000000000001,
  L1    = 0b0000000000000010,
  L2    = 0b0000000000000100,
  LED   = 0b0000000000001000,
  MODE  = 0b0000000000010000,
  FX    = 0b0000000000100000,
  AMP_R = 0b0000000001000000,
  PER_R = 0b0000000010000000,
  DES_R = 0b0000000100000000,
  AMP_G = 0b0000001000000000,
  PER_G = 0b0000010000000000,
  DES_G = 0b0000100000000000,
  AMP_B = 0b0001000000000000,
  PER_B = 0b0010000000000000,
  DES_B = 0b0100000000000000,
  RGB   = 0b0111111111110000,
  ALL   = 0b0111111111111111,
} mask_json_t;

/* ID ações WIFI */
typedef enum {
  NO_ACTION = 0,
  CONNECTION_STATUS_REQUEST = 1,
  CONNECTION_STATUS_OK = 2,
  REQUEST_ALL_STATES = 3,
  DATA_TRANSACTION = 4,
} id_json_t;

  /* Dados para JSON */
typedef struct {
  id_json_t id;
  uint16_t mask;
  lighting_states_t ls;
  led_states_rgb_t rgb;
} data_json_t;

#endif