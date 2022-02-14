#ifndef DATA_TYPES_CUSTON
#define DATA_TYPES_CUSTON

#include <stdlib.h>

  /* Identificador dos botões precionados */
typedef enum {
  BT_1,
  BT_2,
  BT_3
} btn_id_t;

  /* Dados queue Interrupção de Timer */
typedef struct {
  btn_id_t id;
  uint16_t time;
} action_interrupt_timer_t;

  /* Identificador das ações a serem tomadas */
typedef enum {
  NOTHING,
  UPDATE_LD_1,
  UPDATE_LD_2,
  UPDATE_STATUS_LD_3,
  UPDATE_MODO_UP_LD_3,
  UPDATE_MODO_DOWN_LD_3
} actions_t;

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

  /* ID para JSON */
typedef enum {
  ALL = 0,
  L1 = 1,
  L2 = 2,
  L3 = 3
} json_id_t;

  /* Dados para JSON */
typedef struct {
  json_id_t id;
  bool status;
  uint8_t mode;
} data_json_t;

#endif