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
} actions_t;

  /* Dados queue Iluminação */
typedef struct {
  bool l1;
  bool l2;
  bool led;
  //uint8_t mode;
  //bool effect;
} lighting_states_t;

  /* Dados para JSON */
typedef struct {
  uint8_t ID;
  uint8_t LD;
} data_json;
