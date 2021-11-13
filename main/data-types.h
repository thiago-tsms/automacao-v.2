  /* Dados queue Interrupção de Timer */
enum btn_id { btn1, btn2, btn3 };
typedef struct {
  enum btn_id id;
  uint16_t time;
} data_interrupt_timer_t;

/* Dados queue Iluminação */
typedef struct {
  bool l1;
  bool l2;
  bool led;
  uint8_t mode;
} lighting_states_t;