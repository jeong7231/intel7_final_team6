#include "beam.h"

static GPIO_TypeDef* s_port = NULL;
static uint16_t s_pin = 0;
static beam_active_level_t s_active = BEAM_ACTIVE_LOW;
static uint32_t s_db_ms = 2;
static volatile uint32_t s_last_edge_ms = 0;
static volatile bool s_event_broken = false;   // 끊김 이벤트 래치
static beam_event_cb_t s_cb = NULL;

static inline bool pin_is_active_level(void) {
  GPIO_PinState level = HAL_GPIO_ReadPin(s_port, s_pin);
  if (s_active == BEAM_ACTIVE_LOW)  return (level == GPIO_PIN_RESET);
  else                              return (level == GPIO_PIN_SET);
}

beam_status_t beam_init(GPIO_TypeDef* port,
                        uint16_t pin,
                        beam_active_level_t active_level,
                        uint32_t debounce_ms,
                        beam_event_cb_t cb)
{
  if (!port || (pin == 0)) return BEAM_ERR_PARAM;
  s_port = port;
  s_pin = pin;
  s_active = active_level;
  s_db_ms = (debounce_ms == 0) ? 1 : debounce_ms;
  s_cb = cb;

  /* 초기 상태 클리어 */
  s_last_edge_ms = HAL_GetTick();
  s_event_broken = false;

  return BEAM_OK;
}

/* EXTI 공용 콜백에서 호출 */
void beam_exti_isr(uint16_t gpio_pin)
{
  if (gpio_pin != s_pin) return;

  uint32_t now = HAL_GetTick();
  if ((now - s_last_edge_ms) < s_db_ms) return;  // 소프트 디바운스
  s_last_edge_ms = now;

  /* 핀의 현재 레벨을 읽어 실제 ‘끊김/복구’ 확정 */
  bool active = pin_is_active_level(); // true면 ‘빔 끊김’
  if (active) {
    s_event_broken = true;    // 래치
  }

  if (s_cb) {
    s_cb(active);             // 즉시 알림(원하면)
  }
}

bool beam_is_blocked(void)
{
  if (!s_port) return false;
  return pin_is_active_level();
}

bool beam_fetch_broken_event(void)
{
  bool latched = s_event_broken;
  if (latched) {
    __disable_irq();
    s_event_broken = false;
    __enable_irq();
  }
  return latched;
}

void beam_set_debounce(uint32_t debounce_ms)
{
  s_db_ms = (debounce_ms == 0) ? 1 : debounce_ms;
}
