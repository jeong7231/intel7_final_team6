#include "beam.h"

#define BEAM_MAX_INSTANCES  2U

typedef struct {
  GPIO_TypeDef* port;
  uint16_t pin;
  beam_active_level_t active;
  uint32_t debounce_ms;
  volatile uint32_t last_edge_ms;
  volatile bool event_broken;
  beam_event_cb_t cb;
  bool inited;
} beam_ctx_t;

static beam_ctx_t s_ctx[BEAM_MAX_INSTANCES] = {0};

static inline beam_ctx_t* beam_get_ctx(uint8_t id)
{
  if (id >= BEAM_MAX_INSTANCES) {
    return NULL;
  }
  return &s_ctx[id];
}

static inline bool pin_is_active_level(const beam_ctx_t* ctx)
{
  GPIO_PinState level = HAL_GPIO_ReadPin(ctx->port, ctx->pin);
  if (ctx->active == BEAM_ACTIVE_LOW)  return (level == GPIO_PIN_RESET);
  else                                 return (level == GPIO_PIN_SET);
}

beam_status_t beam_init_id(uint8_t id,
                           GPIO_TypeDef* port,
                           uint16_t pin,
                           beam_active_level_t active_level,
                           uint32_t debounce_ms,
                           beam_event_cb_t cb)
{
  beam_ctx_t* ctx = beam_get_ctx(id);
  if (!ctx || !port || (pin == 0)) {
    return BEAM_ERR_PARAM;
  }

  ctx->port = port;
  ctx->pin = pin;
  ctx->active = active_level;
  ctx->debounce_ms = (debounce_ms == 0U) ? 1U : debounce_ms;
  ctx->cb = cb;
  ctx->last_edge_ms = HAL_GetTick();
  ctx->event_broken = false;
  ctx->inited = true;

  return BEAM_OK;
}

beam_status_t beam_init(GPIO_TypeDef* port,
                        uint16_t pin,
                        beam_active_level_t active_level,
                        uint32_t debounce_ms,
                        beam_event_cb_t cb)
{
  return beam_init_id(0U, port, pin, active_level, debounce_ms, cb);
}

void beam_exti_isr(uint16_t gpio_pin)
{
  uint32_t now = HAL_GetTick();

  for (uint8_t i = 0U; i < BEAM_MAX_INSTANCES; ++i) {
    beam_ctx_t* ctx = &s_ctx[i];
    if (!ctx->inited || ctx->pin != gpio_pin) {
      continue;
    }

    if ((now - ctx->last_edge_ms) < ctx->debounce_ms) {
      continue;
    }
    ctx->last_edge_ms = now;

    bool active = pin_is_active_level(ctx);
    if (active) {
      ctx->event_broken = true;
    }

    if (ctx->cb) {
      ctx->cb(active);
    }
  }
}

bool beam_is_blocked_id(uint8_t id)
{
  beam_ctx_t* ctx = beam_get_ctx(id);
  if (!ctx || !ctx->inited) {
    return false;
  }
  return pin_is_active_level(ctx);
}

bool beam_is_blocked(void)
{
  return beam_is_blocked_id(0U);
}

bool beam_fetch_broken_event_id(uint8_t id)
{
  beam_ctx_t* ctx = beam_get_ctx(id);
  if (!ctx || !ctx->inited) {
    return false;
  }

  bool latched = ctx->event_broken;
  if (latched) {
    __disable_irq();
    ctx->event_broken = false;
    __enable_irq();
  }
  return latched;
}

bool beam_fetch_broken_event(void)
{
  return beam_fetch_broken_event_id(0U);
}

void beam_set_debounce_id(uint8_t id, uint32_t debounce_ms)
{
  beam_ctx_t* ctx = beam_get_ctx(id);
  if (!ctx || !ctx->inited) {
    return;
  }

  ctx->debounce_ms = (debounce_ms == 0U) ? 1U : debounce_ms;
}

void beam_set_debounce(uint32_t debounce_ms)
{
  beam_set_debounce_id(0U, debounce_ms);
}
