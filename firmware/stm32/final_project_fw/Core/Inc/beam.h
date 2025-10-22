#ifndef INC_BEAM_H_
#define INC_BEAM_H_

#include <stdint.h>
#include <stdbool.h>
#include "main.h"   // HAL_GetTick, GPIO typedef 사용

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  BEAM_OK = 0,
  BEAM_ERR_PARAM
} beam_status_t;

/* 빔이 끊어졌다고 판단하는 입력 레벨 (예: NPN 센서는 LOW 활성) */
typedef enum {
  BEAM_ACTIVE_LOW  = 0,   // 입력 LOW일 때 빔이 끊긴 것으로 판단
  BEAM_ACTIVE_HIGH = 1    // 입력 HIGH일 때 빔이 끊긴 것으로 판단
} beam_active_level_t;

/* 콜백: true=빔 차단, false=복구 */
typedef void (*beam_event_cb_t)(bool broken);

/* 초기화: 포트/핀, 활성 레벨, 디바운스(ms), 콜백(선택) */
beam_status_t beam_init(GPIO_TypeDef* port,
                        uint16_t pin,
                        beam_active_level_t active_level,
                        uint32_t debounce_ms,
                        beam_event_cb_t cb);
beam_status_t beam_init_id(uint8_t id,
                           GPIO_TypeDef* port,
                           uint16_t pin,
                           beam_active_level_t active_level,
                           uint32_t debounce_ms,
                           beam_event_cb_t cb);

/* HAL_GPIO_EXTI_Callback()에서 호출 */
void beam_exti_isr(uint16_t gpio_pin);

/* 현재 상태를 즉시 폴링: true=차단 */
bool beam_is_blocked(void);
bool beam_is_blocked_id(uint8_t id);

/* 래치된 차단 이벤트를 반환 후 클리어 */
bool beam_fetch_broken_event(void);
bool beam_fetch_broken_event_id(uint8_t id);

/* 디바운스 시간을 변경 */
void beam_set_debounce(uint32_t debounce_ms);
void beam_set_debounce_id(uint8_t id, uint32_t debounce_ms);

#ifdef __cplusplus
}
#endif


#endif /* INC_BEAM_H_ */
