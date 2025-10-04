#ifndef INC_BEAM_H_
#define INC_BEAM_H_

#include <stdint.h>
#include <stdbool.h>
#include "main.h"   // HAL_GetTick, GPIO typedef 등

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  BEAM_OK = 0,
  BEAM_ERR_PARAM
} beam_status_t;

/* 빔이 ‘끊김’ 상태일 때의 논리레벨 (보통 풀업+NPN이면 LOW가 끊김) */
typedef enum {
  BEAM_ACTIVE_LOW  = 0,   // 핀=LOW → 빔 끊김
  BEAM_ACTIVE_HIGH = 1    // 핀=HIGH → 빔 끊김
} beam_active_level_t;

/* 이벤트 콜백: true면 끊김, false면 복구 */
typedef void (*beam_event_cb_t)(bool broken);

/* 초기화: 포트/핀, 끊김 레벨, 디바운스(ms), 콜백(선택) */
beam_status_t beam_init(GPIO_TypeDef* port,
                        uint16_t pin,
                        beam_active_level_t active_level,
                        uint32_t debounce_ms,
                        beam_event_cb_t cb);

/* HAL_GPIO_EXTI_Callback()에서 그대로 위임 */
void beam_exti_isr(uint16_t gpio_pin);

/* 현재 핀 상태 폴링(즉시): true=끊김 */
bool beam_is_blocked(void);

/* 래치된 이벤트를 읽고 클리어: true면 이번 호출에서 끊김 이벤트 있었음 */
bool beam_fetch_broken_event(void);

/* 필요 시 런타임에 디바운스 시간 변경 */
void beam_set_debounce(uint32_t debounce_ms);

#ifdef __cplusplus
}
#endif


#endif /* INC_BEAM_H_ */
