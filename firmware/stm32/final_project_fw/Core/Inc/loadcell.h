#ifndef INC_LOADCELL_H_
#define INC_LOADCELL_H_

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "hx711_dfrobot.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  I2C_HandleTypeDef* hi2c;
  uint8_t i2c_addr;         // hx711 dfrobot 보드 주소(예: 0x64)
  float   sign;             // 계측 방향 보정(+1 또는 -1), 보드가 음수로 증가하면 -1
  // 필터/드리프트 제어
  float   max_step_g;       // 샘플 간 허용 점프 한계
  uint8_t med_win;          // 미디언 창 크기 (1=미사용, 3 권장)
  uint8_t alpha_num;        // IIR α = num/den
  uint8_t alpha_den;
  // 오토제로
  uint8_t enable_auto_zero; // 0/1
  float   az_thresh_g;      // |g| < 이 값이면 0 근처로 간주
  uint32_t az_hold_ms;      // 위 조건이 이 시간 지속되면 자동 영점
  // 한계치(선택)
  float   max_weight_g;     // 과적 판단 임계(<=0이면 미사용)
  // 폴링 주기 정보(선택). dt를 모르더라도 필수는 아님
  uint32_t tick_ms;
} lc_cfg_t;

typedef struct {
  float raw_g;      // sign 포함, 필터 전
  float filt_g;     // sign 포함, 필터 후
  uint8_t overweight; // max_weight_g 초과 시 1
  uint32_t ts_ms;   // 최신 샘플 시간
  uint8_t valid;    // 1=유효 업데이트
} lc_reading_t;

typedef enum { LC_OK=0, LC_ERR_INIT, LC_ERR_READ } lc_status_t;

lc_status_t lc_init(const lc_cfg_t* cfg);     // HX711 시작 + 내부 상태 초기화
void        lc_deinit(void);                  // 선택사항

// 주기적으로 호출(타이머-ISR 금지, 메인루프/RTOS task에서 호출 권장)
lc_status_t lc_poll(lc_reading_t* out);

// 수동 기능
void lc_tare(void);                           // 강제 영점(초기화 + 필터 리셋)
void lc_set_sign(float sign);                 // +1 / -1
void lc_set_limits(float max_weight_g);       // 과적 임계 변경(<=0 비활성)
void lc_set_filter(uint8_t med_win, uint8_t alpha_num, uint8_t alpha_den, float max_step_g);

// 마지막 유효값 스냅샷(락-프리)
bool lc_get_last(lc_reading_t* out);

#ifdef __cplusplus
}
#endif


#endif /* INC_LOADCELL_H_ */
