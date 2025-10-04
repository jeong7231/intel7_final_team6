#include "loadcell.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

static hx711df_t s_scale;
static lc_cfg_t  s_cfg;
static uint8_t   s_inited = 0;

// 필터 상태
static float    s_prev_est = 0.0f;
static uint8_t  s_prev_init = 0;
static uint32_t s_nearZ_ts = 0;

// 미디언 버퍼(최대 7개까지 확장 가능)
#define MED_MAX 7
static float s_medbuf[MED_MAX];
static uint8_t s_med_idx = 0;
static uint8_t s_med_cnt = 0;

// 최근 값 래치
static volatile lc_reading_t s_last = {0};

static float clamp_step(float prev, float cur, float max_step)
{
  if (max_step <= 0.0f || !s_prev_init) return cur;
  float d = cur - prev;
  if (d >  max_step) return prev + max_step;
  if (d < -max_step) return prev - max_step;
  return cur;
}

static float median_push_get(float x)
{
  if (s_cfg.med_win <= 1) return x;
  uint8_t win = s_cfg.med_win;
  if (win > MED_MAX) win = MED_MAX;

  s_medbuf[s_med_idx] = x;
  s_med_idx = (s_med_idx + 1) % win;
  if (s_med_cnt < win) s_med_cnt++;

  float tmp[MED_MAX];
  for (uint8_t i=0;i<s_med_cnt;i++) tmp[i] = s_medbuf[i];

  // insertion sort
  for (uint8_t i=1;i<s_med_cnt;i++) {
    float key = tmp[i];
    int j = i-1;
    while (j>=0 && tmp[j] > key) { tmp[j+1] = tmp[j]; j--; }
    tmp[j+1] = key;
  }

  if (s_med_cnt < 3) {
    float s=0.f; for (uint8_t i=0;i<s_med_cnt;i++) s+=tmp[i];
    return s/(float)s_med_cnt;
  }
  return tmp[s_med_cnt/2];
}

static float iir_lp(float in, uint8_t reset)
{
  if (s_cfg.alpha_den == 0) return in;
  static float y = 0.0f;
  static uint8_t init = 0;
  const float a = (float)s_cfg.alpha_num / (float)s_cfg.alpha_den;

  if (reset || !init) { y = in; init = 1; return y; }
  y = y + a * (in - y);
  return y;
}

static void filt_reset(void)
{
  s_prev_est  = 0.f;
  s_prev_init = 0;
  s_nearZ_ts  = 0;
  s_med_idx   = 0;
  s_med_cnt   = 0;
  (void)iir_lp(0.f, 1);
}

lc_status_t lc_init(const lc_cfg_t* cfg)
{
  if (!cfg || !cfg->hi2c) return LC_ERR_INIT;
  memset(&s_cfg, 0, sizeof(s_cfg));
  s_cfg = *cfg;
  if (s_cfg.med_win == 0) s_cfg.med_win = 1;

  if (!hx711df_begin(&s_scale, s_cfg.hi2c, s_cfg.i2c_addr)) {
    s_inited = 0; return LC_ERR_INIT;
  }

  HAL_Delay(3000);       // 전원/기구 안정
  hx711df_peel(&s_scale);// 영점

  filt_reset();
  s_inited = 1;
  return LC_OK;
}

void lc_deinit(void)
{
  s_inited = 0;
}

void lc_tare(void)
{
  if (!s_inited) return;
  hx711df_peel(&s_scale);
  filt_reset();
}

void lc_set_sign(float sign)              { s_cfg.sign = (sign>=0)? 1.f : -1.f; }
void lc_set_limits(float max_weight_g)    { s_cfg.max_weight_g = max_weight_g; }
void lc_set_filter(uint8_t med_win, uint8_t alpha_num, uint8_t alpha_den, float max_step_g)
{
  s_cfg.med_win   = (med_win==0)?1:med_win;
  s_cfg.alpha_num = alpha_num;
  s_cfg.alpha_den = alpha_den;
  s_cfg.max_step_g= max_step_g;
  // 필터 파라미터 바꾸면 내부 상태는 유지(원하면 여기서 filt_reset() 호출)
}

static void latch_last(float raw, float filt, uint8_t ow, uint32_t ts)
{
  lc_reading_t t;
  t.raw_g = raw;
  t.filt_g = filt;
  t.overweight = ow;
  t.ts_ms = ts;
  t.valid = 1;
  // 구조체 단일 쓰기: 32비트 MCU에서 이 정도는 원자적 복사로 충분
  s_last = t;
}

bool lc_get_last(lc_reading_t* out)
{
  if (!out) return false;
  *out = s_last;
  return (out->valid != 0);
}

lc_status_t lc_poll(lc_reading_t* out)
{
  if (!s_inited) return LC_ERR_INIT;

  float g_raw = 0.0f;                    // ← 반드시 먼저 선언
  if (!hx711df_read_weight(&s_scale, 10, &g_raw)) {
    return LC_ERR_READ;
  }

  uint32_t now = HAL_GetTick();

  // 0) sign → abs: 내부 파이프라인을 항상 양수로
  g_raw *= (s_cfg.sign >= 0 ? 1.f : -1.f);
  g_raw = fabsf(g_raw);

  // 1) step clamp
  float step = clamp_step(s_prev_est, g_raw, s_cfg.max_step_g);

  // 2) median
  float med = median_push_get(step);

  // 3) IIR
  float filt = iir_lp(med, !s_prev_init);
  filt = fabsf(filt);                    // 안전하게 양수 고정

  s_prev_est  = filt;
  s_prev_init = 1;

  // 4) auto zero
  if (s_cfg.enable_auto_zero) {
    if (fabsf(filt) < s_cfg.az_thresh_g) {
      if (s_nearZ_ts == 0) s_nearZ_ts = now;
      if ((now - s_nearZ_ts) > s_cfg.az_hold_ms) {
        hx711df_peel(&s_scale);
        filt_reset();                    // 다음 폴링에서 재안정화
      }
    } else {
      s_nearZ_ts = 0;
    }
  }

  // 5) overweight
  uint8_t ow = 0;
  if (s_cfg.max_weight_g > 0.f && fabsf(filt) > s_cfg.max_weight_g) ow = 1;

  latch_last(g_raw, filt, ow, now);      // 이미 abs 처리된 값 저장
  if (out) *out = s_last;

  return LC_OK;
}
