#pragma once
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  I2C_HandleTypeDef* hi2c;
  uint8_t addr7;              // 0x64~0x67
  float   cal;                // calibration factor
  int32_t offset;             // tare offset (raw)
} hx711df_t;

bool hx711df_begin(hx711df_t* dev, I2C_HandleTypeDef* hi2c, uint8_t addr7);
bool hx711df_peel(hx711df_t* dev);                     // tare
bool hx711df_enable_cal(hx711df_t* dev);
bool hx711df_get_cal_flag(hx711df_t* dev, bool* done);
bool hx711df_get_calibration(hx711df_t* dev, float* cal);
bool hx711df_set_cal_weight(hx711df_t* dev, uint16_t g);
bool hx711df_set_threshold(hx711df_t* dev, uint16_t g);

bool hx711df_read_raw(hx711df_t* dev, int32_t* raw);
bool hx711df_read_weight(hx711df_t* dev, uint8_t avg, float* g);
