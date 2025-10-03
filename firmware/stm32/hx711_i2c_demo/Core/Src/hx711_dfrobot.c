#include "hx711_dfrobot.h"

#define REG_CLEAR_REG_STATE        0x65
#define REG_DATA_GET_RAM_DATA      0x66
#define REG_DATA_GET_CALIBRATION   0x67
#define REG_DATA_GET_PEEL_FLAG     0x69
#define REG_DATA_INIT_SENSOR       0x70
#define REG_SET_CAL_THRESHOLD      0x71
#define REG_SET_TRIGGER_WEIGHT     0x72
#define REG_CLICK_RST              0x73
#define REG_CLICK_CAL              0x74

static inline HAL_StatusTypeDef tx1(hx711df_t* d, uint8_t v){
  return HAL_I2C_Master_Transmit(d->hi2c, d->addr7<<1, &v, 1, 100);
}
static inline HAL_StatusTypeDef tx(hx711df_t* d, const uint8_t* p, uint16_t n){
  return HAL_I2C_Master_Transmit(d->hi2c, d->addr7<<1, (uint8_t*)p, n, 100);
}
static inline HAL_StatusTypeDef rx(hx711df_t* d, uint8_t* p, uint16_t n){
  return HAL_I2C_Master_Receive(d->hi2c, (d->addr7<<1)|1, p, n, 100);
}

bool hx711df_begin(hx711df_t* dev, I2C_HandleTypeDef* hi2c, uint8_t addr7){
  if(!dev || !hi2c) return false;
  dev->hi2c = hi2c;
  dev->addr7 = addr7;
  dev->cal = 2236.0f;       // 라이브러리 기본값
  dev->offset = 0;

  HAL_Delay(50);
  // Wire.beginTransmission(addr); write(0x70); write(0x65); endTransmission();
  uint8_t init_seq[2] = { REG_DATA_INIT_SENSOR, REG_CLEAR_REG_STATE };
  if (tx(dev, init_seq, sizeof(init_seq)) != HAL_OK) return false;

  // 아두이노 라이브러리는 여기서 평균 10회를 읽어 offset 저장
  int32_t sum = 0, v = 0;
  for (int i=0;i<10;i++){
    if (!hx711df_read_raw(dev, &v)) return false;
    sum += v;
    HAL_Delay(2);
  }
  dev->offset = sum/10;
  return true;
}

bool hx711df_read_raw(hx711df_t* dev, int32_t* raw){
  // readReg(0x66, 4)
  if (tx1(dev, REG_DATA_GET_RAM_DATA) != HAL_OK) return false;
  HAL_Delay(22); // 라이브러리 동일 대기
  uint8_t d[4]={0};
  if (rx(dev, d, 4) != HAL_OK) return false;
  if (d[0] != 0x12) return false;       // 헤더 검증
  int32_t v = ((int32_t)d[1]<<16) | ((int32_t)d[2]<<8) | (int32_t)d[3];
  v ^= 0x800000;                        // 부호 변환
  *raw = v;
  return true;
}

bool hx711df_peel(hx711df_t* dev){
  // _offset = average(); writeReg(0x73, 0x00)
  int32_t sum=0, v;
  for (int i=0;i<10;i++){ if(!hx711df_read_raw(dev,&v)) return false; sum += v; HAL_Delay(2); }
  dev->offset = sum/10;
  uint8_t pkt[2] = { REG_CLICK_RST, 0x00 };
  return tx(dev, pkt, 2)==HAL_OK;
}

bool hx711df_enable_cal(hx711df_t* dev){
  uint8_t pkt[2] = { REG_CLICK_CAL, 0x00 };
  return tx(dev, pkt, 2)==HAL_OK;
}

bool hx711df_get_cal_flag(hx711df_t* dev, bool* done){
  if (tx1(dev, REG_DATA_GET_PEEL_FLAG) != HAL_OK) return false;
  HAL_Delay(22);
  uint8_t b=0;
  if (rx(dev, &b, 1) != HAL_OK) return false;
  *done = (b==2);
  return true;
}

bool hx711df_get_calibration(hx711df_t* dev, float* cal){
  if (tx1(dev, REG_DATA_GET_CALIBRATION) != HAL_OK) return false;
  HAL_Delay(22);
  uint8_t d[4]={0};
  if (rx(dev, d, 4) != HAL_OK) return false;
  // big-endian 4바이트를 u32로 만든 뒤 float 재해석
  uint32_t u = ((uint32_t)d[0]<<24)|((uint32_t)d[1]<<16)|((uint32_t)d[2]<<8)|d[3];
  float f;
  memcpy(&f, &u, sizeof(f));
  if (cal) *cal = f;
  dev->cal = f;
  return true;
}

bool hx711df_set_cal_weight(hx711df_t* dev, uint16_t g){
  uint8_t pkt[3] = { REG_SET_TRIGGER_WEIGHT, (uint8_t)(g>>8), (uint8_t)(g&0xFF) };
  if (tx(dev, pkt, 3) != HAL_OK) return false;
  HAL_Delay(50);
  return true;
}
bool hx711df_set_threshold(hx711df_t* dev, uint16_t g){
  uint8_t pkt[3] = { REG_SET_CAL_THRESHOLD, (uint8_t)(g>>8), (uint8_t)(g&0xFF) };
  if (tx(dev, pkt, 3) != HAL_OK) return false;
  HAL_Delay(50);
  return true;
}

bool hx711df_read_weight(hx711df_t* dev, uint8_t avg, float* g){
  if (avg==0) avg=1;
  int64_t sum=0; int32_t v=0;
  for (uint8_t i=0;i<avg;i++){
    if (!hx711df_read_raw(dev,&v)) return false;
    sum += v; HAL_Delay(2);
  }
  int32_t raw = (int32_t)(sum/avg);
  float grams = ((float)raw - (float)dev->offset) / dev->cal;
  if (g) *g = grams;
  return true;
}
