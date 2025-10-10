#include "usart6_dma.h"
#include <string.h>

extern UART_HandleTypeDef huart6;

// === 설정 ===
#define RX_BUF_SIZE   512
#define TX_BUF_SIZE   512

// === 내부 버퍼 ===
static uint8_t  rx_dma_buf[RX_BUF_SIZE];
static volatile size_t rx_wr_idx = 0; // DMA NDTR 기반으로 계산

static uint8_t  tx_ring[TX_BUF_SIZE];
static volatile size_t tx_head = 0, tx_tail = 0;
static volatile bool   tx_busy = false;
static volatile size_t tx_inflight_len = 0;

// === 약한 심볼: 사용자 프로젝트에서 재정의 가능 ===
__weak void USART6_OnRxBytes(const uint8_t* data, size_t len) {
  (void)data; (void)len;
  // 여기에 프로토콜 파싱 함수 호출하도록 프로젝트 쪽에서 재정의하세요.
}

// 링버퍼 유틸
static inline size_t ring_used(void) { return (tx_head - tx_tail) & (TX_BUF_SIZE-1); }
static inline size_t ring_free(void) { return (TX_BUF_SIZE-1) - ring_used(); }
size_t USART6_TxFree(void) { return ring_free(); }
size_t USART6_TxUsed(void) { return ring_used(); }

// === 초기화 진입점: MX_USART6_UART_Init() 직후 호출 ===
void USART6_DMA_Start(void)
{
  // IDLE 라인 인터럽트 활성화
  __HAL_UART_CLEAR_IDLEFLAG(&huart6);
  __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

  // RX DMA 순환 시작
  HAL_UART_Receive_DMA(&huart6, rx_dma_buf, RX_BUF_SIZE);
}

// === IDLE 처리: “새로 들어온 구간” 계산 후 콜백 ===
static inline void RX_ProcessIdle(void)
{
  // IDLE flag 클리어
  __HAL_UART_CLEAR_IDLEFLAG(&huart6);

  // DMA 남은 카운터로 write index 계산
  size_t ndtr = __HAL_DMA_GET_COUNTER(huart6.hdmarx);
  size_t new_wr = RX_BUF_SIZE - ndtr;

  // 이번에 추가된 데이터 길이 계산(원형 버퍼)
  size_t len = (new_wr >= rx_wr_idx)
             ? (new_wr - rx_wr_idx)
             : (RX_BUF_SIZE - rx_wr_idx + new_wr);

  // 조각이 랩어라운드될 수 있으므로 두 번에 나눠 콜백
  if (len) {
    size_t first = (new_wr >= rx_wr_idx) ? len : (RX_BUF_SIZE - rx_wr_idx);
    USART6_OnRxBytes(&rx_dma_buf[rx_wr_idx], first);
    if (first < len) {
      USART6_OnRxBytes(&rx_dma_buf[0], len - first);
    }
    rx_wr_idx = new_wr;
  }
}

// === TX DMA 킥 ===
static void tx_kick(void)
{
  if (tx_busy) return;
  size_t used = ring_used();
  if (!used) return;

  size_t first_chunk = (tx_head > tx_tail)
      ? (tx_head - tx_tail)
      : (TX_BUF_SIZE - tx_tail);

  tx_busy = true;
  tx_inflight_len = first_chunk;
  HAL_UART_Transmit_DMA(&huart6, &tx_ring[tx_tail], first_chunk);
}

// === 퍼블릭: 송신 큐에 쓰기 ===
bool USART6_Write(const uint8_t* data, size_t len)
{
  if (len > ring_free()) return false; // 공간 부족
  for (size_t i=0; i<len; ++i) {
    tx_ring[tx_head] = data[i];
    tx_head = (tx_head + 1) & (TX_BUF_SIZE-1);
  }
  tx_kick();
  return true;
}

// === 콜백들 ===
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart != &huart6) return;

  size_t sent = tx_inflight_len;
  tx_tail = (tx_tail + sent) & (TX_BUF_SIZE-1);
  tx_inflight_len = 0;
  tx_busy = false;

  if (ring_used()) tx_kick();
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart != &huart6) return;

  // 오버런/프레이밍/노이즈 플래그 클리어
  __HAL_UART_CLEAR_OREFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);

  // RX 재시작
  HAL_UART_AbortReceive(huart);
  HAL_UART_Receive_DMA(huart, rx_dma_buf, RX_BUF_SIZE);
}

void USART6_DMA_IRQHandler_Entry(void)
{
  // IDLE 라인 우선 처리
  if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE) != RESET &&
      __HAL_UART_GET_IT_SOURCE(&huart6, UART_IT_IDLE) != RESET) {
    RX_ProcessIdle();           // <- usart6_dma.c 안에 이미 있는 정리 함수
  }
  // HAL 공용 처리(에러/TC 등)는 stm32f4xx_it.c 쪽에서 HAL_UART_IRQHandler가 해줌
}
