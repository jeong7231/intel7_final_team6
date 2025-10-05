#ifndef INC_USART6_DMA_H_
#define INC_USART6_DMA_H_

#include "main.h"
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// === 초기화 & 구동 ===
void USART6_DMA_Start(void);                 // MX_USART6_UART_Init() 이후 호출
bool USART6_Write(const uint8_t* data, size_t len); // 비동기 송신(큐잉)

// === 약간의 통계/유틸(선택) ===
size_t USART6_TxFree(void);
size_t USART6_TxUsed(void);

// === 수신 프레임 콜백 훅 ===
// IDLE이나 HT/TC에서 ‘새로 들어온 구간’을 알려줄 때 사용.
// 구현은 사용자 파일에서 하면 됨. (여기서는 약한 심볼로 선언)
void USART6_OnRxBytes(const uint8_t* data, size_t len);

#ifdef __cplusplus
}
#endif


#endif /* INC_USART6_DMA_H_ */
