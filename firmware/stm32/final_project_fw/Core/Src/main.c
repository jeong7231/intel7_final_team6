/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "beam.h"
#include "loadcell.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 앱 파라미터 */
#define IDLE_TARE_PERIOD_MS   500U   /* 빔 정상일 때 주기적 영점 */
#define LC_POLL_PERIOD_MS     200U   /* 측정시 로드셀 폴링 주기(=5 Hz) */
#define SETTLE_MS             600U   /* 빔 끊긴 후 안정화 대기 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef enum {
  S_WAIT_ITEM = 0,   /* 빔 정상(물건 없음) → 주기적으로 tare */
  S_SETTLING,        /* 빔 끊김 직후 안정화 대기 */
  S_MEASURING        /* 측정 모드: 폴링하며 5회 평균 산출 */
} app_state_t;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* 상태머신 변수 */
static app_state_t app_state = S_WAIT_ITEM;
static uint32_t t_last_tare   = 0;
static uint32_t t_last_poll   = 0;
static uint32_t t_state_enter = 0;

/* 끊김/복구 로그용(선택) */
static bool prev_blocked = false;

/* “끊기면 단 한 번 평균” 계산용 */
static bool  printed_once = false; /* 이번 끊김 사이클에서 평균을 이미 출력했는가 */
static int   sample_cnt   = 0;     /* 수집 샘플 개수(최대 5) */
static float sample_sum   = 0.0f;  /* 샘플 합 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static void log_uart(const char *s);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern UART_HandleTypeDef huart2;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* 브레이크빔 EXTI ISR 위임 (에지 래치) */
  beam_exti_isr(GPIO_Pin);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  /* 브레이크빔: PA0, 풀업 + FALLING = LOW일 때 끊김 */
  beam_init(GPIOA, GPIO_PIN_0, BEAM_ACTIVE_LOW, /*debounce_ms=*/8, /*cb=*/NULL);

  /* 로드셀 설정 (모듈 내부 auto-zero는 끄고, 앱에서 빈 상태에만 tare) */
  lc_cfg_t cfg = {
    .hi2c = &hi2c1,
    .i2c_addr = 0x64,
    .sign = -1.0f,           /* 보드가 음수 증가라 했으니 -1 */
    .max_step_g = 500.0f,
    .med_win = 3,
    .alpha_num = 1, .alpha_den = 2,
    .enable_auto_zero = 0,   /* ★ 끄는 걸 권장: 빈 상태에서만 수동 tare */
    .az_thresh_g = 0.9f,
    .az_hold_ms = 3000,
    .max_weight_g = 0.0f,    /* 과적 판단 필요 시 값 설정 */
    .tick_ms = LC_POLL_PERIOD_MS
  };

  log_uart("main START\r\n");

  if (lc_init(&cfg) != LC_OK) {
    log_uart("ERR: lc_init\r\n");
    while(1){}
  }
  log_uart("Loadcell ready\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();
    bool blocked = beam_is_blocked();  /* true = 빔 끊김(물건 감지) */

    /* (선택) 끊김/복구 로그 */
    if (!prev_blocked && blocked)  log_uart("BEAM BROKEN\r\n");
    if ( prev_blocked && !blocked) log_uart("BEAM RESTORED\r\n");
    prev_blocked = blocked;

    switch (app_state)
    {

		case S_WAIT_ITEM:
		{
		  // 1) 먼저 끊김부터 체크 → 끊기면 즉시 측정 플로우로, 이 틱에서는 영점 금지
		  if (blocked /* 또는 beam_fetch_broken_event() 사용 가능 */) {
			app_state     = S_SETTLING;
			t_state_enter = now;

			printed_once = false;
			sample_cnt   = 0;
			sample_sum   = 0.0f;

			// 이 틱에서는 더 하지 않음
			break;
		  }

		  // 2) 여기까지 왔다는 건 '빈 상태' 유지 중 → 이제서야 주기적 영점 조건 검사
		  if ((now - t_last_tare) >= IDLE_TARE_PERIOD_MS) {
			lc_tare();                 /* HX711 tare + 필터 리셋 */
			t_last_tare = now;
			// log_uart("TARE\r\n");
		  }
		}
		break;

      case S_SETTLING:
        /* 물건 올려놓은 직후의 진동/충격이 가라앉게 대기 */
        if (!blocked) { /* 물건이 사라지면 다시 대기 */
          app_state = S_WAIT_ITEM;
          break;
        }
        if ((now - t_state_enter) >= SETTLE_MS) {
          /* 안정화 완료 → 측정 시작 */
          t_last_poll = 0;
          app_state   = S_MEASURING;
        }
        break;

      case S_MEASURING:
        if (!blocked) { /* 물건이 떠났으면 다음 사이클 준비 */
          app_state = S_WAIT_ITEM;
          break;
        }
        if ((now - t_last_poll) >= LC_POLL_PERIOD_MS) {
          lc_reading_t r = (lc_reading_t){0};
          if (lc_poll(&r) == LC_OK && r.valid) {

            if (!printed_once) {
              sample_sum += r.filt_g;
              sample_cnt++;

              /* ★ 여기 추가: 각 샘플 로그 (raw/filt, 타임스탬프 포함) */
              {
                char dbg[96];
                int  n = snprintf(dbg, sizeof(dbg),
                                  "SAMPLE %d/3  raw:%.1f g  filt:%.1f g  ts:%lu\r\n",
                                  sample_cnt, r.raw_g, r.filt_g, (unsigned long)r.ts_ms);
                HAL_UART_Transmit(&huart2, (uint8_t*)dbg, n, 30);
              }

              if (sample_cnt >= 3) {
                float avg = sample_sum / (float)sample_cnt;
                char buf[64];
                int n = snprintf(buf, sizeof(buf), "WEIGHT_AVG(5): %.1f g\r\n", avg);
                HAL_UART_Transmit(&huart2, (uint8_t*)buf, n, 30);
                printed_once = true;
              }
            }

          }
          t_last_poll = now;
        }
        break;
    } /* switch */
  }   /* while */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance             = I2C1;
  hi2c1.Init.ClockSpeed      = 100000;
  hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1     = 0;
  hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2     = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance        = USART2;
  huart2.Init.BaudRate   = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits   = UART_STOPBITS_1;
  huart2.Init.Parity     = UART_PARITY_NONE;
  huart2.Init.Mode       = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin (USER 버튼) */
  GPIO_InitStruct.Pin  = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 (브레이크빔 입력) */
  GPIO_InitStruct.Pin  = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;  /* HIGH→LOW 시 인터럽트 */
  GPIO_InitStruct.Pull = GPIO_PULLUP;           /* 평상시 HIGH */
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin (보드 LED) */
  GPIO_InitStruct.Pin   = LD2_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/* USER CODE BEGIN 4 */
static void log_uart(const char *s)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)s, (uint16_t)strlen(s), 20);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1) { }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  (void)file; (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
