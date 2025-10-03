/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hx711_dfrobot.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
hx711df_t g_scale;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ---------------- Noise / Drift control params ---------------- */
#define SIGN            (-1.0f)   /* 보드가 음수로 증가하므로 -1 (정상이면 +1) */

#define ENABLE_AUTO_ZERO   1      /* 먼저 0으로 테스트. OK면 1로 켜세요 */
#define AZ_THRESH_G        0.9f   /* |g|<0.5g를 영점 주변으로 간주 */
#define AZ_HOLD_MS         3000U  /* 3초 지속 시 자동 영점 */

#define MAX_STEP_G         500.0f /* 샘플 간 최대 점프 허용치 */
#define MED_WIN            3      /* 미디언 창 크기 (1=미사용, 3 권장) */
#define ALPHA_NUM          1      /* IIR alpha = 1/2 */
#define ALPHA_DEN          2

/* ---------------- Internal filter state ---------------- */
static float    prev_est  = 0.0f;  /* last filtered estimate */
static int      prev_init = 0;     /* first sample? */
static uint32_t nearZ     = 0;     /* time near zero */

/* UART logger */
static void log_uart(const char *s)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)s, (uint16_t)strlen(s), 10);
}

/* clamp per-sample step size */
static float clamp_step(float prev, float cur, float max_step)
{
  float d = cur - prev;
  if (d >  max_step) return prev + max_step;
  if (d < -max_step) return prev - max_step;
  return cur;
}

/* small median filter window (MED_WIN=3) */
static float median_push_get(float x)
{
#if (MED_WIN <= 1)
  return x;
#else
  static float buf[MED_WIN] = {0};
  static int idx = 0, count = 0;

  buf[idx] = x;
  idx = (idx + 1) % MED_WIN;
  if (count < MED_WIN) count++;

  float tmp[MED_WIN];
  for (int i=0;i<count;i++) tmp[i] = buf[i];

  /* insertion sort */
  for (int i=1;i<count;i++) {
    float key = tmp[i];
    int j = i-1;
    while (j>=0 && tmp[j] > key) { tmp[j+1] = tmp[j]; j--; }
    tmp[j+1] = key;
  }

  if (count < 3) {
    float s=0.0f; for (int i=0;i<count;i++) s+=tmp[i];
    return s / (float)count;
  }
  return tmp[count/2];
#endif
}

/* simple 1st-order low-pass IIR, y += a*(x - y) */
static float iir_lp(float in, int reset)
{
#if (ALPHA_DEN <= 0)
  return in;
#else
  static float y = 0.0f;
  static int init = 0;
  const float alpha = (float)ALPHA_NUM / (float)ALPHA_DEN;

  if (reset || !init) { y = in; init = 1; return y; }
  y = y + alpha * (in - y);
  return y;
#endif
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  if (!hx711df_begin(&g_scale, &hi2c1, 0x64)) {
    log_uart("ERR: hx711df_begin failed\r\n");
    while (1) { }
  }

  HAL_Delay(3000);            /* power/mechanics settle */
  hx711df_peel(&g_scale);     /* tare on empty */
  log_uart("tare done, ready\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    float g_raw = 0.0f;

    if (hx711df_read_weight(&g_scale, 10, &g_raw)) {
      /* 0) sign correction */
      g_raw *= SIGN;

      /* 1) step clamp */
      float g_step = prev_init ? clamp_step(prev_est, g_raw, MAX_STEP_G) : g_raw;

      /* 2) median */
      float g_med = median_push_get(g_step);

      /* 3) IIR low-pass (reset on first sample) */
      float g_filt = iir_lp(g_med, !prev_init);
      prev_est  = g_filt;
      prev_init = 1;

#if ENABLE_AUTO_ZERO
      /* 4) auto-zero near 0 for a while */
      if (fabsf(g_filt) < AZ_THRESH_G) {
        if (nearZ == 0) nearZ = HAL_GetTick();
        if (HAL_GetTick() - nearZ > AZ_HOLD_MS) {
          hx711df_peel(&g_scale);
          iir_lp(0.0f, 1);    /* reset filter */
          prev_est  = 0.0f;
          prev_init = 0;
          nearZ     = 0;
          log_uart("Auto zeroed\r\n");
        }
      } else {
        nearZ = 0;
      }
#endif

      /* 5) integer-friendly logging: show raw & filt in 0.1 g units */
      float g_raw_abs  = fabsf(g_raw);
      float g_filt_abs = fabsf(g_filt);

      int d_raw  = (int)llroundf(g_raw_abs  * 10.0f);
      int d_filt = (int)llroundf(g_filt_abs * 10.0f);

      char buf[96];
      int n = snprintf(buf, sizeof(buf),
                       "raw:%d.%01d g, filt:%d.%01d g\r\n",
                       d_raw/10,  abs(d_raw%10),
                       d_filt/10, abs(d_filt%10));
      if (n > 0) HAL_UART_Transmit(&huart2, (uint8_t*)buf, n, 50);
    }

    HAL_Delay(200); /* ~5 Hz */
  }
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
