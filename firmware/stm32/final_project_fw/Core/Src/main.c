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
#include "beam.h"
#include "esp.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_IN1_Port        GPIOB
#define MOTOR_IN1_Pin         GPIO_PIN_0
#define MOTOR_IN2_Port        GPIOB
#define MOTOR_IN2_Pin         GPIO_PIN_1
#define MOTOR2_IN1_Port       GPIOC
#define MOTOR2_IN1_Pin        GPIO_PIN_5
#define MOTOR2_IN2_Port       GPIOC
#define MOTOR2_IN2_Pin        GPIO_PIN_6
#define MOTOR_RUN_DURATION_MS 2000U
#define MOTOR_EXTEND_AFTER_BEAM_MS 5000U
#define ZONE_QUEUE_CAPACITY   8U
#define SERVO_PULSE_ZONE1     1350U
#define SERVO_PULSE_ZONE2      700U
#define SERVO_PULSE_ZONE3     2000U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static bool prev_blocked = false;
static bool beam2_prev_blocked = false;
static uint8_t zone_queue[ZONE_QUEUE_CAPACITY] = {0};
static uint8_t zone_queue_head = 0U;
static uint8_t zone_queue_tail = 0U;
static uint8_t zone_queue_count = 0U;
static bool  esp_ready    = false;
static bool  esp_initialized = false;
static uint32_t t_last_esp_check = 0U;
static bool  system_enabled = true;
static uint16_t servo_pulse = SERVO_PULSE_ZONE1;

typedef enum {
  MOTOR_STOPPED = 0,
  MOTOR_FORWARD,
  MOTOR_REVERSE
} motor_state_t;
static motor_state_t motor_state = MOTOR_STOPPED;
static uint32_t motor_deadline = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
static void log_uart(const char *s);
static void maintain_esp_link(uint32_t now_ms);
static void process_socket_commands(void);
static void motor_start(bool forward);
static void motor_brake(void);
static void motor_service(void);
static void motor_stop(void);
static void motor2_start_forward(void);
static void motor2_stop(void);
static void update_servo(uint16_t pulse);
static void motor_extend_deadline(uint32_t now_ms, uint32_t keep_ms);
static bool zone_queue_push(uint8_t zone);
static bool zone_queue_pop(uint8_t *zone);
static void handle_zone_selection(uint8_t zone);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Forward the EXTI interrupt to the beam helper */
  beam_exti_isr(GPIO_Pin);
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
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /* Break-beam input: PA0 with internal pull-up, falling edge */
  beam_init(GPIOA, GPIO_PIN_0, BEAM_ACTIVE_LOW, 8U, NULL);
  /* Second break-beam input: PA1 */
  beam_init_id(1U, GPIOA, GPIO_PIN_1, BEAM_ACTIVE_LOW, 8U, NULL);
  beam2_prev_blocked = beam_is_blocked_id(1U);

  log_uart("main START\r\n");
  if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  update_servo(servo_pulse);

  if (drv_uart_init() != 0) {
    log_uart("ERR: drv_uart_init\r\n");
  }

  if (drv_esp_init() == 0) {
    esp_initialized = true;
    AiotClient_Init();
    t_last_esp_check = HAL_GetTick();
    if (esp_get_status() == 0) {
      esp_ready = true;
      log_uart("ESP ready\r\n");
      esp_send_data("stm\r\n");
    } else {
      esp_ready = false;
      log_uart("ESP waiting for connection\r\n");
    }
  } else {
    log_uart("ERR: ESP init\r\n");
  }

  motor_stop();
  motor2_stop();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();
    bool blocked = beam_is_blocked();  /* true when the beam is interrupted */
    bool beam2_blocked = beam_is_blocked_id(1U);

    process_socket_commands();
    maintain_esp_link(now);
    if (!system_enabled) {
      motor_stop();
      motor2_stop();
    } else {
      motor2_start_forward();

      /* Optional logging: beam state transitions */
      if (!prev_blocked && blocked) {
        log_uart("BEAM BROKEN\r\n");
        if (esp_ready) {
          esp_send_data("jetson|capture|t\r\n");
        }
      }
      if (blocked) {
        motor_extend_deadline(now, MOTOR_RUN_DURATION_MS);
      }
      if (prev_blocked && !blocked) {
        log_uart("BEAM RESTORED\r\n");
        motor_extend_deadline(now, MOTOR_EXTEND_AFTER_BEAM_MS);
      }
    }
    prev_blocked = blocked;

    if (!beam2_prev_blocked && beam2_blocked) {
      log_uart("BEAM2 BROKEN\r\n");
      uint8_t zone = 0U;
      if (zone_queue_pop(&zone)) {
        handle_zone_selection(zone);
        motor_extend_deadline(now, MOTOR_RUN_DURATION_MS);
        motor2_start_forward();
        if (esp_ready) {
          esp_send_data("qt|ir2|t\r\n");
        } else {
          log_uart("WARN qt|ir2|t skipped (ESP not ready)\r\n");
        }
      } else {
        log_uart("WARN zone queue empty\r\n");
      }
    } else if (beam2_prev_blocked && !beam2_blocked) {
      log_uart("BEAM2 RESTORED\r\n");
    }
    beam2_prev_blocked = beam2_blocked;

    motor_service();
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void log_uart(const char *s)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)s, (uint16_t)strlen(s), 20);
}

static void motor_brake(void)
{
  HAL_GPIO_WritePin(MOTOR_IN1_Port, MOTOR_IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_IN2_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);
}

static void motor_stop(void)
{
  motor_brake();
  motor_state = MOTOR_STOPPED;
  motor_deadline = 0U;
}

static void motor2_start_forward(void)
{
  HAL_GPIO_WritePin(MOTOR2_IN1_Port, MOTOR2_IN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MOTOR2_IN2_Port, MOTOR2_IN2_Pin, GPIO_PIN_RESET);
}

static void motor2_stop(void)
{
  HAL_GPIO_WritePin(MOTOR2_IN1_Port, MOTOR2_IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR2_IN2_Port, MOTOR2_IN2_Pin, GPIO_PIN_RESET);
}

static void update_servo(uint16_t pulse)
{
  servo_pulse = pulse;
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse);
}

static void motor_extend_deadline(uint32_t now_ms, uint32_t keep_ms)
{
  if (motor_state == MOTOR_STOPPED) {
    return;
  }

  uint32_t desired_deadline = now_ms + keep_ms;
  if ((int32_t)(motor_deadline - desired_deadline) < 0) {
    motor_deadline = desired_deadline;
  }
}


static void motor_start(bool forward)
{
  motor_brake();
  if (forward) {
    HAL_GPIO_WritePin(MOTOR_IN1_Port, MOTOR_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_IN2_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);
    motor_state = MOTOR_FORWARD;
  } else {
    HAL_GPIO_WritePin(MOTOR_IN1_Port, MOTOR_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN2_Port, MOTOR_IN2_Pin, GPIO_PIN_SET);
    motor_state = MOTOR_REVERSE;
  }
  motor_deadline = HAL_GetTick() + MOTOR_RUN_DURATION_MS;
}

static void motor_service(void)
{
  if (motor_state == MOTOR_STOPPED) {
    return;
  }
  if ((int32_t)(HAL_GetTick() - motor_deadline) >= 0) {
    motor_stop();
  }
}

static void maintain_esp_link(uint32_t now_ms)
{
  if (!esp_initialized) {
    return;
  }

  if (esp_has_pending()) {
    return;
  }

  if ((now_ms - t_last_esp_check) < 5000U) {
    return;
  }
  t_last_esp_check = now_ms;

  if (esp_get_status() == 0) {
    if (!esp_ready) {
      esp_ready = true;
      log_uart("ESP connected\r\n");
      esp_send_data("stm\r\n");
    }
  } else {
    if (esp_ready) {
      esp_ready = false;
      log_uart("ESP reconnect...\r\n");
    }
    esp_client_conn();
  }
}

static void process_socket_commands(void)
{
  char msg[128];
  while (1) {
    int len = esp_fetch_message(msg, sizeof(msg));
    if (len <= 0) {
      break;
    }

    char *tok = strtok(msg, "\r\n");
    while (tok != NULL) {
      if (strstr(tok, "stm|zon|0") != NULL) {
        log_uart("CMD stm|zon|0\r\n");
        system_enabled = true;
        motor_start(false);
      } else if (strstr(tok, "stm|zon|1") != NULL) {
        log_uart("CMD stm|zon|1\r\n");
        system_enabled = true;
        motor_start(true);
        if (!zone_queue_push(1U)) {
          log_uart("WARN zone queue full\r\n");
        }
      } else if (strstr(tok, "stm|zon|2") != NULL) {
        log_uart("CMD stm|zon|2\r\n");
        system_enabled = true;
        motor_start(true);
        if (!zone_queue_push(2U)) {
          log_uart("WARN zone queue full\r\n");
        }
      } else if (strstr(tok, "stm|zon|3") != NULL) {
        log_uart("CMD stm|zon|3\r\n");
        system_enabled = true;
        motor_start(true);
        if (!zone_queue_push(3U)) {
          log_uart("WARN zone queue full\r\n");
        }
      } else if (strstr(tok, "stm|zon|4") != NULL) {
        log_uart("CMD stm|zon|4\r\n");
        system_enabled = true;
        motor_start(true);
        if (!zone_queue_push(4U)) {
          log_uart("WARN zone queue full\r\n");
        }
      } else if (strstr(tok, "stm|es|t") != NULL || strstr(tok, "stm|tur|t") != NULL) {
        const char *msg = (strstr(tok, "stm|tur|t") != NULL) ? "CMD stm|tur|t\r\n"
                                                             : "CMD stm|es|t\r\n";
        log_uart(msg);
        system_enabled = false;
        motor_stop();
        motor2_stop();
      } else if (strstr(tok, "stm|es|f") != NULL || strstr(tok, "stm|tur|f") != NULL) {
        const char *msg = (strstr(tok, "stm|tur|f") != NULL) ? "CMD stm|tur|f\r\n"
                                                             : "CMD stm|es|f\r\n";
        log_uart(msg);
        system_enabled = true;
        motor2_start_forward();
      }
      tok = strtok(NULL, "\r\n");
    }
  }
}

static bool zone_queue_push(uint8_t zone)
{
  if (zone_queue_count >= ZONE_QUEUE_CAPACITY) {
    return false;
  }
  zone_queue[zone_queue_tail] = zone;
  zone_queue_tail = (zone_queue_tail + 1U) % ZONE_QUEUE_CAPACITY;
  zone_queue_count++;
  return true;
}

static bool zone_queue_pop(uint8_t *zone)
{
  if (zone_queue_count == 0U || zone == NULL) {
    return false;
  }
  *zone = zone_queue[zone_queue_head];
  zone_queue_head = (zone_queue_head + 1U) % ZONE_QUEUE_CAPACITY;
  zone_queue_count--;
  return true;
}

static void handle_zone_selection(uint8_t zone)
{
  const char *zone_msg = NULL;
  switch (zone) {
    case 1U:
      update_servo(SERVO_PULSE_ZONE1);
      zone_msg = "Servo -> Zone1 (1350us)\r\n";
      break;
    case 2U:
      update_servo(SERVO_PULSE_ZONE2);
      zone_msg = "Servo -> Zone2 (700us)\r\n";
      break;
    case 3U:
      update_servo(SERVO_PULSE_ZONE3);
      zone_msg = "Servo -> Zone3 (2000us)\r\n";
      break;
    default:
      zone_msg = "WARN unknown zone\r\n";
      break;
  }
  if (zone_msg) {
    log_uart(zone_msg);
  }
}
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
