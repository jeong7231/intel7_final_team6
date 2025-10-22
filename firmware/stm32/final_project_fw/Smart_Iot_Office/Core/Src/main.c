g/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include "esp.h"
#include "clcd.h"
#include "dht.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
#define ARR_CNT 5
#define CMD_SIZE 50

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

// ir 송신기 관련
#define NEC_HDR_MARK   9000
#define NEC_HDR_SPACE  4500
#define NEC_BIT_MARK    560
#define NEC_ONE_SPACE  1690
#define NEC_ZERO_SPACE  560

#define DEBUG 0

// 시간
typedef struct {
  int hour;
  int min;
  int sec;
} DATETIME;
DATETIME dateTime;

// 초음파 관련 변수들
uint32_t ic_val1 = 0, ic_val2 = 0, distance = 0;
uint32_t ic2_val1 = 0, ic2_val2 = 0, distance2 = 0;
uint32_t ic3_val1 = 0, ic3_val2 = 0, distance3 = 0;
uint8_t is_first_captured = 0;
uint8_t is_2_captured = 0;
uint8_t is_3_captured = 0;
volatile uint8_t HC_flag = 0;
volatile uint8_t HC_flag3 = 0;
volatile uint8_t HC_flag2 = 0;
volatile uint8_t HC_flag1 = 1;
volatile int tim3Flag10Ms = 1;

volatile unsigned int tim3Sec = 0;

volatile int tim3Flag2Sec = 1; // 서보모터 관련 변수들

volatile uint8_t Office_flag = 0; // 회사 on/off flag

// 재난 on/off flag
volatile uint8_t Fire_flag = 0;
volatile uint8_t Eq_flag = 0;
volatile int tim3_Buzzer = 1;

volatile uint8_t Inv_flag = 1; // 침입 감지
volatile uint8_t AC_flag = 0; // 선풍기 on/off

// wifi관련 변수들
uint8_t rx2char;
extern cb_data_t cb_data;
extern volatile unsigned char rx2Flag;
extern volatile char rx2Data[50];

volatile int tim3Flag1Sec=1;
volatile unsigned int tim3Sec;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
char strBuff[MAX_ESP_COMMAND_LEN];

void HCSR04_Read(GPIO_TypeDef *port, uint16_t pin);
void esp_event(char *);
void elec_off(void);
void clock_calc(DATETIME *datetime);
void delay_us(uint32_t us);
void pwm_on();
void pwm_off();
void send_mark(uint16_t time_us);
void send_space(uint16_t time_us);
void send_nec(uint32_t data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	int ret = 0;
	DHT11_TypeDef dht11Data;
	char time_buf[30], dht_buf[30];
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
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
#if DEBUG
  printf("Start main()! - MINI!\r\n");
#endif
  ret |= drv_uart_init();
  ret |= drv_esp_init();
  if(ret != 0)
  {
#if DEBUG
	  printf("Esp response error\r\n");
#endif
	  Error_Handler();
  }

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  AiotClient_Init();
  DHT11_Init();
  LCD_init(&hi2c1);


  if(HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
  {
	  Error_Handler();
  }

  if(HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK)
  {
	  Error_Handler();
  }
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 1800);

  esp_send_data("[GETTIME]\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(strstr((char *)cb_data.buf,"+IPD") && cb_data.buf[cb_data.length-1] == '\n')
		{
			//?��?��?���??  \r\n+IPD,15:[KSH_LIN]HELLO\n
			strcpy(strBuff,strchr((char *)cb_data.buf,'['));
			memset(cb_data.buf,0x0,sizeof(cb_data.buf));
			cb_data.length = 0;
			esp_event(strBuff);
		}
		if(rx2Flag)
		{
#if DEBUG
			printf("recv2 : %s\r\n",rx2Data);
#endif
			rx2Flag =0;
		}
		if(tim3Flag1Sec)	//1초에 한번
		{
			tim3Flag1Sec = 0;
			clock_calc(&dateTime);
			sprintf(time_buf, "%02d : %02d : %02d    \r\n", dateTime.hour, dateTime.min, dateTime.sec);
			LCD_writeStringXY(0, 1, time_buf);

			if(!(tim3Sec % 10)) //10초에 한번
			{
				if(esp_get_status() != 0)
				{
#if DEBUG
					printf("server connecting ...\r\n");
#endif
					esp_client_conn();
				}
			}
			if(!(tim3Sec % 60)) // 60초에 한번
			{
				dht11Data = DHT11_readData();
				if(dht11Data.rh_byte1 != 255)
				{
					sprintf(dht_buf, "[KMW_SQL]SETDB@DHT@%d@%d.%d\n", dht11Data.rh_byte1, dht11Data.temp_byte1, dht11Data.temp_byte2);
					esp_send_data(dht_buf);

					sprintf(dht_buf,"h: %d%% t: %d.%d'C", dht11Data.rh_byte1, dht11Data.temp_byte1, dht11Data.temp_byte2);				}
				else
					sprintf(dht_buf,"DHT response error");
				LCD_writeStringXY(1, 1, dht_buf);

				esp_send_data("[GETTIME]\n");
			}
		}
		if(Office_flag) // 회사안에 사람이 있는지 체크
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
			if(HC_flag) // 10ms
			{
			  HC_flag1 = 0;
			  HCSR04_Read(GPIOA, GPIO_PIN_8);
			  is_first_captured = 0;
			  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
			  while(!HC_flag1);

			  HC_flag2 = 0;
			  HCSR04_Read(GPIOB, GPIO_PIN_10);
			  is_2_captured = 0;
			  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
			  while(!HC_flag2);

			  if(distance <= 8 || distance2 <= 8) // 초음파 자동문 감지
			  {
				  tim3Flag2Sec = 1;
				  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 650);
			  }
			  HC_flag = 0;
			}
		}
		else if(!Office_flag && !(Eq_flag || Fire_flag)) // 회사에 사람이 없고 재난상황이 아닐 때
		{
			if(HC_flag) // 10ms
			{
				HC_flag3 = 0;
				HCSR04_Read(GPIOB, GPIO_PIN_12);
				is_3_captured = 0;
				HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);
				while(!HC_flag3);

				if(distance3 <= 22) // 초음파 침입 감지
				{
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
					if(Inv_flag)
						esp_send_data("[KMW_ARD]INV\n");
					Inv_flag = 0;
				}
			}
		}
		else if(Eq_flag || Fire_flag) // 재난 상황 발생 시
		{
			tim3Flag2Sec = 1;
			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 650);
		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2210;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 20000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.Pulse = 500;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 38400;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, LED_Pin|LD2_Pin|TRIG1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TRIG2_Pin|TRIG3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DHT11_Pin|BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin LD2_Pin TRIG1_Pin */
  GPIO_InitStruct.Pin = LED_Pin|LD2_Pin|TRIG1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : TRIG2_Pin TRIG3_Pin */
  GPIO_InitStruct.Pin = TRIG2_Pin|TRIG3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DHT11_Pin BUZZER_Pin */
  GPIO_InitStruct.Pin = DHT11_Pin|BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void esp_event(char * recvBuf)
{
  int i=0;
  char * pToken;
  char * pArray[ARR_CNT]={0};
  char sendBuf[MAX_UART_COMMAND_LEN]={0};

  strBuff[strlen(recvBuf)-1] = '\0';	//'\n' cut
#if DEBUG
  printf("\r\nDebug recv : %s\r\n",recvBuf);
#endif

  pToken = strtok(recvBuf,"[@]");
  while(pToken != NULL)
  {
    pArray[i] = pToken;
    if(++i >= ARR_CNT)
      break;
    pToken = strtok(NULL,"[@]");
  }

  if(!strcmp(pArray[1],"LAMP") && Office_flag && !Eq_flag && !Fire_flag)
  {
  	if(!strcmp(pArray[2],"ON"))
  	{
  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  		esp_send_data("[KMW_SQL]SETDB@LAMP@ON\n"); // db update
  	}
	else if(!strcmp(pArray[2],"OFF"))
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		esp_send_data("[KMW_SQL]SETDB@LAMP@OFF\n"); // db update
	}
  	sprintf(sendBuf,"[%s]%s@%s\n", pArray[0], pArray[1], pArray[2]);
  }

  else if(!strcmp(pArray[1], "AC") && !Eq_flag && !Fire_flag)
  {
	  if(!strcmp(pArray[2], "POW"))
	  {
		  send_nec(0xFFFF629D);
		  if(AC_flag)
		  {
			  esp_send_data("[KMW_SQL]SETDB@AC@OFF\n"); // db update
			  AC_flag = 0;
			  sprintf(sendBuf,"[%s]%s@OFF\n", pArray[0], pArray[1]);
		  }
		  else
		  {
			  esp_send_data("[KMW_SQL]SETDB@AC@ON\n"); // db update
			  AC_flag = 1;
			  sprintf(sendBuf,"[%s]%s@ON\n", pArray[0], pArray[1]);
		  }
	  }
	  else if(!strcmp(pArray[2], "DOWN"))
	  {
		  send_nec(0xFFFF906F);
	  }
	  else if(!strcmp(pArray[2], "UP"))
	  {
		  send_nec(0xFFFFE01F);
	  }
	  else if(!strcmp(pArray[2], "SLP"))
	  {
		  send_nec(0xFFFF30CF);
	  }
	  else if(!strcmp(pArray[2], "TIM"))
	  {
		  send_nec(0xFFFF7A85);
	  }
	  else if(!strcmp(pArray[2], "ROTA"))
	  {
		  send_nec(0xFFFF42BD);
	  }
	  else if(!strcmp(pArray[2], "BRIG"))
	  {
		  send_nec(0xFFFF52AD);
	  }
	  if(strcmp(pArray[2], "POW"))
	  {
		  sprintf(sendBuf, "[%s]%s@%s\n", pArray[0], pArray[1], pArray[2]);
	  }
  }

  else if(!strcmp(pArray[1], "OFFI") && !Eq_flag && !Fire_flag)
  {
	  if(!strcmp(pArray[2], "ON"))
	  {
		  Office_flag = 1;
	  }
	  else if(!strcmp(pArray[2], "OFF"))
	  {
		  elec_off();
	  }
	  sprintf(sendBuf,"[%s]%s@%s\n",pArray[0],pArray[1],pArray[2]);
  }

  else if(!strcmp(pArray[1], "DIS"))
  {
	  if(!strcmp(pArray[2], "FIRE"))
	  {
		  if(!strcmp(pArray[3], "ON"))
		  {
			  Fire_flag = 1;
			  elec_off();
		  }
		  else if(!strcmp(pArray[3], "OFF"))
		  {
			  Fire_flag = 0;
			  if(!Eq_flag)
				  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
		  }
	  }
	  else if(!strcmp(pArray[2], "EQ"))
	  {
		  if(!strcmp(pArray[3], "ON"))
		  {
			  Eq_flag = 1;
			  elec_off();
		  }
		  else if(!strcmp(pArray[3], "OFF"))
		  {
			  Eq_flag = 0;
			  if(!Fire_flag)
				  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
		  }
	  }
	  sprintf(sendBuf,"[%s]%s@%s@%s\n",pArray[0],pArray[1],pArray[2], pArray[3]);
  }

  else if(!strcmp(pArray[1], "INV"))
  {
	  if(!strcmp(pArray[2], "OFF"))
	  {
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
		  Inv_flag = 1;
	  }
  }

  else if (!strcmp(pArray[0], "GETTIME"))
  {  //GETTIME
	  dateTime.hour = (pArray[1][9] - 0x30) * 10 + pArray[1][10] - 0x30;
      dateTime.min = (pArray[1][12] - 0x30) * 10 + pArray[1][13] - 0x30;
      dateTime.sec = (pArray[1][15] - 0x30) * 10 + pArray[1][16] - 0x30;
      return;
  }

  else if(!strncmp(pArray[1]," New conn",8))
  {
#if DEBUG
	  printf("Debug : %s, %s\r\n",pArray[0],pArray[1]);
#endif
	  return;
  }
  else if(!strncmp(pArray[1]," Already log",8))
  {
#if DEBUG
 	  printf("Debug : %s, %s\r\n",pArray[0],pArray[1]);
#endif
 	  esp_client_conn();
      return;
  }

  else
      return;

  esp_send_data(sendBuf);
#if DEBUG
  printf("Debug send : %s\r\n",sendBuf);
#endif
}

void elec_off(void)
{
	  esp_send_data("[KMW_SQL]SETDB@LAMP@OFF\n"); // db update
	  Office_flag = 0;
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	  if(AC_flag)
	  {
		  send_nec(0xFFFF629D); // IR AC OFF
		  AC_flag = 0;
		  esp_send_data("[KMW_SQL]SETDB@AC@OFF\n"); // db update
	  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)		//1ms 마다 호출
{
	static int tim3Cnt = 0;
	tim3Cnt++;
	tim3Flag10Ms++;
	tim3Flag2Sec++;
	tim3_Buzzer++;

	// 초음파 감지
	if(tim3Flag10Ms >= 10)
	{
		HC_flag = 1;
		tim3Flag10Ms = 1;
	}

	if(tim3_Buzzer >= 300)
	{
		tim3_Buzzer = 1;
		if(Eq_flag || Fire_flag)
		{
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10);
		}
	}

	// 서보모터 2초
	if(tim3Flag2Sec >= 3000)
	{
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 1800);
	}

	if(tim3Cnt >= 1000) //1ms * 1000 = 1Sec
	{
		tim3Flag1Sec = 1;
		tim3Cnt = 0;
		tim3Sec++;

	}
}

void HCSR04_Read(GPIO_TypeDef *port, uint16_t pin)
{
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);  // TRIG LOW
  delay_us(2);
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);    // TRIG HIGH
  delay_us(10);  // 10us
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);  // TRIG LOW
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		if(is_first_captured == 0)
		{
			ic_val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); // 상승 에지 시간 저장
			is_first_captured = 1;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING); // 다음은 하강 에지 캡처
		}
		else
		{
			ic_val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); // 하강 에지 시간 저장
			uint32_t diff = ic_val2 - ic_val1; // 펄스 폭 계산 (시간 차)
			distance = (diff * 0.0343) / 2; // 거리 계산
			HC_flag1 = 1;
			is_first_captured = 0;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING); // 다시 상승 에지 캡처로 설정
			HAL_TIM_IC_Stop_IT(htim, TIM_CHANNEL_2); // 캡처 인터럽트 종료
		}
	}

	if(htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
	{
		if(is_2_captured == 0)
		{
			ic2_val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
			is_2_captured = 1;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else
		{
			ic2_val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
			uint32_t diff = ic2_val2 - ic2_val1;
			distance2 = (diff * 0.0343) / 2;
			HC_flag2 = 1;
			is_2_captured = 0;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
			HAL_TIM_IC_Stop_IT(htim, TIM_CHANNEL_3);
		}
	}

	if(htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
	{
		if(is_3_captured == 0)
		{
			ic3_val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
			is_3_captured = 1;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else
		{
			ic3_val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
			uint32_t diff = ic3_val2 - ic3_val1;
			distance3 = (diff * 0.0343) / 2;
			HC_flag3 = 1;
			is_3_captured = 0;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
			HAL_TIM_IC_Stop_IT(htim, TIM_CHANNEL_4);
		}
	}
}

void clock_calc(DATETIME *dateTime) {

  dateTime->sec++;  // increment second

  if (dateTime->sec >= 60)  // if second = 60, second = 0
  {
    dateTime->sec = 0;
    dateTime->min++;

    if (dateTime->min >= 60)  // if minute = 60, minute = 0
    {
      dateTime->min = 0;
      dateTime->hour++;  // increment hour
      if (dateTime->hour == 24) {
        dateTime->hour = 0;
      }
    }
  }
}

void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (HAL_RCC_GetHCLKFreq() / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
}

// PWM 출력 ON (Duty 33%)
void pwm_on()
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, htim2.Init.Period / 3);
}

// PWM 출력 OFF
void pwm_off()
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
}

// MARK: PWM ON 유지
void send_mark(uint16_t time_us)
{
    pwm_on();
    delay_us(time_us);
}

// SPACE: PWM OFF 유지
void send_space(uint16_t time_us)
{
    pwm_off();
    delay_us(time_us);
}

// NEC 32비트 데이터 전송 (LSB First)
void send_nec(uint32_t data)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    send_mark(NEC_HDR_MARK);
    send_space(NEC_HDR_SPACE);

    for (int i = 0; i < 32; i++)
    {
        send_mark(NEC_BIT_MARK);
        if (data & 0x80000000)
            send_space(NEC_ONE_SPACE);
        else
            send_space(NEC_ZERO_SPACE);
        data <<= 1;
    }

    send_mark(NEC_BIT_MARK);  // Stop bit
    send_space(0);
    pwm_off();

    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
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

#ifdef  USE_FULL_ASSERT
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

