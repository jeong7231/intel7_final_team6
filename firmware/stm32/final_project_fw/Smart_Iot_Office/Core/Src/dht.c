/*
 * dht.c
 *
 *  Created on: Nov 20, 2024
 *      Author: IoT Main
 */

#include "DHT.h"
uint32_t DWT_Delay_Init(void)
{
  /* Disable TRC */
  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000;
  /* Enable TRC */
  CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;

  /* Disable clock cycle counter */
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
  /* Enable  clock cycle counter */
  DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk; //0x00000001;

  /* Reset the clock cycle counter value */
  DWT->CYCCNT = 0;

     /* 3 NO OPERATION instructions */
  __ASM volatile ("NOP");
  __ASM volatile ("NOP");
  __ASM volatile ("NOP");

  /* Check if clock cycle counter has started */
  if(DWT->CYCCNT)
  {
 	return 0; /*clock cycle counter started*/
  }
  else
  {
  	return 1; /*clock cycle counter not started*/
  }

}

void DWT_Delay_us(volatile uint32_t microseconds)
{
  uint32_t clk_cycle_start = DWT->CYCCNT;

  /* Go to number of cycles for system */
  microseconds *= (HAL_RCC_GetHCLKFreq() / 1000000);

  /* Delay till end */
  while ((DWT->CYCCNT - clk_cycle_start) < microseconds);
}

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	//GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/*********************************** DHT11 FUNCTIONS ********************************************/

void DHT11_Init(void)
{
	DWT_Delay_Init();
	Set_Pin_Output (DHT11_PORT, DHT11_PIN);  // set the pin as output
	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 1);   // pull the pin high
	HAL_Delay(1000);
}

void DHT11_Start (void)
{
	Set_Pin_Output (DHT11_PORT, DHT11_PIN);  // set the pin as output
	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 1);   // pull the pin high
	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 0);   // pull the pin low
	DWT_Delay_us(18000);
	//delay_us (18000);   // wait for 18ms
	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 1);   // pull the pin high
	DWT_Delay_us(20);
	//delay_us (20);   // wait for 20us
	Set_Pin_Input(DHT11_PORT, DHT11_PIN);    // set as input
}

void DHT11_Stop (void)
{
	Set_Pin_Output (DHT11_PORT, DHT11_PIN);  // set the pin as output
	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 1);   // pull the pin high
}

uint8_t DHT11_Check_Response (void)
{
	uint8_t Response = 0;

	if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))
	{
		DWT_Delay_us(80);
		//delay_us (80);
		if ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))) Response = 1;
		else Response = -1; // 255
	}
	while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));

	return Response;
}

uint8_t DHT11_Read (void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));   // wait for the pin to go high
		DWT_Delay_us(40);
		//delay_us (40);   // wait for 40 us
		if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));  // wait for the pin to go low
	}
	return i;
}

DHT11_TypeDef DHT11_readData()
{
	DHT11_TypeDef dht11;
	DHT11_Start();
	if ((dht11.status = DHT11_Check_Response()) < 0) return dht11;
	dht11.rh_byte1 = DHT11_Read ();
	dht11.rh_byte2 = DHT11_Read ();
	dht11.temp_byte1 = DHT11_Read ();
	dht11.temp_byte2 = DHT11_Read ();
	dht11.checksum = DHT11_Read();
	DHT11_Stop();
	return dht11;
}


