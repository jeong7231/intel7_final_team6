/*
 * dht.h
 *
 *  Created on: Nov 20, 2024
 *      Author: IoT Main
 */
#ifndef INC_DHT_H_
#define INC_DHT_H_

#include "stm32f4xx_hal.h"

#define DHT11_PORT GPIOC
#define DHT11_PIN GPIO_PIN_8

typedef struct {
	uint8_t rh_byte1;
	uint8_t rh_byte2;
	uint8_t temp_byte1;
	uint8_t temp_byte2;
	uint8_t checksum;
	uint8_t status;
}DHT11_TypeDef;

void DHT11_Init(void);
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void DHT11_Start (void);
void DHT11_Stop (void);
uint8_t DHT11_Check_Response (void);
uint8_t DHT11_Read (void);
DHT11_TypeDef DHT11_readData();

#endif /* INC_DHT_H_ */
