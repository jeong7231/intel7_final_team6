/*
 * clcd.h
 *
 *  Created on: Nov 20, 2024
 *      Author: IoT Main
 */

#ifndef INC_CLCD_H_
#define INC_CLCD_H_

#include "stm32f4xx_hal.h"

#define LCD_RS			0
#define LCD_RW			1
#define LCD_E			2
#define LCD_BL			3 // back light'

#define LCD_4BIT_FUNC_SET	0x28
#define LCD_DISPLAY_OFF		0x08
#define LCD_DISPLAY_ON		0x0c
#define LCD_DISPLAY_CLEAR	0x01
#define LCD_ENTRY_MODE_SET	0x06

#define LCD_DEV_ADDR	0x27 // basic setting

void LCD_init(I2C_HandleTypeDef *phI2C);
void LCD_writeCmdData(uint8_t data);
void LCD_writeCharData(uint8_t data);
void LCD_sendData(uint8_t data);
void LCD_sendByte(uint8_t data);
void LCD_backLightOn();
void LCD_backLightOff();
void LCD_cmdMode();
void LCD_charMode();
void LCD_writeMode();
void LCD_enableHigh();
void LCD_enableLow();
void LCD_sendDataToInterface();

void LCD_writeString(char *str);
void LCD_gotoXY(uint8_t row, uint8_t col);
void LCD_writeStringXY(uint8_t row, uint8_t col, char *str);

#endif /* INC_CLCD_H_ */
