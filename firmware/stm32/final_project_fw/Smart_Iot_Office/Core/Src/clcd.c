/*
 * clcd.c
 *
 *  Created on: Nov 20, 2024
 *      Author: IoT Main
 */

#include "clcd.h"

uint8_t lcdData = 0;
I2C_HandleTypeDef *phLcdI2C;

void LCD_init(I2C_HandleTypeDef *phI2C) {
	phLcdI2C = phI2C;

	HAL_Delay(50);
	LCD_cmdMode();
	LCD_writeMode();
	LCD_sendData(0x30);
	HAL_Delay(5);
	LCD_sendData(0x30);
	HAL_Delay(1);
	LCD_sendData(0x30);
	LCD_sendData(0x20);
	LCD_sendByte(LCD_4BIT_FUNC_SET);
	LCD_sendByte(LCD_DISPLAY_OFF);
	LCD_sendByte(LCD_DISPLAY_CLEAR);
	LCD_sendByte(LCD_ENTRY_MODE_SET);
	LCD_sendByte(LCD_DISPLAY_ON);
	LCD_backLightOn();
}

void LCD_writeCmdData(uint8_t data) {
	LCD_cmdMode();			// RS pin Low
	LCD_writeMode();		// R/W pin Low

	LCD_sendByte(data);		// send byte data
}

void LCD_writeCharData(uint8_t data) {
	LCD_charMode(); 		// RS pin High
	LCD_writeMode(); 		// R/W pin Low

	LCD_sendByte(data);		// send byte data
}
void LCD_sendData(uint8_t data) {
	LCD_enableHigh();		// E pin High
	HAL_Delay(2);			// LCD need delay
	lcdData = (lcdData & 0x0f) | (data & 0xf0);
	LCD_sendDataToInterface();
	LCD_enableLow();		// E pin Low
	HAL_Delay(2);			// LCD delay
}

void LCD_sendByte(uint8_t data) {
	LCD_sendData(data);		// send upper data
	data = data << 4;		// 4bit data shift (move to upper bit)
	LCD_sendData(data);		// send lower data
}
void LCD_backLightOn() {
	lcdData |= (1 << LCD_BL);
	LCD_sendDataToInterface();
}

void LCD_backLightOff() {
	lcdData &= ~(1 << LCD_BL);
	LCD_sendDataToInterface();
}

void LCD_cmdMode() {
	lcdData &= ~(1 << LCD_RS); // 0 bit low
	LCD_sendDataToInterface();
}

void LCD_charMode() {
	lcdData |= (1 << LCD_RS); // 0 bit high
	LCD_sendDataToInterface();
}

void LCD_writeMode() {
	lcdData &= ~(1 << LCD_RW); // 1 bit low
	LCD_sendDataToInterface();
}

void LCD_enableHigh() {
	lcdData |= (1 << LCD_E); // 2 bit high
	LCD_sendDataToInterface();
}

void LCD_enableLow() {
	lcdData &= ~(1 << LCD_E); // 2 bit low
	LCD_sendDataToInterface();
}


void LCD_sendDataToInterface() {
	HAL_I2C_Master_Transmit(phLcdI2C, LCD_DEV_ADDR << 1, &lcdData, 1, 100);
}

void LCD_writeString(char *str) {
	for (int i = 0; str[i]; i++) {
		LCD_writeCharData(str[i]);
	}
}

void LCD_gotoXY(uint8_t row, uint8_t col) {
	col %= 16;
	row %= 2;

	uint8_t lcdRegAddr = ((0x40 * row) + col);
	uint8_t command = (0x80 + lcdRegAddr);
	LCD_writeCmdData(command);
}

void LCD_writeStringXY(uint8_t row, uint8_t col, char *str) {
	LCD_gotoXY(row, col);
	LCD_writeString(str);
}
