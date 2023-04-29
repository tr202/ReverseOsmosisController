#ifndef LCD_H_
#define LCD_H_
//------------------------------------------------
//#include "stm32f1xx_hal.h"
#include "stm32f4xx_hal.h"
#include <stdlib.h>
#include <stdbool.h>
//------------------------------------------------
#define e_set() LCD_WriteByteI2CLCD(portlcd|=0x04)  //установка линии Е в 1
#define e_reset() LCD_WriteByteI2CLCD(portlcd&=~0x04) //установка линии Е в 0
#define rs_set() LCD_WriteByteI2CLCD(portlcd|=0x01) //установка линии RS в 1
#define rs_reset() LCD_WriteByteI2CLCD(portlcd&=~0x01) //установка линии RS в 0
#define setled() LCD_WriteByteI2CLCD(portlcd|=0x08) //установка линии RS в 1
#define offLed() LCD_WriteByteI2CLCD(portlcd &=~0x08)
#define setwrite() LCD_WriteByteI2CLCD(portlcd&=~0x02)//установка линии RS в 0

//------------------------------------------------
void LCD_StringPos(char* st,uint8_t x,uint8_t y);
uint8_t LCD_GetIniStatus(void);
uint8_t LCD_getBackLightState(void);
void LCD_ini(void);
void LCD_Clear(void);
void LCD_SendChar(char ch);
void LCD_String(char* st);
void LCD_StringU(uint8_t* st);
void LCD_SetPos(uint8_t x, uint8_t y);
void LCD_offLed(void);
void LCD_onLed(void);
//------------------------------------------------
#endif /* LCD_H_ */
