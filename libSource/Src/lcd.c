#include "lcd.h"
#include "cmsis_os.h"
//------------------------------------------------
void Error_Handler(void);
uint8_t backLightState=0;
char lcdStringBlock = 0;
uint8_t buf[1]={0};
extern I2C_HandleTypeDef hi2c1;
char str1[100];
uint8_t portlcd; //ячейка для хранения данных порта микросхемы расширения
uint8_t iniStatus =0;

//------------------------------------------------
//__STATIC_INLINE void DelayMicro(__IO uint32_t micros)
//{
	//micros *=(SystemCoreClock / 1000000) / 5;
	//while (micros--);
//}
//------------------------------------------------

uint8_t LCD_GetIniStatus(void)
{
return iniStatus;
}


bool LCD_WriteByteI2CLCD(uint8_t bt)
{
	buf[0]=bt;


	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) ;

	if(HAL_I2C_Master_Transmit_IT(&hi2c1,(uint16_t) 0x40,buf,1)!=HAL_OK) {
		iniStatus=0;
		return false;
	}
	return true;
}
//------------------------------------------------
void sendhalfbyte(uint8_t c)
{
	c<<=4;
	e_set();//включаем линию E
	//osDelay(1);//50

	LCD_WriteByteI2CLCD(portlcd|c);
	e_reset();//выключаем линию E
	//osDelay(1);//50
	//DelayMicro(50);
}
//------------------------------------------------
void sendbyte(uint8_t c, uint8_t mode)
{
	if(mode==0) rs_reset();
	else rs_set();
	uint8_t hc=0;
	hc=c>>4;
	sendhalfbyte(hc);sendhalfbyte(c);
}
//------------------------------------------------
void LCD_Clear(void)
{

	sendbyte(0x01,0);
	osDelay(2);

}
//------------------------------------------------
void LCD_SendChar(char ch)
{

	sendbyte(ch,1);

}
//------------------------------------------------


void LCD_StringU(uint8_t* st)
{

	uint8_t i=0;
	while(st[i]<20)
	{
		sendbyte(st[i],1);
		i++;
	}

}

void LCD_StringPos(char* st, uint8_t x, uint8_t y)
{
	    LCD_SetPos(x,y);
	    uint8_t i=0;
		while(st[i]!=0 && i<20)
		{
			sendbyte(st[i],1);
			i++;
		}


}

void LCD_String(char* st)
{

	uint8_t i=0;
	while(st[i]!=0)
	{
		sendbyte(st[i],1);
		i++;
	}

}


//------------------------------------------------
void LCD_SetPos(uint8_t x, uint8_t y)
{


	switch(y)
	{
		case 0:
			sendbyte(x|0x80,0);
			osDelay(1);
			break;
		case 1:
			sendbyte((0x40+x)|0x80,0);
			osDelay(1);
			break;
		case 2:
			sendbyte((0x14+x)|0x80,0);
			osDelay(1);
			break;
		case 3:
			sendbyte((0x54+x)|0x80,0);
			osDelay(1);
			break;
	}
}

//------------------------------------------------
void LCD_ini(void)
{


	osDelay(15);
	sendhalfbyte(0x03);
	osDelay(5);
	sendhalfbyte(0x03);
	osDelay(100);
	sendhalfbyte(0x03);
	osDelay(5);
	sendhalfbyte(0x02);
	osDelay(5);
	sendbyte(0x28,0);//режим 4 бит, 2 линии (для нашего большого дисплея это 4 линии, шрифт 5х8
	osDelay(5);
	sendbyte(0x0C,0);//дисплей включаем (D=1), курсоры никакие не нужны
	osDelay(5);
	sendbyte(0x01,0);//уберем мусор
	osDelay(5);
	sendbyte(0x06,0);//пишем влево
	osDelay(5);
	setled();//подсветка
	backLightState =1;
	setwrite();//запись
	iniStatus=1;
	osDelay(10);

}

//------------------------------------------------

void LCD_offLed(void)
{

	backLightState =0;
	offLed();
	//setwrite();
}

void LCD_onLed(void)
{

	backLightState =1;
	setled();
	//setwrite();

}
uint8_t LCD_getBackLightState(void)
{

return 	backLightState;
}

void error_Handler(void)
{
while(1)
{
	HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
	HAL_Delay(100);
}

}

////////////


