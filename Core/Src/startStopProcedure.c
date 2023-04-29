/*
 * startStopProcedure.c
 *
 *  Created on: Dec 14, 2020
 *      Author: Marid
 */
#include "startStopProcedure.h"

ui8 inStart=0;
ui8 inStop=0;
ui8 state=0;
ui8 startStage =0;
ui8 emergencyStopFlag=0;
ui8 getInStart(void){return inStart;}
ui8 getInStop(void){return inStop;}
ui8 getStateStartStop(void){return state;}
ui8 getStartStage(void);
ui8 getStartStage(void)
{
return startStage;
}

bool emergencyStop(void)
{

	while(1){
	emergencyStopFlag=1;
	HAL_GPIO_WritePin(HighPressurePump_GPIO_Port, HighPressurePump_Pin, GPIO_PIN_SET); //HighPressurePumpStop
	HAL_GPIO_WritePin(AirPump_GPIO_Port, AirPump_Pin, GPIO_PIN_SET); //AirPumpStop
	HAL_GPIO_WritePin(R2SourceValve_GPIO_Port, R2SourceValve_Pin, GPIO_PIN_SET);//CloseInjetValve
	HAL_GPIO_WritePin(Relay4_GPIO_Port, Relay4_Pin, GPIO_PIN_SET); //Rotameter light off
	HAL_GPIO_WritePin(Relay5_GPIO_Port, Relay5_Pin, GPIO_PIN_SET); //Rotameter light on
	HAL_GPIO_WritePin(Relay6_GPIO_Port, Relay6_Pin, GPIO_PIN_SET); //Rotameter light on
	inStart=1;
	inStop =1;
	osDelay(100);
	LCD_Clear();
	osDelay(100);
	LCD_SetPos(3, 1);
	osDelay(100);
	LCD_String("EmergencyStop");
	osDelay(100);
	LCD_SetPos(4, 2);
	osDelay(100);
	LCD_String("Need Restart");
	osDelay(1000);
	}

	return true;

}


bool start(void){
if(inStop==0 && getStateStartStop()==0 && !emergencyStopFlag)
{
inStart=1;
state=3;
startStage=1;
osDelay(500);
LCD_onLed();
osDelay(100);
LCD_Clear();
osDelay(100);
LCD_SetPos(5, 0); LCD_String("Start");
osDelay(5000);
HAL_GPIO_WritePin(Relay5_GPIO_Port, Relay5_Pin, GPIO_PIN_RESET); //Rotameter light on
osDelay(2000);
HAL_GPIO_WritePin(Relay6_GPIO_Port, Relay6_Pin, GPIO_PIN_RESET); //Rotameter light on
osDelay(2000);
HAL_GPIO_WritePin(Relay4_GPIO_Port, Relay4_Pin, GPIO_PIN_RESET); //Rotameter light on
LCD_SetPos(0, 0); LCD_String("FeedValveOpen");
if (emergencyStopFlag) {LCD_SetPos(4,3);LCD_String("Stopped"); return false;}
HAL_GPIO_WritePin(R2SourceValve_GPIO_Port, R2SourceValve_Pin, GPIO_PIN_RESET);//open injet valve
startStage=2;
osDelay(15000);
LCD_Clear();
LCD_SetPos(0, 0); LCD_String("AirPumpStart");
if (emergencyStopFlag) {LCD_SetPos(4,3);LCD_String("Stopped"); return false;}
HAL_GPIO_WritePin(AirPump_GPIO_Port, AirPump_Pin, GPIO_PIN_RESET);//AirPumpStart
startStage=3;
osDelay(10000);
//if(getSourcePressure()<2.5){LCD_String("Low feed water pressure"); return false;}
LCD_Clear();
LCD_SetPos(0, 0); LCD_String("HighPressPumpStart");
startStage=4;
if (emergencyStopFlag) {LCD_SetPos(4,3);LCD_String("Stopped"); return false;}
if(getHighPressurePumpStartFlag()!=1){LCD_String ("Weak source Pressure"); osDelay(6000);return false;}
HAL_GPIO_WritePin(HighPressurePump_GPIO_Port, HighPressurePump_Pin, GPIO_PIN_RESET); //HighPressurePumpStart
startStage=5;
if (emergencyStopFlag) {LCD_SetPos(4,3);LCD_String("Stopped"); return false;}
osDelay(5000);
LCD_Clear();
LCD_SetPos(0, 0);
LCD_String("Works");
osDelay(1000);
inStart=0;
if (emergencyStopFlag) {LCD_SetPos(4,3);LCD_String("Stopped"); return false;}

}
startStage=0;
return true;
}


bool stop(void)
{
if(inStart==0 && getStateStartStop()==3)
{
state=2;
inStop = 1;
osDelay(500);
#define Relay5_Pin GPIO_PIN_5
#define Relay5_GPIO_Port GPIOC


LCD_onLed();
osDelay(100);
LCD_Clear();
osDelay(100);
LCD_SetPos(5, 0); LCD_String("STOP");
osDelay(5000);
LCD_Clear();
LCD_SetPos(0, 0); LCD_String("DrainValveOpen");
HAL_GPIO_WritePin(R3DrainValve_GPIO_Port, R3DrainValve_Pin, GPIO_PIN_RESET);//OpenDrainValve
osDelay(15000);
LCD_Clear();
LCD_SetPos(0, 0); LCD_String("HighPressurePumpStop");
HAL_GPIO_WritePin(HighPressurePump_GPIO_Port, HighPressurePump_Pin, GPIO_PIN_SET); //HighPressurePumpStop
if (emergencyStopFlag) {LCD_SetPos(4,3);LCD_String("Stopped"); return false;}
osDelay(1000);
LCD_Clear();
LCD_SetPos(0, 0); LCD_String("AirPumpStop");
HAL_GPIO_WritePin(AirPump_GPIO_Port, AirPump_Pin, GPIO_PIN_SET); //AirPumpStop
if (emergencyStopFlag) {LCD_SetPos(4,3);LCD_String("Stopped"); return false;}
osDelay(4000);
LCD_Clear();
LCD_SetPos(0, 0); LCD_String("InjetValveClose");
HAL_GPIO_WritePin(R2SourceValve_GPIO_Port, R2SourceValve_Pin, GPIO_PIN_SET);//CloseInjetValve
if (emergencyStopFlag) {LCD_SetPos(4,3);LCD_String("Stopped"); return false;}
osDelay(15000);
LCD_Clear();
LCD_SetPos(0, 0); LCD_String("WashValveOpen");
if (emergencyStopFlag) {LCD_SetPos(4,3);LCD_String("Stopped"); return false;}
HAL_GPIO_WritePin(R0WashValve_GPIO_Port, R0WashValve_Pin, GPIO_PIN_RESET); //WashValveOpen
osDelay(5000);
LCD_Clear();
LCD_SetPos(0, 0); LCD_String("Washing");
osDelay(30000);
LCD_Clear();
LCD_SetPos(0, 0); LCD_String("WashValveClose");
HAL_GPIO_WritePin(R0WashValve_GPIO_Port, R0WashValve_Pin, GPIO_PIN_SET); //WashValveClose
if (emergencyStopFlag) {LCD_SetPos(4,3);LCD_String("Stopped"); return false;}
osDelay(5000);
LCD_SetPos(0, 0); LCD_String("DrainValveClose");
HAL_GPIO_WritePin(R3DrainValve_GPIO_Port, R3DrainValve_Pin, GPIO_PIN_SET); //DrainValveClose
if (emergencyStopFlag) {LCD_SetPos(4,3);LCD_String("Stopped"); return false;}
osDelay(5000);
HAL_GPIO_WritePin(Relay4_GPIO_Port, Relay4_Pin, GPIO_PIN_SET); //Rotameter light off
osDelay(2000);
HAL_GPIO_WritePin(Relay5_GPIO_Port, Relay5_Pin, GPIO_PIN_SET); //Rotameter light on
osDelay(2000);
HAL_GPIO_WritePin(Relay6_GPIO_Port, Relay6_Pin, GPIO_PIN_SET); //Rotameter light on
osDelay(2000);
LCD_Clear();
inStop=0;
state=0;

}
return true;
}


