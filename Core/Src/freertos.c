/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"
#include "dma.h"
#include "i2c.h"

#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "startStopProcedure.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//////////////LCD Light On Off Flag
ui8 LCDOnOffFlag=1;
ui8 i2cErrorFlag=0;
//////////////Inputs button Float switches and motion sensor GPIOs
typedef struct
{
GPIO_TypeDef* Port;
uint16_t Pin;
uint16_t Count;
uint16_t Cycles;
GPIO_PinState activePinState;
uint8_t inputName;
uint8_t pinState;
} portStateTypedef;

//FrequencyCalculation//
uint8_t globalTim2Flag =0;
typedef struct
{
TIM_HandleTypeDef* tim;
uint32_t channel;
uint32_t CCRx[2];
float frequence;
uint8_t namePos;
uint8_t compFlag;
uint16_t liveTime;
uint8_t sensorType;
float flow;
}frequencyCalcTypedef;
//RatesCalculation//
typedef struct
{
float regectionRate;
float recyclingRate;
float membraneFlow;
}ratesCalculationTypedef;


typedef struct
{
_RTC timeDate;
float concentrateTDS;
float permeateTDS;
float reserved;
float waterTemp;
float soufcePressure;
float membranePressure;
float sourceFlow;
float drainFlow;
float recycleFlow;
float permeateFlow;
float membraneFlow;
float recyclingRate;
float permeateRate;
uint8_t userButtonS;
uint8_t startButtonS;
uint8_t stopButtonS;

uint8_t highLevelBulbS;
uint8_t lowLevelBulbS;
uint8_t motionSensorS;



/*
          sprintf(uartStringTimeDate,(char*)format,rtc.Hour, rtc.Min, rtc.Sec, rtc.DaysOfWeek, rtc.Month, rtc.Year);
          sprintf(lcdStringTds," Concentrate TDS %.2fppm   Permeate TDS %.2fppm " " Reserved %u ",getTds(adcAverageValues[0]),getTds(adcAverageValues[1]),adcAverageValues[2]);
		  float hpressure = getPressure(adcAverageValues[5],25,1.13);
		  sprintf(lcdStringTemp,"Water temp %.2f*C  Source pressure %.2f bar  Membrane pressure %.2f bar ",getTemp(adcAverageValues[3]),getPressure(adcAverageValues[4],5,1),hpressure);
		  sprintf(lcdStringFlow1,"Source %.1f l/m  Drain %.1f l/m Recyle %.1f l/m Permeate %.1f l/m",frequences[0].flow,frequences[1].flow,frequences[2].flow,frequences[3].flow);

		  sprintf(lcdStringRates,"Membrane Flow %.1f" "l/m " "Recycling rate %.1f%% ""Permeate rate %.1f%% ",rates.membraneFlow,rates.recyclingRate,rates.regectionRate);
          sprintf(usartStringPortStates,"User button %u Start button %u Stop button %u High level bulb %u Low level bulb %u Motion sensor %u ",portStates[0].pinState,portStates[1].pinState,portStates[2].pinState, portStates[3].pinState,portStates[4].pinState,portStates[5].pinState);
*/
}sensorValuesTypedef;

sensorValuesTypedef sensorValues = {0,};




_RTC rtc = {
    .Year = 20, .Month = 12, .Date = 22,
    .DaysOfWeek = FRIDAY,
    .Hour = 4, .Min = 31, .Sec = 10
};

uint8_t regVal;
float rtcTemp;




/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

const char *formatsLong[] = {
		"%02d:%02d:%02d", "%02d.%02d.%2d","Concentrate TDS %.2fppm","Permeate TDS %.2fppm","Reserved %.2f",
		"Water temp %.2f*C","Source pressure %.2f bar","Membrane pressure %.2f bar",
		"Source %.2f l/m","Drain %.2f l/m","Recyle %.2f l/m","Permeate %.2f l/m",
		"Membrane Flow %.1f l/m ","Recycling rate %.1f%%","Permeate rate %.1f%%",
		"User button %u","Start button %u","Stop button %u","High level bulb %u","Low level bulb %u","Motion sensor %u"

};

const char *formatsShort[] = {
		"%02d:%02d:%02d  %02d.%02d.20%2d", //0
		"Atendado        TDS",//1
		"Concentrate %.1fppm",//2
		"Permeate %.1fppm   ",//3
		"Reserved %.2f      ",//4
		"Membr pres %.2f MPa",//5
		"Water temp %.2f C  ",//6
        "Feed press %.2f MPa",//7

		"Feed %.2f l/m       ",//8
		"Drain %.2f l/m      ",//9
		"Recycle %.2f l/m    ",//10
		"Permeate %.2f l/m   ",//11
		"Membrane %.2f l/m   ",//12
		"Recycling %.2f%%    ",//13
		"Permeate %.2f%%     ",//14
		"     Input states   ",//15
		"Blue button %u      ",//16
		"Start %u Stop %u    ",//17
		"High %u Low %u float",//18
		"Motion sensor %u    ",//19
		"En progreso      TDS", //20
		"Atendado  AUTO   TDS",//21
		"Atendado  MANUAL TDS",//22
		"En progreso AUTO TDS",//23
		"En progreso  MAN TDS"//24


};

//char stringArrau[80] = {0,};


char window[20][30] = {
       // 012345678901234567890123456789
		{"00:00:00   0.00.2000"},
		{"  row1              "},
		{"  row2              "},
		{"  row3              "},
	    {"  row4              "},
		{"  row5              "},
	    {"  row6              "},

};


//typedef struct
//{
	//uint8_t formatNum;
//	char param[20];

//}sensorVariaTypedef;

//sensorVariaTypedef sensorV[20];


//typedef struct
//{

//	ui8 x;
//	ui8 y;
//	ui8 length;
//char value[];
//}strSensorValuesTypedef;

//strSensorValuesTypedef stringSensorValues[20];

float sensorValuesArr[13]={0,};
uint8_t sensorValuesArrLenght = 13;


uint8_t i2cFlag=1;
uint8_t LCD_IniTryingInterval =2;
uint8_t LCD_IniTryingIntervalInitialValue = 20;


uint16_t cycles =500; //number of read input pin cycles to generate inputActiveLevel
portStateTypedef portStates[6] = {0};
uint16_t portStatesLenght = 6;
char lcdStringTime[20]={0,};
char lcdString[20]={0};
char lcdStringT[20]={0,};
char lcdStringTm[20]={0,};
char lcdStringF[20]={0,};
char lcdWindow[80][20]={0,};
char *str[20]={0,};
ui8 lcdWindowRenewalFlag=0;
uint8_t lcdWindowString[20]={0,};



uint8_t usartString[1000]={0,};
uint32_t usartStringLen = 1000;
char usartStringTmp[100] = {0,};
uint16_t timeRenovationPeriod = 100; // osDelau Units*10 (500)

uint8_t uartFlag=0;
char* timePtr=0;
char uartStringTimeDate[100]={0,};
char usartStringPortStates[100]={0,};
char lcdStringTds[100]={0,};
char lcdStringTemp[100]={0,};
char lcdStringFlow1[100]={0,};
char lcdStringRates[100]={0,};

////////////////////////////////////////////////////////////////////////////////////
frequencyCalcTypedef frequences[4]={0};
uint8_t frequencesLength = 4;
uint32_t sysClock = 180000000;
uint32_t dividend=1000000;
uint16_t timeUnactivityToClearFrequency = 10;//osDelay Units*10 1/100 sec
ratesCalculationTypedef rates={0};

//ADC
uint16_t adcChannelsValues[30]={0,};
uint32_t lengthOfAdcChannelsValues = 30;
uint8_t adcReadyFlag=0;
uint16_t adcAverageValues[6] = {0,};
uint8_t adcAverageValuesLength = 6;
uint16_t shiftedData[6]={0,};
uint32_t adc = 0;
uint32_t adcPr = 0;
uint32_t adcPr1 = 0;

//dif
uint16_t timeToBackLightOff = 60000;//osDelay Units*10 1/100 sec
uint16_t varTimeToBackLightOff=60000;

uint8_t highPressurePumpStartFlag=0;
uint8_t autoManStartFlag=0;

//usart Buffer
///uint8_t usartBuffer[50] ={0,};


/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for inputs */
osThreadId_t inputsHandle;
const osThreadAttr_t inputs_attributes = {
  .name = "inputs",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for adc */
osThreadId_t adcHandle;
const osThreadAttr_t adc_attributes = {
  .name = "adc",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for getFlowsTask */
osThreadId_t getFlowsTaskHandle;
const osThreadAttr_t getFlowsTask_attributes = {
  .name = "getFlowsTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for checkValues */
osThreadId_t checkValuesHandle;
const osThreadAttr_t checkValues_attributes = {
  .name = "checkValues",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 500 * 4
};
/* Definitions for usart */
osThreadId_t usartHandle;
const osThreadAttr_t usart_attributes = {
  .name = "usart",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 500 * 4
};
/* Definitions for lcd */
osThreadId_t lcdHandle;
const osThreadAttr_t lcd_attributes = {
  .name = "lcd",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 500 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void checkPort(portStateTypedef* portState, uint16_t cycles);
uint16_t getArray_getMedian(uint16_t* dataForShift,u16 dataShift,u16 shiftIndex, u16 dataLen);
uint16_t getMedianNum(uint16_t* bArray, uint16_t iFilterLen);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartInputs(void *argument);
void StartAdc(void *argument);
void StartGetFlows(void *argument);
void StartCheckValues(void *argument);
void StartUsart(void *argument);
void StartLcd(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of inputs */
  inputsHandle = osThreadNew(StartInputs, NULL, &inputs_attributes);

  /* creation of adc */
  adcHandle = osThreadNew(StartAdc, NULL, &adc_attributes);

  /* creation of getFlowsTask */
  getFlowsTaskHandle = osThreadNew(StartGetFlows, NULL, &getFlowsTask_attributes);

  /* creation of checkValues */
  checkValuesHandle = osThreadNew(StartCheckValues, NULL, &checkValues_attributes);

  /* creation of usart */
  usartHandle = osThreadNew(StartUsart, NULL, &usart_attributes);

  /* creation of lcd */
  lcdHandle = osThreadNew(StartLcd, NULL, &lcd_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
 void toggleFlag(uint8_t flag)
 {
	 if(flag == 0) {flag =1;} else if (flag==1){flag =0;}
 }

/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  /* Infinite loop */
  for(;;)
  {

	  if(portStates[0].pinState ==1 && portStates[2].pinState==1)
	  {

		  if(autoManStartFlag == 0) {autoManStartFlag =1;osDelay(3000);} else {autoManStartFlag=0;osDelay(3000);}

	  } // Set manual or set auto start when when the set time comes

	  if(portStates[3].pinState ==0 && portStates[4].pinState==1 && !getStateStartStop() && autoManStartFlag ==1)
	  {
		  if(rtc.Hour>=23)
		  {
			  start();
		  }
		  if(rtc.Hour<6)
		  {
			  start();
		  }
	  }

	  if(portStates[3].pinState !=0 && getStateStartStop()==3){stop();} // HighLevelfloat
	  if(portStates[1].pinState==1 && portStates[3].pinState==0 && getStateStartStop()==0){start();}
	  if(portStates[2].pinState==1 && getStateStartStop()==3){stop();}


	  //if(portStates[1].pinState==1 && portStates[2].pinState==1){emergencyStop();}

    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartInputs */
/**
* @brief Function implementing the inputs thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartInputs */
void StartInputs(void *argument)
{
  /* USER CODE BEGIN StartInputs */

portStates[0].Pin = UserButton_Pin;
portStates[0].Port=UserButton_GPIO_Port;
portStates[0].activePinState = GPIO_PIN_RESET;
portStates[0].inputName = userButton;
portStates[0].Cycles =10;
portStates[1].Pin = startButton_Pin;
portStates[1].Port =  startButton_GPIO_Port;
portStates[1].activePinState = GPIO_PIN_RESET;
portStates[1].inputName = startButton;
portStates[1].Cycles = cycles;
portStates[2].Pin = stopButton_Pin;
portStates[2].Port = stopButton_GPIO_Port;
portStates[2].activePinState = GPIO_PIN_RESET;
portStates[2].inputName = stopButton;
portStates[2].Cycles = cycles;
portStates[3].Pin = HighLevelBulb_Pin;
portStates[3].Port = HighLevelBulb_GPIO_Port;
portStates[3].activePinState = GPIO_PIN_SET;
portStates[3].inputName = highLevelBulb;
portStates[3].Cycles = cycles;
portStates[4].Pin = LowLevelBulb_Pin;
portStates[4].Port = LowLevelBulb_GPIO_Port;
portStates[4].activePinState = GPIO_PIN_RESET;
portStates[4].inputName = lowLevelBulb;
portStates[4].Cycles = cycles;
portStates[5].Pin = motionSensor_Pin;
portStates[5].Port = motionSensor_GPIO_Port;
portStates[5].activePinState = GPIO_PIN_SET;
portStates[5].inputName = motionSensor;
portStates[5].Cycles = cycles;
  /* Infinite loop */
  for(;;)
  {

	 for (uint16_t i=0;i<portStatesLenght;i++)
	 {
		 checkPort(&portStates[i],portStates[i].Cycles);
	 }

	 //if(portStates[1].pinState==1 && portStates[2].pinState==1){emergencyStop();}

osDelay(1);

  }

  /* USER CODE END StartInputs */
}

/* USER CODE BEGIN Header_StartAdc */
/**
* @brief Function implementing the adc thread.
* @param argument: Not used
* @retval None
*/
uint16_t getArray_getMedian(uint16_t* dataForShift,u16 dataPositionInArray,u16 dataPeriodInArray, u16 dataLen){

	//uint16_t shiftedData[dataLen]={0,};
	for(uint16_t i =0; i<dataLen;i++){
		shiftedData[i]=dataForShift[(i*dataPeriodInArray) + dataPositionInArray];
	}
	return getMedianNum(shiftedData, dataLen);

}

uint16_t getMedianNum(uint16_t* bArray, uint16_t iFilterLen)
{
      uint16_t bTab[iFilterLen];
      for (uint8_t i = 0; i<iFilterLen; i++)
      bTab[i] = bArray[i];
      uint16_t i, j, bTemp;
      for (j = 0; j < iFilterLen - 1; j++)
      {
      for (i = 0; i < iFilterLen - j - 1; i++)
          {
        if (bTab[i] > bTab[i + 1])
            {
        bTemp = bTab[i];
            bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
         }
      }
      }
      if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
      else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      return bTemp;
}



/* USER CODE END Header_StartAdc */
void StartAdc(void *argument)
{
  /* USER CODE BEGIN StartAdc */
	if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adcChannelsValues, lengthOfAdcChannelsValues)!= HAL_OK){LCD_String("Error");};
  /* Infinite loop */
  for(;;)
  {
	  if(adcReadyFlag!=0)
	  {
		  adcReadyFlag = 0;
        //LCD_String("ADC");
		  	 	  	   adcAverageValues[0] = getArray_getMedian(adcChannelsValues,0,6,5);//average(adcChannelsValues,0);//(uint16_t)(adcChannelsValues[5]);CH0 TDS Concentrate 0-3.3v
		  	 	  	   adcAverageValues[1] = getArray_getMedian(adcChannelsValues,1,6,5);//average(adcChannelsValues,1);//(uint16_t)adcChannelsValues[6]; CH1 TDS Permeate 0-3.3v
		  	 	  	   adcAverageValues[2] = getArray_getMedian(adcChannelsValues,2,6,5);//average(adcChannelsValues,2);//(uint16_t)adcChannelsValues[7]; CH4 Reserved for  Before source Pressure 4-20 mA
		  	 	  	   adcAverageValues[3] = getArray_getMedian(adcChannelsValues,3,6,5);//average(adcChannelsValues,3);//(uint16_t)adcChannelsValues[8]; CH6 Temp 23-172 kOm to 3.3v
		  	 	  	   adcAverageValues[4] = getArray_getMedian(adcChannelsValues,4,6,5);//average(adcChannelsValues,4);//(uint16_t)adcChannelsValues[9]; CH7 Source Pressure 4-20 mA
		  	 	  	   adcAverageValues[5] = getArray_getMedian(adcChannelsValues,5,6,5);//average(adcChannelsValues,4);//(uint16_t)adcChannelsValues[9]; CH8 High Pressure 4-20 mA
		  	 	  	   __HAL_ADC_CLEAR_FLAG(&hadc1, EOF);
		  	 	  	 if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adcChannelsValues, lengthOfAdcChannelsValues)!=HAL_OK){LCD_String("Error");};
		  	 	  	// LCD_String("ADC");
	  }
	 // sprintf(lcdString,"%lu",HAL_ADC_GetError(&hadc1));
	  //LCD_String(lcdString);

      osDelay(1000);
  }
  /* USER CODE END StartAdc */
}

/* USER CODE BEGIN Header_StartGetFlows */
/**
* @brief Function implementing the getFlowsTask thread.
* @param argument: Not used
* @retval None
*/
void ratesCalculation(void)
{
if(frequences[0].flow>0){
rates.regectionRate = (frequences[3].flow*100)/frequences[0].flow;
rates.recyclingRate = (frequences[2].flow*100)/frequences[0].flow;
rates.membraneFlow = frequences[0].flow + frequences[2].flow;
}
}

void frequencyCalc(frequencyCalcTypedef* fr)
{
fr->frequence = dividend /((fr->CCRx[1]-fr->CCRx[0])*1.0);
switch(fr->sensorType)
{
case 1 : {fr->flow = (frequences[0].flow/(frequences[1].frequence+frequences[3].frequence))*fr->frequence;}//{fr->flow = fr->frequence/11;break;}
case 2 : {fr->flow = fr->frequence/6.6;break;}
case 3 : {fr->flow = (fr->frequence+8)/6;break;}
}
ratesCalculation();
}

void getCapturedValue(frequencyCalcTypedef* fr)
{
	fr->CCRx[0]=fr->CCRx[1];
	fr->CCRx[1]=HAL_TIM_ReadCapturedValue(fr->tim,fr->channel);
	fr->liveTime = timeUnactivityToClearFrequency;
	fr->compFlag=0;
}



/* USER CODE END Header_StartGetFlows */
void StartGetFlows(void *argument)
{
  /* USER CODE BEGIN StartGetFlows */

	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
	frequences[0].channel = TIM_CHANNEL_1; // Source
	frequences[0].tim = &htim2;
	frequences[0].sensorType = 3; //G 1"
	frequences[1].channel = TIM_CHANNEL_2; // drain
	frequences[1].tim = &htim2;
	frequences[1].sensorType = 1; //G 1/2
	frequences[2].channel = TIM_CHANNEL_3; // recycle
	frequences[2].tim = &htim2;
	frequences[2].sensorType = 2; //G 3/4
	frequences[3].channel = TIM_CHANNEL_4; //permeate
	frequences[3].tim = &htim2;
	frequences[3].sensorType = 1; //G 1/2
  /* Infinite loop */
  for(;;)
  {
	  if(globalTim2Flag)
		  {
		  	  globalTim2Flag=0;
			  for(uint8_t q =0; q< frequencesLength;q++)
				  {
					  if(frequences[q].compFlag!=0){getCapturedValue(&frequences[q]);}
				  }
		  }

	  if(portStates[1].pinState==1 && portStates[2].pinState==1){emergencyStop();}
    osDelay(1);
  }
  /* USER CODE END StartGetFlows */
}

/* USER CODE BEGIN Header_StartCheckValues */
/**
* @brief Function implementing the checkValues thread.
* @param argument: Not used
* @retval None
*/
uint8_t getHighPressurePumpStartFlag(void)
{
return highPressurePumpStartFlag;
}


/* USER CODE END Header_StartCheckValues */
void StartCheckValues(void *argument)
{
  /* USER CODE BEGIN StartCheckValues */
	uint16_t aw =0;

  /* Infinite loop */
  for(;;)
  {

	      taskENTER_CRITICAL( );

	  	  	  sensorValuesArr[0] = getTds(adcAverageValues[0]);//concentrateTDS
	  		  sensorValuesArr[1] = getTds(adcAverageValues[1]);//permeateTDS
	  		  sensorValuesArr[2] = (float)adcAverageValues[2];//reserved
	  		  sensorValuesArr[3] = getPressure(adcAverageValues[5],2.5,1);//membranePressure
	  		  sensorValuesArr[4] = getTemp(adcAverageValues[3]);//waterTemp
	  	      sensorValuesArr[5] = getPressure(adcAverageValues[4],0.5,1);//sourcePressure
	  	      frequencyCalc(&frequences[0]);
	  		  sensorValuesArr[6] = frequences[0].flow;//sourceFlow
	  		  frequencyCalc(&frequences[1]);
	  		  sensorValuesArr[7] = frequences[1].flow;//drainFlow
	  		  frequencyCalc(&frequences[2]);
	  		  sensorValuesArr[8] = frequences[2].flow;//recycleFlow
	  		  frequencyCalc(&frequences[3]);
	  		  sensorValuesArr[9] = frequences[3].flow;//permeateFlow
	  		  sensorValuesArr[10] = rates.membraneFlow;
	  		  sensorValuesArr[11] = rates.recyclingRate;
	  		  sensorValuesArr[12] = rates.regectionRate;

	  		      sprintf(&window[0][0],(char*)formatsShort[0],rtc.Hour,rtc.Min,rtc.Sec,rtc.Date,rtc.Month,rtc.Year);

	  			  if(getStateStartStop()==0)
	  			  {
	  				  if(autoManStartFlag==0){sprintf(&window[1][0],(char*)formatsShort[22]);}else{sprintf(&window[1][0],(char*)formatsShort[21]);}
	  			  }
	  			  else
	  			  {
	  				if(autoManStartFlag==0){sprintf(&window[1][0],(char*)formatsShort[24]);}else{sprintf(&window[1][0],(char*)formatsShort[23]);}
	  				  //sprintf(&window[1][0],(char*)formatsShort[20]);
	  			  }

	  			          for (ui8 x=2; x<sensorValuesArrLenght+2;x++)
	  			          {
	  			        	  sprintf(&window[x][0],(char*)formatsShort[x],sensorValuesArr[x-2]);
	  			          }

	  			  sprintf(&window[15][0],(char*)formatsShort[15]);
	  			  sprintf(&window[16][0],(char*)formatsShort[16],portStates[0].pinState);
	  			  sprintf(&window[17][0],(char*)formatsShort[17],portStates[1].pinState,portStates[2].pinState);
	  			  sprintf(&window[18][0],(char*)formatsShort[18],portStates[3].pinState,portStates[4].pinState);
	  			  sprintf(&window[19][0],(char*)formatsShort[19],portStates[5].pinState);



	  			  if(getStateStartStop()==3 && getStartStage() ==0) ///////////////////////////////////// In progreso parameter check
	  				  {

	  					  if(sensorValuesArr[5]<0.2){emergencyStop();} //Check source pressure
	  					 if(sensorValuesArr[3]<1 || sensorValuesArr[3] >1.95 ) {emergencyStop();}  // Check membrane pressure
	  				  }
	  			 if(sensorValuesArr[5]<0.2){highPressurePumpStartFlag =0;}else{highPressurePumpStartFlag=1;}

	  			  ////////////////////////////////////////////////////////////////////////////////////////////////

	  	  taskEXIT_CRITICAL( );


	  		  //sensorV[0].formatNum = 0;
	  		  //sprintf((char*)sensorV[0].param,(char*)formatsShort[(sensorV[0].formatNum)],rtc.Hour,rtc.Min,rtc.Sec,rtc.Date,rtc.Month,rtc.Year);


	  		  /*
	  		   *portStates[0].Pin = UserButton_Pin;
				portStates[0].Port=UserButton_GPIO_Port;
				portStates[0].activePinState = GPIO_PIN_RESET;
				portStates[0].inputName = userButton;
				portStates[1].Pin = startButton_Pin;
				portStates[1].Port =  startButton_GPIO_Port;
				portStates[1].activePinState = GPIO_PIN_RESET;
				portStates[1].inputName = startButton;
				portStates[2].Pin = stopButton_Pin;
				portStates[2].Port = stopButton_GPIO_Port;
				portStates[2].activePinState = GPIO_PIN_RESET;
				portStates[2].inputName = stopButton;
				portStates[3].Pin = HighLevelBulb_Pin;
				portStates[3].Port = HighLevelBulb_GPIO_Port;
				portStates[3].activePinState = GPIO_PIN_SET;
				portStates[3].inputName = highLevelBulb;
				portStates[4].Pin = LowLevelBulb_Pin;
				portStates[4].Port = LowLevelBulb_GPIO_Port;
				portStates[4].activePinState = GPIO_PIN_RESET;
				portStates[4].inputName = lowLevelBulb;
				portStates[5].Pin = motionSensor_Pin;
				portStates[5].Port = motionSensor_GPIO_Port;
				portStates[5].activePinState = GPIO_PIN_SET;
				portStates[5].inputName = motionSensor;
	  		   */



	if (aw >= timeRenovationPeriod)
	{
		aw=0;


	}
	else
	{
		aw++;
	}

    if(varTimeToBackLightOff <=0)   // && LCD_getBackLightState()!=0)
    {
    	LCDOnOffFlag = 0;
    	//LCD_offLed();
    }
    else if(varTimeToBackLightOff>0)
    {
    	varTimeToBackLightOff--;
    	LCDOnOffFlag=1;
    }
    if(portStates[5].pinState==1){varTimeToBackLightOff = timeToBackLightOff;}
    if(portStates[0].pinState==1){varTimeToBackLightOff = timeToBackLightOff;}
    for(uint16_t y=0;y<frequencesLength;y++)
    {
    	if(frequences[y].liveTime<=0){frequences[y].frequence=0;frequences[y].flow=0;}else{frequences[y].liveTime--;}
        //HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
        //osDelay(100);
    }
    if(frequences[0].flow<=0){rates.membraneFlow=0;rates.recyclingRate=0;rates.regectionRate=0;}



    osDelay(10);

  }
  /* USER CODE END StartCheckValues */
}

/* USER CODE BEGIN Header_StartUsart */
/**
* @brief Function implementing the usart thread.
* @param argument: Not used
* @retval None
* */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

 if(huart->Instance ==USART1){uartFlag=0;};


}


/* USER CODE END Header_StartUsart */
void StartUsart(void *argument)
{
  /* USER CODE BEGIN StartUsart */
	//ui8 zero[2]={10,13};
  /* Infinite loop */

	  for(;;)
	    {

		 //if(uartFlag==0){
          //uartFlag=1;
		  /*
          for(ui8 q=0;q<20;q++){


          				   while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY){};
                           if ( HAL_UART_Transmit_IT(&huart1,(uint8_t*)&window[q][0],20)!=HAL_OK){}
                           osDelay(20);
                           while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY){};
                           osDelay(10);
                           if ( HAL_UART_Transmit_IT(&huart1,(uint8_t*)zero,2)!=HAL_OK){}
          }
          */
          HAL_IWDG_Refresh(&hiwdg);
		  osDelay(30);

	   }

  /* USER CODE END StartUsart */
}

/* USER CODE BEGIN Header_StartLcd */
/**
* @brief Function implementing the lcd thread.
* @param argument: Not used
* @retval None
*/
void tim7(void)
{
	//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
	lcdWindowRenewalFlag=1;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
	if(hi2c->Instance==I2C1){i2cErrorFlag=1;};
	HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
	        //osDelay(100);
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_ErrorCallback could be implemented in the user file
   */
}





void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  if(hi2c->Instance==I2C1){i2cFlag=1;};

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_MasterTxCpltCallback could be implemented in the user file
   */
}
void setLCDOnOffFlag(void){LCDOnOffFlag = 1;}

/* USER CODE END Header_StartLcd */
void StartLcd(void *argument)
{
  /* USER CODE BEGIN StartLcd */
  /* Infinite loop */

	HAL_TIM_Base_Start_IT(&htim7);

	LCD_ini();
	DS3231_Init(&hi2c1);
	ui8 shift =0;

	//DS3231_SetTime(&rtc);

			  /* Configure Alarm1 */
 			  //DS3231_ClearAlarm1();
			  //DS3231_SetAlarm1(ALARM_MODE_ONCE_PER_SECOND, 0, 0, 0, 0);
			  //DS3231_SetAlarm1(ALARM_MODE_SEC_MATCHED, 0, 0, 0, 30);

	                // DS3231_GetTime(&rtc);
			 	 	 //DS3231_ReadTemperature(&rtcTemp);
			 	 	 //ReadRegister(DS3231_REG_STATUS, &regVal);
				    //  if(regVal & DS3231_STA_A1F)
				    //  {
				    //    regVal &= ~DS3231_STA_A1F;
				    //    WriteRegister(DS3231_REG_STATUS, regVal);
				    //  }




  for(;;)
  {

	  //if (getStateStartStop()!=0 && LCD_getBackLightState()==0){LCD_onLed();}
	  if (LCDOnOffFlag==1 && getStateStartStop()==0 && LCD_getBackLightState()==0 && i2cErrorFlag==0)
	 	  {
	 			 LCD_onLed();

	 	  }

		  if(i2cErrorFlag==1)
	  {
		  MX_I2C1_Init();
		  LCD_ini();
		  DS3231_Init(&hi2c1);
		  i2cErrorFlag=0;
	  }



	  if(LCDOnOffFlag == 0 && getStateStartStop()==0 && LCD_getBackLightState()==1){ LCD_offLed(); }









	     if(lcdWindowRenewalFlag==1 && getInStart()==0 && getInStop()==0)
	     {

	    	if (i2cFlag){i2cFlag=0; DS3231_GetTime(&rtc);}

            if (shift>=17){shift=0;}
				 for(ui8 q=0;q<4;q++){
					 //taskENTER_CRITICAL( );
					 LCD_StringPos(&window[q+shift][0],0,q);
					 //taskEXIT_CRITICAL( );
					 osDelay(5);

				 }

		    if(portStates[0].pinState ==1)
		    {
		    shift++;

		    }


	    	 lcdWindowRenewalFlag =0;



	    }

	  HAL_IWDG_Refresh(&hiwdg);
	  osDelay(10);
  }
  /* USER CODE END StartLcd */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
ui8 getCircularIndex(ui8 index)
{
if(index <=79){return index;}else{return index-80;}


}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) // колбек по захвату
{
      if (htim->Instance == TIM2){
    	  	globalTim2Flag=1;
        	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){frequences[0].compFlag=1;}
        	else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){frequences[1].compFlag=1;}
        	else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){frequences[2].compFlag=1;}
        	else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4){frequences[3].compFlag=1;}


        }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

	//LCD_String("12345");
	HAL_ADC_Stop(&hadc1);
	//HAL_ADC_Stop_DMA(&hadc1);
	adcReadyFlag =1;
	//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
	//osDelay(500);

}
void checkPort(portStateTypedef* portState, uint16_t cycles)
{

		if(HAL_GPIO_ReadPin(portState->Port,portState->Pin) != portState->activePinState)
		{
			portState->Count=0; portState->pinState=0;
		}
		else
		{
			if(portState->Count >= cycles)
			{
				portState->pinState = 1;
			}
			else
				{

				if (portState->Count < cycles) {portState->Count++;}
				}
		}

}

//float getSourcePressure(){return getPressure(adcAverageValues[4],5,1);}
uint8_t getPortState(uint8_t port){return portStates[port].pinState;}


/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
