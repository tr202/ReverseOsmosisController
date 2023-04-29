/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "testProcedure.h"
#include "lcd.h"
#include "stdio.h"
#include "TDS.h"
#include "Ntcthermistor.h"
#include "pressureSensor.h"
#include "string.h"
#include "stm32_ds3231.h"
#include <stdlib.h>
#include <stdbool.h>
#include "startStopProcedure.h"
#include "iwdg.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct
{
uint8_t* data;
uint16_t dataSize;
}usartStructTypedef;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void tim7(void);
uint8_t getHighPressurePumpStartFlag(void);
void toggleFlag(uint8_t flag);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define UserButton_Pin GPIO_PIN_13
#define UserButton_GPIO_Port GPIOC
#define R0WashValve_Pin GPIO_PIN_0
#define R0WashValve_GPIO_Port GPIOC
#define R1AcidClean_Pin GPIO_PIN_1
#define R1AcidClean_GPIO_Port GPIOC
#define R2SourceValve_Pin GPIO_PIN_2
#define R2SourceValve_GPIO_Port GPIOC
#define R3DrainValve_Pin GPIO_PIN_3
#define R3DrainValve_GPIO_Port GPIOC
#define GreenLD2_Pin GPIO_PIN_5
#define GreenLD2_GPIO_Port GPIOA
#define Relay4_Pin GPIO_PIN_4
#define Relay4_GPIO_Port GPIOC
#define Relay5_Pin GPIO_PIN_5
#define Relay5_GPIO_Port GPIOC
#define permeateFlow_Pin GPIO_PIN_2
#define permeateFlow_GPIO_Port GPIOB
#define recycleFlow_Pin GPIO_PIN_10
#define recycleFlow_GPIO_Port GPIOB
#define Relay6_Pin GPIO_PIN_6
#define Relay6_GPIO_Port GPIOC
#define Relay7_Pin GPIO_PIN_7
#define Relay7_GPIO_Port GPIOC
#define startButton_Pin GPIO_PIN_8
#define startButton_GPIO_Port GPIOC
#define stopButton_Pin GPIO_PIN_9
#define stopButton_GPIO_Port GPIOC
#define sourceFlow_Pin GPIO_PIN_15
#define sourceFlow_GPIO_Port GPIOA
#define HighPressurePump_Pin GPIO_PIN_10
#define HighPressurePump_GPIO_Port GPIOC
#define AirPump_Pin GPIO_PIN_11
#define AirPump_GPIO_Port GPIOC
#define MainPower_Pin GPIO_PIN_12
#define MainPower_GPIO_Port GPIOC
#define motionSensor_Pin GPIO_PIN_2
#define motionSensor_GPIO_Port GPIOD
#define concentrateFlow_Pin GPIO_PIN_3
#define concentrateFlow_GPIO_Port GPIOB
#define HighLevelBulb_Pin GPIO_PIN_4
#define HighLevelBulb_GPIO_Port GPIOB
#define LowLevelBulb_Pin GPIO_PIN_5
#define LowLevelBulb_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define toggle HAL_GPIO_TogglePin
#define od500 osDelay(500)
#define userButton 1
#define startButton 2
#define stopButton 3
#define highLevelBulb 4
#define lowLevelBulb 5
#define motionSensor 6
#define ui8 uint8_t
#define ui16 uint16_t
#define ui32 uint32_t
#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t
//#define HAL_TIMEOUT_VALUE 0xFFFFFFFF

#define countof(a) (sizeof(a) / sizeof(*(a)))
//HAL_UART_Transmit(&hlpuart1, (uint8_t*)&Part1TxBuffer, countof(Part1TxBuffer)-1, HAL_TIMEOUT_VALUE);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
