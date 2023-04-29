/*
 * testProcedure.c
 *
 *  Created on: 11 дек. 2020 г.
 *      Author: Marid
 */

#include "testProcedure.h"



void testRelay(void)
{


	toggle(R0WashValve_GPIO_Port,R0WashValve_Pin);
	osDelay(500);
	HAL_GPIO_TogglePin(R1AcidClean_GPIO_Port, R1AcidClean_Pin);
	osDelay(500);
	toggle(R2SourceValve_GPIO_Port, R2SourceValve_Pin);
	osDelay(500);
	toggle(R3DrainValve_GPIO_Port,R3DrainValve_Pin);
	osDelay(500);
	toggle(Relay4_GPIO_Port,Relay4_Pin);
	osDelay(500);
	toggle(Relay5_GPIO_Port,Relay5_Pin);
	osDelay(500);
	toggle(Relay6_GPIO_Port,Relay6_Pin);
	osDelay(500);
	toggle(Relay7_GPIO_Port,Relay7_Pin);
	osDelay(500);

}



