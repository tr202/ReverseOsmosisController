/*
 * pressureSensor.c
 *
 *  Created on: Dec 14, 2020
 *      Author: Marid
 */
#include "pressureSensor.h"
float V=3.3;     // [V]
float R=150; //ohm
float getPressure(uint16_t adc, float range, float units ) //1 mPa 1,013 bar
{
	float Vout=adc*(V/4095.0); // calc for ntc
	float current = Vout/R;
	return ((range/0.016)*(current-0.004));

}


