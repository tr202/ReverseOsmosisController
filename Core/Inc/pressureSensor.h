/*
 * pressureSensor.h
 *
 *  Created on: Dec 14, 2020
 *      Author: Marid
 */

#ifndef INC_PRESSURESENSOR_H_
#define INC_PRESSURESENSOR_H_
#include "stdio.h"
#include "math.h"
#include "stdlib.h"

float getPressure(uint16_t adc, float range, float units);


#endif /* INC_PRESSURESENSOR_H_ */
