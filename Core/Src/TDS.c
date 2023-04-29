/*
 * TDS.c
 *
 *  Created on: Dec 13, 2020
 *      Author: Marid
 */
#include "TDS.h"
#define VREF 3.3      // analog reference voltage(Volt) of the ADC
float constTemp = 7;
float temp=0;



   float getTds(uint16_t adcValue)
   {

	  float sensorTemp =  getTemp();
	  if(!sensorTemp>25 && !sensorTemp<1){temp=sensorTemp;}else{temp=constTemp;}
      float averageVoltage = adcValue * VREF / 4095.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      float compensationCoefficient=1.0+0.02*(temp-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationVolatge=averageVoltage/compensationCoefficient;  //temperature compensation
      return(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; //convert voltage value to tds value

   }

