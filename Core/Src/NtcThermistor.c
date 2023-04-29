/*
 * NtcThermistor.c
 *
 *  Created on: Dec 14, 2020
 *      Author: Marid
 */

#include "NtcThermistor.h"
#include "math.h"

float Vref =4.76;
float Vin=3.3;     // [V]
float Rt=38000;    // Resistor t [ohm]
float R0=50000;    // value of rct in T0 [ohm]
float T0=298.15;   // use T0 in Kelvin [K] 25 celsius
float Vout=0.0;    // Vout in A0
float Rout=0.0;    // Rout in A0
// use the datasheet to get this data.
float T1=273.15;      // [K] in datasheet 0º C
float T2=323.15;      // [K] in datasheet 50 C
float RT1=164000;   // [ohms]  resistence in T1
float RT2=17750;    // [ohms]   resistence in T2
float beta=0.0;    // initial parameters [K]
float Rinf=0.0;    // initial parameters [ohm]
float TempK=0.0;   // variable output
float TempC=0.0;   // variable output

#define beta (ln(RT1/RT2))/((1/T1)-(1/T2))
#define Rinf R0*exp(-beta/T0)



float getTemp(uint16_t adc)
{
	Vout=adc*(Vin/4095.0); // calc for ntc
	Rout = ((Vin-Vout)/(Vout/Rt));
	float lnbet =(log(Rout)-log(17750))/3950;
	TempK = 1/(lnbet+(1/323.15));

	return TempK-273.15;
}







