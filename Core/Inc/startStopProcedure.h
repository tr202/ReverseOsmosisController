/*
 * startStopProcedure.h
 *
 *  Created on: Dec 14, 2020
 *      Author: Marid
 */

#ifndef INC_STARTSTOPPROCEDURE_H_
#define INC_STARTSTOPPROCEDURE_H_

#include "main.h"
bool start(void);
bool stop(void);
uint8_t getInStart(void);
uint8_t getInStop(void);
uint8_t getStateStartStop(void);
uint8_t getStartStage(void);
bool emergencyStop(void);
#endif /* INC_STARTSTOPPROCEDURE_H_ */
