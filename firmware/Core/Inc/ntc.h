/*
 * ntc.h
 *
 *  Created on: Dec 10, 2023
 *      Author: Guim
 */

#ifndef INC_NTC_H_
#define INC_NTC_H_

#include <main.h>
#include <math.h>

#define NTC_PULLUP	10000.f
#define NTC_R0		10000.f
#define NTC_BETA	3568.f
#define NTC_T0		298.15f

typedef struct {
	// GPIO
	ANALOG * analog;

	float temperature;
} NTC;

void NTCUpdateTemperature(NTC * ntc);

#endif /* INC_NTC_H_ */
