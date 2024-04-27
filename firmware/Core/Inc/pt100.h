/*
 * pt100.h
 *
 *  Created on: Dec 10, 2023
 *      Author: Guim
 */

#ifndef INC_PT100_H_
#define INC_PT100_H_


/* Includes */
#include <main.h>
#include <math.h>

/* Defines */
#define PT100A 3383.809523809524f
#define PT100B 13181768.62502577f
#define PT100C 17316.01731601731f
#define FILT   0.05

/* Structs */
typedef struct {
	// GPIO
	ANALOG * analog;

	float temperature;
} PT100;


/* Functions */
void pt100UpdateTemperature(PT100 * pt100);

#endif /* INC_PT100_H_ */
