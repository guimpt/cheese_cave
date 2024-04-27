/*
 * pid_loop.h
 *
 *  Created on: Jan 24, 2023
 *      Author: Guim
 */

#ifndef INC_PIDLOOP_H_
#define INC_PIDLOOP_H_

#include <main.h>

/*
 * DEFINES
 */

#define MULTIPLIER 1024



/*
 * STRUCTS & ENUMS
 */

typedef struct{
	// Interrupt flag
	uint8_t update_flag;

	// Input, output and feedback
	int32_t y_k;
	int32_t u_k; // Not saturated

	// PID gains
	int32_t kp_int;
	int32_t ki_int;

	// Saturation
	int32_t Umax_int;
	int32_t Umin_int;

	// Other signals
	int32_t xi_k;
	int32_t xp_k;
} PID;

/*
 * FUNCTIONS
 */

void pidLoop_Initialize(PID *pid);
void pidLoop_Update(PID *pid);

#endif /* INC_PIDLOOP_H_ */
