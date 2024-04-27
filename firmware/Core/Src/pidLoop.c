/*
 * pid_loop.c
 *
 *  Created on: Jan 24, 2023
 *      Author: Guim
 */


#include <pidLoop.h>

/**
  * @brief pidLoop class initialization
  * @param PID data type
  * @retval none
  */
void pidLoop_Initialize(PID *pid){
	pid->update_flag = 0;

	// Other signals
	pid->xi_k = 0;
	pid->xp_k = 0;
}

/**
  * @brief pidLoop Update (+1 loop cycle)
  * @param PID data type
  * @retval none
  */
void pidLoop_Update(PID *pid){
	// Integral
	if(pid->u_k == pid->y_k){
		pid->xi_k += ((((int64_t)pid->xp_k * (int64_t)pid->kp_int) / MULTIPLIER) * (int64_t)pid->ki_int) / MULTIPLIER;
	}

	// Control action
	pid->u_k = ((int64_t)pid->kp_int * (int64_t)pid->xp_k) / MULTIPLIER + pid->xi_k;

	// Output
	if(pid->u_k > pid->Umax_int) pid->y_k = pid->Umax_int;
	else if(pid->u_k < pid->Umin_int) pid->y_k = pid->Umin_int;
	else pid->y_k = pid->u_k;

}

