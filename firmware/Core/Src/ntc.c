/*
 * ntc.c
 *
 *  Created on: Dec 10, 2023
 *      Author: Guim
 */


#include <ntc.h>

void NTCUpdateTemperature(NTC * ntc){
	float Vout_rel = (float)*ntc->analog->raw_ntc / 4096.0;
	float Rntc = NTC_PULLUP * Vout_rel / (1 - Vout_rel) ;
	ntc->temperature = NTC_T0 * NTC_BETA / (NTC_T0 * log(Rntc / NTC_R0) + NTC_BETA) - 273.15;
}
