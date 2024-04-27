/*
 * pt100.c
 *
 *  Created on: Dec 10, 2023
 *      Author: Guim
 */

#include <pt100.h>

void pt100UpdateTemperature(PT100 * pt100){
//	uint16_t vrefint_cal;
//	vrefint_cal = *((uint16_t *)VREFINT_CAL_ADDRESS);
//	float Vout = (float)*analog->raw_pt100 / 4096.0 * 3.3 * (float)vrefint_cal / (float)*analog->raw_int_ref;
	float Vout_rel = (float)*pt100->analog->raw_pt100 / 4096.0;
	float Rpt100 = (5000000. + 571000.*Vout_rel)/(50571. - 571.*Vout_rel);
	pt100->temperature = PT100A - sqrt(PT100B - PT100C * Rpt100);
}
