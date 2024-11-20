/*
 * display.c
 *
 *  Created on: Dec 10, 2023
 *      Author: Guim
 */


#include <display.h>
#include <ssd1306.h>
#include <display_lookup.h>
#include <stdio.h>

HAL_StatusTypeDef displayInit(SSD1306Display * ssd1306){
	for (uint8_t p = 0; p < SSD1306_PAGES; p++){
		memset(ssd1306->display_data[p], 0x00, SSD1306_WIDTH);
	}

	return ssd1306UpdateDisplay(ssd1306);
}

HAL_StatusTypeDef displayGlyph(SSD1306Display * ssd1306, const uint8_t * glyph, uint8_t x_size, uint8_t p_size, uint8_t x0, uint8_t p0){
	for (uint8_t p = 0; p < p_size; p++){ // Pages
		memcpy(&ssd1306->display_data[p0+p][x0], &glyph[p*x_size], x_size);
	}
	return HAL_OK;
}

HAL_StatusTypeDef writeTemp(SSD1306Display * ssd1306, float temperature, uint8_t is_humidity){

	// Clear display
	for (uint8_t p = 0; p < SSD1306_PAGES; p++){
		memset(ssd1306->display_data[p], 0x00, SSD1306_WIDTH);
	}

	uint8_t text[DEC_PLACES+3+2];
	uint8_t intPart = (uint8_t)temperature;
	uint8_t decimalPart = (uint8_t)((temperature - (float)intPart) * 100);
	uint8_t temp;

	if (intPart >= 10){
		temp = intPart / 10;
		text[0] = temp + INT_TO_CHAR;
		text[1] = intPart - temp * 10 + INT_TO_CHAR;
	}
	else{
		temp = intPart / 10;
		text[0] = ' ';
		text[1] = intPart + INT_TO_CHAR;
	}

	text[2] = '.';

	if (decimalPart >= 10){
		temp = decimalPart / 10;
		text[3] = temp + INT_TO_CHAR;
		text[4] = decimalPart - temp * 10 + INT_TO_CHAR;
	}
	else{
		temp = decimalPart / 10;
		text[3] = '0';
		text[4] = decimalPart + INT_TO_CHAR;
	}

	if (is_humidity){
		text[5] = ' ';
		text[6] = ' ';
	}
	else{
		text[5] = 'd';
		text[6] = 'C';
	}

	uint8_t w[] = {L_SPACE, L_SPACE + DIG_SPACE, L_SPACE + 2*DIG_SPACE, L_SPACE + 2*DIG_SPACE + DOT_SPACE
			, L_SPACE + 3*DIG_SPACE + DOT_SPACE, L_SPACE + 4*DIG_SPACE + DOT_SPACE
			, L_SPACE + 4*DIG_SPACE + DOT_SPACE + DEG_SPACE};
	for (uint8_t i = 0; i < (DEC_PLACES+3+2); i++){
		const uint8_t * glyph = ssd1306_get_glyph(text[i]);
		displayGlyph(ssd1306, &glyph[2], glyph[0], glyph[1]/8, w[i], 1);
	}

	return HAL_OK;
}

HAL_StatusTypeDef bootLogo(SSD1306Display * ssd1306){
	const uint8_t * glyph = ssd1306_get_glyph('l');
	displayGlyph(ssd1306, &glyph[2], glyph[0], glyph[1]/8, 20, 0);
	return ssd1306UpdateDisplay(ssd1306);
}

HAL_StatusTypeDef drawBar(SSD1306Display * ssd1306, uint8_t percentage){
	if (percentage > 100) percentage = 100;
	ssd1306->display_data[6][12] = 0x3C;
	ssd1306->display_data[6][13] = 0xFF;
	memset(&ssd1306->display_data[6][14], 0xFF, percentage);
	memset(&ssd1306->display_data[6][14+percentage], 0x81, 100-percentage);
	ssd1306->display_data[6][114] = 0xFF;
	ssd1306->display_data[6][115] = 0x3C;
	return HAL_OK;
}
