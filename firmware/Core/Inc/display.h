/*
 * display.h
 *
 *  Created on: Dec 10, 2023
 *      Author: Guim
 */

#ifndef SRC_DISPLAY_H_
#define SRC_DISPLAY_H_

#include <main.h>
#include <ssd1306.h>

#define DEC_PLACES 2
#define INT_TO_CHAR 48

#define L_SPACE   13
#define DIG_SPACE 18
#define DOT_SPACE 6
#define DEG_SPACE 8

HAL_StatusTypeDef displayInit(SSD1306Display * ssd1306);
HAL_StatusTypeDef bootLogo(SSD1306Display * ssd1306);
HAL_StatusTypeDef clearDisplay(SSD1306Display * ssd1306);
HAL_StatusTypeDef writeTemp(SSD1306Display * ssd1306, float temperature, uint8_t is_humidity);
HAL_StatusTypeDef drawBar(SSD1306Display * ssd1306, uint8_t percentage);


#endif /* SRC_DISPLAY_H_ */
