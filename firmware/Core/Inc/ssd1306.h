/*
 * ssd1306.h
 *
 *  Created on: Dec 9, 2023
 *      Author: Guim
 */

#ifndef INC_SSD1306_H_
#define INC_SSD1306_H_

/* Includes */
#include <main.h>


/* Defines */
#define SSD1306_ADDRESS						  (0x3C << 1)
#define SSD1306_WIDTH						  128U
#define SSD1306_HEIGTH						  64U
#define SSD1306_PAGES						  SSD1306_HEIGTH/8U
#define SSD1306_I2C_TIEMOUT					  100

// SSD1306 Display Driver Registers
#define SSD1306_CONTROL_BYTE_CMD  			  0x00  // Co = 0, D/C# = 0
#define SSD1306_CONTROL_BYTE_CMDs  			  0x80  // Co = 1, D/C# = 0
#define SSD1306_CONTROL_BYTE_DATA 			  0x40  // Co = 0, D/C# = 1
#define SSD1306_CONTROL_BYTE_DATAs 			  0xC0  // Co = 1, D/C# = 1

// Fundamental Commands
#define SSD1306_SET_CONTRAST_CONTROL          0x81
#define SSD1306_DISPLAY_ALL_ON_RESUME         0xA4
#define SSD1306_DISPLAY_ALL_ON                0xA5
#define SSD1306_NORMAL_DISPLAY                0xA6
#define SSD1306_INVERT_DISPLAY                0xA7
#define SSD1306_DISPLAY_OFF                   0xAE
#define SSD1306_DISPLAY_ON                    0xAF

// Addressing Setting Commands
#define SSD1306_SET_MEMORY_ADDR_MODE          0x20
#define SSD1306_SET_COLUMN_ADDR               0x21
#define SSD1306_SET_PAGE_ADDR                 0x22

// Hardware Configuration Commands
#define SSD1306_SET_DISPLAY_START_LINE        0x40
#define SSD1306_SET_SEGMENT_REMAP             0xA0
#define SSD1306_SET_MUX_RATIO                 0xA8
#define SSD1306_SET_COM_OUTPUT_SCAN_DIR       0xC0
#define SSD1306_SET_DISPLAY_OFFSET            0xD3
#define SSD1306_SET_COM_PINS                  0xDA

// Timing and Driving Scheme Setting Commands
#define SSD1306_SET_DISPLAY_CLK_DIV           0xD5
#define SSD1306_SET_PRECHARGE_PERIOD          0xD9
#define SSD1306_SET_VCOM_DESELECT             0xDB

// Charge Pump Setting Commands
#define SSD1306_SET_CHARGE_PUMP               0x8D

// Scroll Commands
#define SSD1306_SCROLL_HORIZONTAL_RIGHT       0x26
#define SSD1306_SCROLL_HORIZONTAL_LEFT        0x27
#define SSD1306_SCROLL_VERTICAL_AND_RIGHT     0x29
#define SSD1306_SCROLL_VERTICAL_AND_LEFT      0x2A
#define SSD1306_DEACTIVATE_SCROLL             0x2E
#define SSD1306_ACTIVATE_SCROLL               0x2F
#define SSD1306_SET_VERTICAL_SCROLL_AREA      0xA3

// Additional Commands
#define SSD1306_NOP                           0xE3

/* Structs */
typedef struct {
	// I2C handle
	I2C_HandleTypeDef * i2c_handle;

	// Reset pin
	GPIO_TypeDef * reset_port;
	uint16_t reset_pin;

	// Copy of display memory
	uint8_t display_data[SSD1306_PAGES][SSD1306_WIDTH];
} SSD1306Display;

/* Functions */
HAL_StatusTypeDef ssd1306Init(SSD1306Display * ssd1306);
HAL_StatusTypeDef ssd1306SetContrast(SSD1306Display * ssd1306, uint8_t contrast);
HAL_StatusTypeDef ssd1306SendCmd(SSD1306Display * ssd1306, uint8_t reg, uint8_t data);
HAL_StatusTypeDef ssd1306SendCmds(SSD1306Display * ssd1306, uint8_t * data, uint8_t size);
HAL_StatusTypeDef ssd1306SendDatas(SSD1306Display * ssd1306, uint8_t * data, uint8_t size);
HAL_StatusTypeDef ssd1306UpdateDisplay(SSD1306Display * ssd1306);
HAL_StatusTypeDef ssd1306SendCmdByte(SSD1306Display * ssd1306, uint8_t data);

#endif /* INC_SSD1306_H_ */
