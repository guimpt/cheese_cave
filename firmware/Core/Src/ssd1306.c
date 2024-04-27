/*
 * ssd1306.c
 *
 *  Created on: Dec 9, 2023
 *      Author: Guim
 */

#include <ssd1306.h>

HAL_StatusTypeDef ssd1306Init(SSD1306Display * ssd1306){
	// Reset display
	HAL_GPIO_WritePin(ssd1306->reset_port, ssd1306->reset_pin, 0);
	HAL_Delay(10);
	HAL_GPIO_WritePin(ssd1306->reset_port, ssd1306->reset_pin, 1);
	HAL_Delay(100);

	HAL_StatusTypeDef status;

	// Set display off (0xAE)
	status = ssd1306SendCmdByte(ssd1306, SSD1306_DISPLAY_OFF);
	if (status != HAL_OK) return status;

	// Set memory address mode
	status = ssd1306SendCmd(ssd1306, SSD1306_SET_MEMORY_ADDR_MODE, 0x00); // Horizontal mode
	if (status != HAL_OK) return status;

	// Set clock divide ratio (0xD5, 0x80)
	status = ssd1306SendCmd(ssd1306, SSD1306_SET_DISPLAY_CLK_DIV, 0x80);
	if (status != HAL_OK) return status;

	// Set mux ratio (0xA8, 0x3F)
	status = ssd1306SendCmd(ssd1306, SSD1306_SET_MUX_RATIO, 0x3F);
	if (status != HAL_OK) return status;

	// Set display offset (0xD3, 0x00)
	status = ssd1306SendCmd(ssd1306, SSD1306_SET_DISPLAY_OFFSET, 0x00);
	if (status != HAL_OK) return status;

	// Set Display Start Line (0x40)
	status = ssd1306SendCmdByte(ssd1306, SSD1306_SET_DISPLAY_START_LINE | 0x00);
	if (status != HAL_OK) return status;

	// Set Segment Re-Map (0xA1)
	status = ssd1306SendCmdByte(ssd1306, SSD1306_SET_SEGMENT_REMAP | 0x01);
	if (status != HAL_OK) return status;

	// Set COM Output Scan Direction (0xC8)
	status = ssd1306SendCmdByte(ssd1306, SSD1306_SET_COM_OUTPUT_SCAN_DIR | 0x08);
	if (status != HAL_OK) return status;

	// Set COM Pins Hardware Configuration (0xDA, 0x12)
	status = ssd1306SendCmd(ssd1306, SSD1306_SET_COM_PINS, 0x12);
	if (status != HAL_OK) return status;

	// Set Contrast Control (0x81, 0xCF)
	status = ssd1306SendCmd(ssd1306, SSD1306_SET_CONTRAST_CONTROL, 0xCF);
	if (status != HAL_OK) return status;

	// Set Pre-Charge Period (0xD9, 0xF1)
	status = ssd1306SendCmd(ssd1306, SSD1306_SET_PRECHARGE_PERIOD, 0xF1);
	if (status != HAL_OK) return status;

	// Set VCOMH Deselect Level (0xDB, 0x30)
	status = ssd1306SendCmd(ssd1306, SSD1306_SET_VCOM_DESELECT, 0x30);
	if (status != HAL_OK) return status;

	// Set Entire Display On/Off (0xA4)
	status = ssd1306SendCmdByte(ssd1306, SSD1306_DISPLAY_ALL_ON_RESUME);
	if (status != HAL_OK) return status;

	// Set Normal/Inverse Display (0xA6)
	status = ssd1306SendCmdByte(ssd1306, SSD1306_NORMAL_DISPLAY);
	if (status != HAL_OK) return status;

	// Set Charge Pump (0x8D, 0x14)
	status = ssd1306SendCmd(ssd1306, SSD1306_SET_CHARGE_PUMP, 0x14);
	if (status != HAL_OK) return status;

	// Set Display On (0xAF)
	status = ssd1306SendCmdByte(ssd1306, SSD1306_DISPLAY_ON);
	if (status != HAL_OK) return status;


	// 100ms delay
	HAL_Delay(100);

	return HAL_OK;
}

HAL_StatusTypeDef ssd1306UpdateDisplay(SSD1306Display * ssd1306){

	// Set page address range (0x22)
	uint8_t buff[3];
	buff[0] = SSD1306_SET_PAGE_ADDR;
	buff[1] = 0;
	buff[2] = 7;

	ssd1306SendCmds(ssd1306, buff, 3);

	// Set column address range (0x21)
	buff[0] = SSD1306_SET_COLUMN_ADDR;
	buff[1] = 0;
	buff[2] = 127;

	ssd1306SendCmds(ssd1306, buff, 3);

	for (uint8_t p = 0; p < SSD1306_PAGES; p++){
		ssd1306SendDatas(ssd1306, ssd1306->display_data[p], SSD1306_WIDTH);
	}

	return HAL_OK;
}

HAL_StatusTypeDef ssd1306SetContrast(SSD1306Display * ssd1306, uint8_t contrast){
	return ssd1306SendCmd(ssd1306, SSD1306_SET_CONTRAST_CONTROL, contrast);
}


HAL_StatusTypeDef ssd1306SendCmdByte(SSD1306Display * ssd1306, uint8_t data){
	return HAL_I2C_Mem_Write(ssd1306->i2c_handle, SSD1306_ADDRESS, SSD1306_CONTROL_BYTE_CMD, I2C_MEMADD_SIZE_8BIT, &data, 1, SSD1306_I2C_TIEMOUT);
}

HAL_StatusTypeDef ssd1306SendCmd(SSD1306Display * ssd1306, uint8_t reg, uint8_t data){
	uint8_t cmd[2];
	cmd[0] = reg;
	cmd[1] = data;
	return HAL_I2C_Mem_Write(ssd1306->i2c_handle, SSD1306_ADDRESS, SSD1306_CONTROL_BYTE_CMD, I2C_MEMADD_SIZE_8BIT, cmd, 2, SSD1306_I2C_TIEMOUT);
}

HAL_StatusTypeDef ssd1306SendCmds(SSD1306Display * ssd1306, uint8_t * data, uint8_t size){
	return HAL_I2C_Mem_Write(ssd1306->i2c_handle, SSD1306_ADDRESS, SSD1306_CONTROL_BYTE_CMD, I2C_MEMADD_SIZE_8BIT, data, size, SSD1306_I2C_TIEMOUT);
}

HAL_StatusTypeDef ssd1306SendDatas(SSD1306Display * ssd1306, uint8_t * data, uint8_t size){
	return HAL_I2C_Mem_Write(ssd1306->i2c_handle, SSD1306_ADDRESS, SSD1306_CONTROL_BYTE_DATA, I2C_MEMADD_SIZE_8BIT, data, size, SSD1306_I2C_TIEMOUT);
}
