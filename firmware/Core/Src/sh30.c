/*
 * sh30.c
 *
 *  Created on: Dec 10, 2023
 *      Author: Guim
 */

#include <sh30.h>

HAL_StatusTypeDef sh30Init(SH30 * sh30){
    uint8_t config_data[2] = {0x22, 0x36}; // Two reading per second

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(sh30->i2c_handle, SH30_ADDRESS, config_data, 2, SH30_I2C_TIEMOUT);
	HAL_Delay(100);
	return status;
}

HAL_StatusTypeDef sh30ReadData(SH30 * sh30) {
    uint8_t buffer[6];
	HAL_StatusTypeDef status;

    // Read data
    status = HAL_I2C_Master_Receive(sh30->i2c_handle, SH30_ADDRESS, buffer, 6, SH30_I2C_TIEMOUT);

    // Read temperature
    uint16_t raw = (buffer[0] << 8) | buffer[1];
    sh30->temperature = (float)raw / 65535.0 * 175.0 - 45.0;

    // Read humidity
    raw = (buffer[3] << 8) | buffer[4];
    sh30->humidity = (float)raw / 65535.0 * 100.0;

    return status;
}

