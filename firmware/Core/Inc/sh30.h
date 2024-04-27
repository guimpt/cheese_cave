/*
 * sh30.h
 *
 *  Created on: Dec 10, 2023
 *      Author: Guim
 */

#ifndef INC_SH30_H_
#define INC_SH30_H_

/* Includes */
#include <main.h>

/* Defines */
#define SH30_ADDRESS	      (0x44 << 1)
#define SH30_I2C_TIEMOUT	  100

// Control Register
#define SH30_CTRL_REG          0x00

// Status Register
#define SH30_STATUS_REG        0x01

// Temperature Data Registers
#define SH30_TEMP_MSB_REG      0x02
#define SH30_TEMP_LSB_REG      0x03

// Humidity Data Registers
#define SH30_HUMIDITY_MSB_REG  0x04
#define SH30_HUMIDITY_LSB_REG  0x05

// Configuration Register
#define SH30_CONFIG_REG        0x06

// Device ID Register
#define SH30_DEVICE_ID_REG     0x0F
#define SH30_DEVICE_ID_VALUE   0x30

// Configuration Register Bits
#define SH30_CONFIG_RESOLUTION_MASK  (0x03 << 0)
#define SH30_CONFIG_RESOLUTION_14BIT  (0x00 << 0)
#define SH30_CONFIG_RESOLUTION_12BIT  (0x01 << 0)

/* Structs */
typedef struct {
	// I2C handle
	I2C_HandleTypeDef * i2c_handle;

	float temperature;
	float humidity;
} SH30;


/* Functions */
HAL_StatusTypeDef sh30Init(SH30 * sh30);
HAL_StatusTypeDef sh30ReadData(SH30 * sh30);


#endif /* INC_SH30_H_ */
