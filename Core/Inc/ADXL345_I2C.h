/*
 * bgt60ltr11_spi.h
 *
 *  Created on: Nov 20, 2024
 *      Author: tilen
 */

#ifndef INC_ADXL345_I2C_H_
#define INC_ADXL345_I2C_H_

#include "stdint.h"
#include "i2c.h"
#include "stm32h7xx_hal.h"


// Registers' Address
#define ADXL345_DEVID_REG 					0x0
#define ADXL345_BW_RATE_REG					0x2C
#define ADXL345_DATA_FORMAT_REG 			0x31
#define ADXL345_FIFO_CTL_REG 				0x38
#define ADXL345_DATA0_REG					0x32
#define ADXL345_POWER_CTL_REG 				0x2D
#define ADXL345_THRESH_TAP_REG				0x1D
#define ADXL345_DUR_REG						0x21
#define ADXL345_TAP_AXES_REG                0x2A
#define ADXL345_INT_ENABLE_REG				0x2E
#define ADXL345_INT_MAP_REG					0x2F
#define ADXL345_LATENT_REG					0x22
#define ADXL345_WINDOW_REG					0x23
#define ADXL345_THRESH_ACT_REG				0x24
#define ADXL345_THRESH_INACT_REG			0x25
#define ADXL345_TIME_INAT_REG				0x26
#define ADXL345_ACT_INACT_CTL_REG			0x27
#define ADXL345_THRESH_FF_REG 				0x28
#define ADXL345_TIME_FF_REG					0x29
#define ADXL345_OFFX_REG					0x1E
#define ADXL345_OFFY_REG					0x1F
#define ADXL345_OFFZ_REG					0x20
#define ADXL345_INT_SOURCE_REG				0x30



// Init. Definitions
#define SPIMODE_3WIRE 1
#define SPIMODE_4WIRE 0

#define LPMODE_NORMAL 0
#define LPMODE_LOWPOWER 1

#define BWRATE_6_25 	6
#define BWRATE_12_5 	7
#define BWRATE_25 		8
#define BWRATE_50 		9
#define BWRATE_100		10
#define BWRATE_200		11
#define BWRATE_400		12
#define BWRATE_800		13
#define BWRATE_1600   	14
#define BWRATE_3200   	15

#define BWRATE_12_5 	7
#define BWRATE_25 		8
#define BWRATE_50 		9
#define BWRATE_100		10
#define BWRATE_200		11
#define BWRATE_400		12

#define INT_ACTIVEHIGH 0
#define INT_ACTIVELOW  1

#define RESOLUTION_FULL  1
#define RESOLUTION_10BIT 0

#define JUSTIFY_MSB 	1
#define JUSTIFY_SIGNED  0

#define	SLEEP_RATE_1HZ 3
#define SLEEP_RATE_2HZ 2
#define SLEEP_RATE_4HZ 1
#define SLEEP_RATE_8HZ 0

#define RANGE_2G  0
#define RANGE_4G  1
#define RANGE_8G  2
#define RANGE_16G 3

#define AUTOSLEEPON  1
#define AUTOSLEEPOFF 0

#define LINKMODEON  1
#define LINKMODEOFF 0

// Init Type Def
typedef struct {
	uint8_t SPIMode;
	uint8_t IntMode;
	uint8_t LPMode;
	uint8_t Rate;
	uint8_t Range;
	uint8_t Resolution;
	uint8_t Justify;
	uint8_t AutoSleep;
	uint8_t LinkMode;
}ADXL_InitTypeDef;


// Private functions

/** Writing ADXL Registers.
* @address: 8-bit address of register
* @value  : 8-bit value of corresponding register
* Since the register values to be written are 8-bit, there is no need to multiple writing
*/
static void ADXL345_I2C_writeRegister(uint8_t address,uint8_t value);


/** Reading ADXL Registers.
* @address: 8-bit address of register
* @value  : array of 8-bit values of corresponding register
* @num		: number of bytes to be written
*/

static void ADXL345_I2C_readRegister(uint8_t reg, uint8_t* data_rec, uint8_t numofBytes);

HAL_StatusTypeDef ADXL345_I2C_ReadDEVID(uint8_t *devid);
HAL_StatusTypeDef ADXL345_I2C_Init(void);
HAL_StatusTypeDef ADXL345_I2C_ReadAccXYZ(uint8_t* data_rec);


/* ADXL 7b addres is 0x1D followed by R/nW bit so
 * 11101 0 -> Write
 * 11101 1 -> Read
 * */

// ADXL345 I2C addresses (8-bit, including R/W bit)
#define ADXL345_ADDR_1D        0x3A // SDO/ALT ADDRESS high
#define ADXL345_ADDR_WRITE_1D  0x3A // SDO/ALT ADDRESS high, write
#define ADXL345_ADDR_READ_1D   0x3B // SDO/ALT ADDRESS high, read

#define ADXL345_ADDR_53        0xA6 // SDO/ALT ADDRESS low
#define ADXL345_ADDR_WRITE_53  0xA6 // SDO/ALT ADDRESS low, write
#define ADXL345_ADDR_READ_53   0xA7 // SDO/ALT ADDRESS low, read

#define ADXL345_DEVID_REG      0x00 // DEVID register address
#define ADXL345_BW_RATE_REG    0x2C // Bandwidth rate register
#define ADXL345_POWER_CTL_REG  0x2D // Power control register


#define ADXL345_DEVID_REG 	  (0x00000000U)      // DEVID register address
//#define I2C_MEMADD_SIZE_8BIT  (0x00000001U)

#endif /* INC_ADXL345_I2C_H_ */
