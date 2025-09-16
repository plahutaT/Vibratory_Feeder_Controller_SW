/*
 * ADXL345_I2C.c
 *
 *  Created on: Aug 5, 2025
 *      Author: tilen
 */

#include <ADXL345_I2C.h>



float GAINX = 0.0f;
float GAINY = 0.0f;
float GAINZ = 0.0f;


/** Writing ADXL Registers.
* @address: 8-bit address of register
* @value  : 8-bit value of corresponding register
* Since the register values to be written are 8-bit, there is no need to multiple writing
*/
static void ADXL345_I2C_writeRegister(uint8_t address,uint8_t value)
{
	//if (address > 63)
	//address = 63;

	uint8_t data[2] = {address, value};

	//HAL_I2C_Mem_Read(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout)
	// Mem_Read() integrates writing a specific address for reading and than reading it, same goes for Mem_Write

	HAL_I2C_Master_Transmit(&hi2c2, ADXL345_ADDR_1D, data, sizeof(data), HAL_MAX_DELAY);



}


/** Reading ADXL Registers.
* @address: 8-bit address of register
* @retval value  : array of 8-bit values of corresponding register
* @num		: number of bytes to be written
*/

static void ADXL345_I2C_readRegister(uint8_t reg, uint8_t* data_rec, uint8_t numofBytes)
{

    HAL_I2C_Mem_Read(&hi2c2, ADXL345_ADDR_1D, reg, I2C_MEMADD_SIZE_8BIT, data_rec, numofBytes, HAL_MAX_DELAY);

}


/** Reading ADXL Registers.
* @address: 8-bit address of register
* @retval value  : array of 8-bit values of corresponding register
* @num		: number of bytes to be written
*/

HAL_StatusTypeDef ADXL345_I2C_ReadDEVID(uint8_t *devid)
{
  return HAL_I2C_Mem_Read(&hi2c2, ADXL345_ADDR_1D, ADXL345_DEVID_REG, I2C_MEMADD_SIZE_8BIT, devid, 1, HAL_MAX_DELAY);
}


/**
Bandwidth Settings:
 Setting BW_RATE register
 BWRATE[4] = LOW POWER SETTING
 BWRATE[0-3] = DATA RATE i.e. 0110 for 6.25 Hz // See Table 6,7
 @param LPMode = 0 // Normal mode, Default
							 = 1 // Low Power Mode
 @param BW : Badwidth; See Tables 6 and 7

								NORMAL MODE
				BW value 	|  Output Data Rate (Hz)
				---------------------------------
						6 		|  				6.25 // Default
						7 		|  				12.5
						8 		|  				25
						9 		|  				50
						10 		|  				100
						11 		|  				200
						12 		|  				400
						13 		|  				800
						14 		|  				1600
						15 		|  				3200


								LOWPOWER MODE
				BW value 	|  Output Data Rate (Hz)
				---------------------------------
						7 		|  				12.5	// Default
						8 		|  				25
						9 		|  				50
						10 		|  				100
						11 		|  				200
						12 		|  				400
*/


HAL_StatusTypeDef ADXL345_I2C_Init(void) {

	// Test if ADXL is responding

	uint8_t devID;
	ADXL345_I2C_ReadDEVID(&devID);
	if (devID != 0xe5) return HAL_ERROR;



	ADXL345_I2C_writeRegister(ADXL345_POWER_CTL_REG, 0x00); // reset all bits
	ADXL345_I2C_writeRegister(ADXL345_POWER_CTL_REG, 0x08); // set measure bit

	ADXL345_I2C_writeRegister(ADXL345_DATA_FORMAT_REG, 0x01); // set range +- 4g

	return HAL_OK;

}



HAL_StatusTypeDef ADXL345_I2C_ReadAccXYZ(uint8_t* data_rec) {

	// Test if ADXL is responding

	ADXL345_I2C_readRegister(ADXL345_DATA0_REG, data_rec, 6);

	return HAL_OK;

}






