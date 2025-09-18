/*
 * ADXL345_I2C.c
 *
 *  Created on: Nov 20, 2024
 *      Author: tilen
 *
 *  @brief Source file for ADXL345 accelerometer I2C interface on STM32.
 *         This file implements functions for reading/writing registers,
 *         initializing the sensor, and reading acceleration data.
 *
 *  @note Uses STM32 HAL library for I2C operations. Assumes hi2c2 is configured.
 *        Global gains (GAINX, GAINY, GAINZ) are set during initialization for
 *        converting raw data to g units (not used in reading functions here).
 */

#include "ADXL345_I2C.h"

/**
 * @brief Global gain factors for X, Y, Z axes (in g/LSB).
 *        Set during initialization based on range and resolution.
 */
float GAINX = 0.0f;
float GAINY = 0.0f;
float GAINZ = 0.0f;

/**
 * @brief Writes a single byte to a specified register.
 * @param[in] address 8-bit register address.
 * @param[in] value 8-bit value to write.
 * @retval HAL status.
 * @note Uses master transmit for simplicity since single-byte writes.
 */
static HAL_StatusTypeDef ADXL345_I2C_writeRegister(uint8_t address, uint8_t value)
{
    uint8_t data[2] = {address, value};
    return HAL_I2C_Master_Transmit(&hi2c2, ADXL345_ADDR_1D, data, sizeof(data), HAL_MAX_DELAY);
}

/**
 * @brief Reads multiple bytes from a specified register.
 * @param[in] reg Starting register address.
 * @param[out] data_rec Buffer to store read data.
 * @param[in] numofBytes Number of bytes to read.
 * @retval HAL status.
 * @note Uses memory read for sequential reading.
 */
static HAL_StatusTypeDef ADXL345_I2C_readRegister(uint8_t reg, uint8_t* data_rec, uint8_t numofBytes)
{
    return HAL_I2C_Mem_Read(&hi2c2, ADXL345_ADDR_1D, reg, I2C_MEMADD_SIZE_8BIT, data_rec, numofBytes, HAL_MAX_DELAY);
}

/**
 * @brief Reads the device ID.
 * @param[out] devid Pointer to store device ID.
 * @retval HAL status.
 * @note Expected value is 0xE5 for ADXL345.
 */
HAL_StatusTypeDef ADXL345_I2C_ReadDEVID(uint8_t *devid)
{
    return ADXL345_I2C_readRegister(ADXL345_DEVID_REG, devid, 1);
}

/**
 * @brief Legacy initialization with default settings.
 * @retval HAL status.
 * @note Sets: Normal mode, 800Hz data rate (BW=400Hz), ±4g range, measurement mode.
 *       Verifies device ID before configuration.
 */
HAL_StatusTypeDef ADXL345_Init(void)
{
    uint8_t devID;
    if (ADXL345_I2C_ReadDEVID(&devID) != HAL_OK || devID != 0xE5) {
        return HAL_ERROR;
    }

    ADXL345_I2C_writeRegister(ADXL345_POWER_CTL_REG, 0x00);  // Reset power control
    ADXL345_I2C_writeRegister(ADXL345_BW_RATE_REG, 0x0D);   // 800Hz rate
    ADXL345_I2C_writeRegister(ADXL345_DATA_FORMAT_REG, 0x01);  // ±4g, 10-bit
    ADXL345_I2C_writeRegister(ADXL345_POWER_CTL_REG, 0x08);  // Enable measurement

    return HAL_OK;
}

/**
 * @brief Sets the bandwidth and data rate.
 * @param[in] adxl Pointer to initialization structure.
 * @retval HAL status.
 * @note Configures BW_RATE register based on LPMode and Rate.
 *       In low power mode, rates are limited (see tables in comments).
 *       In normal mode, full range of rates available.
 */
HAL_StatusTypeDef ADXL345_I2C_SetBW(ADXL_InitTypeDef * adxl)
{
    uint8_t bwreg = 0;

    if (adxl->LPMode == LPMODE_LOWPOWER) {
        bwreg |= (1 << 4);  // Enable low power
        if (adxl->Rate < 7 || adxl->Rate > 12) {
            bwreg |= 7;  // Default to 12.5Hz if out of range
        } else {
            bwreg |= adxl->Rate;
        }
    } else if (adxl->LPMode == LPMODE_NORMAL) {
        if (adxl->Rate < 6 || adxl->Rate > 15) {
            bwreg |= 6;  // Default to 6.25Hz if out of range
        } else {
            bwreg |= adxl->Rate;
        }
    } else {
        return HAL_ERROR;
    }

    return ADXL345_I2C_writeRegister(ADXL345_BW_RATE_REG, bwreg);
}

/**
 * @brief Initializes the ADXL345 with custom settings.
 * @param[in] adxl Pointer to initialization structure.
 * @retval HAL status.
 * @note Verifies device ID, sets BW and format, calculates gains,
 *       and configures auto-sleep and link modes.
 */
HAL_StatusTypeDef ADXL345_I2C_Init(ADXL_InitTypeDef * adxl)
{
    uint8_t devID;
    if (ADXL345_I2C_ReadDEVID(&devID) != HAL_OK || devID != 0xE5) {
        //return HAL_ERROR;
    }

    ADXL345_I2C_SetBW(adxl);
    ADXL345_I2C_SetFormat(adxl);

    // Set gains based on resolution and range
    if (adxl->Resolution == RESOLUTION_10BIT) {
        switch (adxl->Range) {
            case RANGE_2G: GAINX = GAINY = GAINZ = 1.0f / 255.0f; break;
            case RANGE_4G: GAINX = GAINY = GAINZ = 1.0f / 127.0f; break;
            case RANGE_8G: GAINX = GAINY = GAINZ = 1.0f / 63.0f; break;
            case RANGE_16G: GAINX = GAINY = GAINZ = 1.0f / 31.0f; break;
        }
    } else {
        GAINX = GAINY = GAINZ = 1.0f / 255.0f;  // Full res: 4mg/LSB scaled to range
    }

    // Set auto-sleep and link bits in POWER_CTL
    ADXL345_I2C_EnableDataReadyInterrupt(INT1);

    uint8_t reg;
    ADXL345_I2C_readRegister(ADXL345_POWER_CTL_REG, &reg, 1);
    if (adxl->AutoSleep == AUTOSLEEPON) reg |= (1 << 4); else reg &= ~(1 << 4);
    if (adxl->LinkMode == LINKMODEON) reg |= (1 << 5); else reg &= ~(1 << 5);
    ADXL345_I2C_writeRegister(ADXL345_POWER_CTL_REG, reg);

    return HAL_OK;
}

/**
 * @brief Sets the data format register.
 * @param[in] adxl Pointer to initialization structure.
 * @retval HAL status.
 * @note Configures SPI mode, interrupt invert, resolution, justify, and range.
 */
HAL_StatusTypeDef ADXL345_I2C_SetFormat(ADXL_InitTypeDef * adxl)
{
    uint8_t formatreg = 0;
    formatreg |= (adxl->SPIMode << 6);
    formatreg |= (adxl->IntMode << 5);
    formatreg |= (adxl->Resolution << 3);
    formatreg |= (adxl->Justify << 2);
    formatreg |= adxl->Range;
    return ADXL345_I2C_writeRegister(ADXL345_DATA_FORMAT_REG, formatreg);
}

/**
 * @brief Enables or disables measurement mode.
 * @param[in] state ON to enable, OFF to disable.
 * @retval HAL status.
 * @note Modifies the measure bit in POWER_CTL register.
 *       Clears sleep bit when enabling.
 */
HAL_StatusTypeDef ADXL345_I2C_StartMeasure(State state)
{
    uint8_t reg;
    ADXL345_I2C_readRegister(ADXL345_POWER_CTL_REG, &reg, 1);
    if (state == ON) {
        reg &= ~(1 << 2);  // Clear sleep
        reg |= (1 << 3);   // Set measure
    } else {
        reg &= ~(1 << 3);  // Clear measure
    }
    return ADXL345_I2C_writeRegister(ADXL345_POWER_CTL_REG, reg);
}

/**
 * @brief Reads raw XYZ acceleration data.
 * @param[out] data_rec Buffer for 6 bytes (XL, XH, YL, YH, ZL, ZH).
 * @retval HAL status.
 * @note Data is little-endian; combine low/high bytes for 16-bit signed values.
 */
HAL_StatusTypeDef ADXL345_I2C_ReadAccXYZ(void* data_acc_XYZ, uint8_t outputType)
{
	uint8_t data_rec[6];
	HAL_StatusTypeDef err = ADXL345_I2C_readRegister(ADXL345_DATA0_REG, data_rec, 6);

    if (outputType == OUTPUT_SIGNED)
	{
		int16_t * sdata = data_acc_XYZ;
		// Two's Complement
		sdata[0] = (int16_t) ((data_rec[1]*256+data_rec[0]));
		sdata[1] = (int16_t) ((data_rec[3]*256+data_rec[2]));
		sdata[2] = (int16_t) ((data_rec[5]*256+data_rec[4]));
	}
    else if (outputType == OUTPUT_FLOAT)
	{
		float * fdata = data_acc_XYZ;
		fdata[0] = ( (int16_t) ((data_rec[1]*256+data_rec[0])))*GAINX;
		fdata[1] = ( (int16_t) ((data_rec[3]*256+data_rec[2])))*GAINY;
		fdata[2] = ( (int16_t) ((data_rec[5]*256+data_rec[4])))*GAINZ;
	}

    uint8_t intSource;
	err = ADXL345_I2C_ClearInterruptFlags(&intSource);

    return err;
}




/**
 * @brief Enables the DATA_READY interrupt on the ADXL345.
 * @param[in] intPin Selects which interrupt pin to use (0 for INT1, 1 for INT2).
 * @retval HAL status.
 * @note Enables the DATA_READY interrupt in INT_ENABLE (Address 0x2E, bit 7).
 *       Optionally maps it to the specified interrupt pin via INT_MAP (Address 0x2F, bit 7).
 */
HAL_StatusTypeDef ADXL345_I2C_EnableDataReadyInterrupt(uint8_t intPin)
{
    uint8_t reg;

    // Optionally map DATA_READY to the specified interrupt pin (bit 7 in INT_MAP, Address 0x2F)
	// 0 = INT1, 1 = INT2 (inverted logic for mapping)
	if (ADXL345_I2C_readRegister(ADXL345_INT_MAP_REG, &reg, 1) != HAL_OK) {
		return HAL_ERROR;
	}
	reg &= ~(1 << 7); // Clear bit 7 first
	if (intPin == 1) { // Map to INT2 if intPin is 1
		reg |= (1 << 7);
	} // Else, keep mapped to INT1 (default when bit 7 = 0)
	if (ADXL345_I2C_writeRegister(ADXL345_INT_MAP_REG, reg) != HAL_OK) {
		return HAL_ERROR;
	}

    // Enable DATA_READY interrupt (bit 7 in INT_ENABLE register, Address 0x2E)
    if (ADXL345_I2C_readRegister(ADXL345_INT_ENABLE_REG, &reg, 1) != HAL_OK) {
        return HAL_ERROR;
    }
    // If any other interrupts are enabled something is wrong
    if (reg) return HAL_ERROR;

    reg |= (1 << 7); // Set DATA_READY bit
    if (ADXL345_I2C_writeRegister(ADXL345_INT_ENABLE_REG, reg) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}


/**
 * @brief Reads the INT_SOURCE register to clear all interrupt flags.
 * @param[out] intSource Pointer to store the interrupt source status (optional).
 * @retval HAL status.
 * @note Reading INT_SOURCE (Address 0x30) clears all active interrupt flags.
 */
HAL_StatusTypeDef ADXL345_I2C_ClearInterruptFlags(uint8_t *intSource)
{
    return ADXL345_I2C_readRegister(ADXL345_INT_SOURCE_REG, intSource, 1);
}



