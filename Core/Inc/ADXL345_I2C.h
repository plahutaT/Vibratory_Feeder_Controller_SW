/*
 * ADXL345_I2C.h
 *
 *  Created on: Nov 20, 2024
 *      Author: tilen
 *
 *  @brief Header file for ADXL345 accelerometer I2C interface on STM32.
 *         This file defines constants, structures, enumerations, and function prototypes
 *         for initializing and interacting with the ADXL345 sensor via I2C.
 *         The ADXL345 is a 3-axis accelerometer with digital output.
 *
 *  @note This implementation assumes the use of STM32 HAL library for I2C communication.
 *        The sensor can operate in either I2C or SPI mode, but this file focuses on I2C.
 *        Some defines (e.g., SPIMODE) are included for compatibility with the DATA_FORMAT register.
 */

#ifndef INC_ADXL345_I2C_H_
#define INC_ADXL345_I2C_H_

#include "stdint.h"
#include "i2c.h"
#include "stm32h7xx_hal.h"

/**
 * @defgroup ADXL345_Addresses I2C Addresses
 * @{
 */

/** @brief ADXL345 I2C address when ALT ADDRESS pin is high (0x1D << 1). */
#define ADXL345_ADDR_1D        0x3A

/** @brief ADXL345 write address when ALT ADDRESS pin is high. */
#define ADXL345_ADDR_WRITE_1D  0x3A

/** @brief ADXL345 read address when ALT ADDRESS pin is high. */
#define ADXL345_ADDR_READ_1D   0x3B

/** @brief ADXL345 I2C address when ALT ADDRESS pin is low (0x53 << 1). */
#define ADXL345_ADDR_53        0xA6

/** @brief ADXL345 write address when ALT ADDRESS pin is low. */
#define ADXL345_ADDR_WRITE_53  0xA6

/** @brief ADXL345 read address when ALT ADDRESS pin is low. */
#define ADXL345_ADDR_READ_53   0xA7

/** @} */  // End of ADXL345_Addresses

/**
 * @defgroup ADXL345_Registers Register Addresses
 * @{
 */

/** @brief Device ID register (read-only, should return 0xE5). */
#define ADXL345_DEVID_REG 					0x00

/** @brief Bandwidth rate register for setting output data rate and power mode. */
#define ADXL345_BW_RATE_REG					0x2C

/** @brief Data format register for range, resolution, and other format settings. */
#define ADXL345_DATA_FORMAT_REG 			0x31

/** @brief FIFO control register. */
#define ADXL345_FIFO_CTL_REG 				0x38

/** @brief Starting address for acceleration data (X0). */
#define ADXL345_DATA0_REG					0x32

/** @brief Power control register for measurement mode, sleep, etc. */
#define ADXL345_POWER_CTL_REG 				0x2D

/** @brief Tap threshold register. */
#define ADXL345_THRESH_TAP_REG				0x1D

/** @brief Tap duration register. */
#define ADXL345_DUR_REG						0x21

/** @brief Tap axes control register. */
#define ADXL345_TAP_AXES_REG                0x2A

/** @brief Interrupt enable register. */
#define ADXL345_INT_ENABLE_REG				0x2E

/** @brief Interrupt mapping register. */
#define ADXL345_INT_MAP_REG					0x2F

/** @brief Double-tap latency register. */
#define ADXL345_LATENT_REG					0x22

/** @brief Double-tap window register. */
#define ADXL345_WINDOW_REG					0x23

/** @brief Activity threshold register. */
#define ADXL345_THRESH_ACT_REG				0x24

/** @brief Inactivity threshold register. */
#define ADXL345_THRESH_INACT_REG			0x25

/** @brief Inactivity time register. */
#define ADXL345_TIME_INAT_REG				0x26

/** @brief Activity/inactivity control register. */
#define ADXL345_ACT_INACT_CTL_REG			0x27

/** @brief Free-fall threshold register. */
#define ADXL345_THRESH_FF_REG 				0x28

/** @brief Free-fall time register. */
#define ADXL345_TIME_FF_REG					0x29

/** @brief X-axis offset register. */
#define ADXL345_OFFX_REG					0x1E

/** @brief Y-axis offset register. */
#define ADXL345_OFFY_REG					0x1F

/** @brief Z-axis offset register. */
#define ADXL345_OFFZ_REG					0x20

/** @brief Interrupt source register (read-only). */
#define ADXL345_INT_SOURCE_REG				0x30

/** @} */  // End of ADXL345_Registers

/**
 * @defgroup ADXL345_Init_Definitions Initialization Definitions
 * @{
 */

/** @brief SPI mode: 3-wire. */
#define SPIMODE_3WIRE 1

/** @brief SPI mode: 4-wire (default for I2C, but used in DATA_FORMAT). */
#define SPIMODE_4WIRE 0

/** @brief Low power mode: Normal operation. */
#define LPMODE_NORMAL 0

/** @brief Low power mode: Low power. */
#define LPMODE_LOWPOWER 1

/** @brief Bandwidth rates for normal mode. */
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

/** @brief Interrupt polarity: Active high. */
#define INT_ACTIVEHIGH 0

/** @brief Interrupt polarity: Active low. */
#define INT_ACTIVELOW  1

/** @brief Resolution: Full resolution (up to 13-bit). */
#define RESOLUTION_FULL  1

/** @brief Resolution: 10-bit fixed. */
#define RESOLUTION_10BIT 0

/** @brief Justification: MSB (left-justified). */
#define JUSTIFY_MSB 	1

/** @brief Justification: Signed (right-justified with sign extension). */
#define JUSTIFY_SIGNED  0

/** @brief Sleep mode rates. */
#define	SLEEP_RATE_1HZ 3
#define SLEEP_RATE_2HZ 2
#define SLEEP_RATE_4HZ 1
#define SLEEP_RATE_8HZ 0

/** @brief Measurement ranges. */
#define RANGE_2G  0
#define RANGE_4G  1
#define RANGE_8G  2
#define RANGE_16G 3

/** @brief Auto-sleep: Enabled. */
#define AUTOSLEEPON  1

/** @brief Auto-sleep: Disabled. */
#define AUTOSLEEPOFF 0

/** @brief Link mode: Enabled (links activity and inactivity detection). */
#define LINKMODEON  1

/** @brief Link mode: Disabled. */
#define LINKMODEOFF 0

/** @brief Output format: Float. */
#define OUTPUT_FLOAT  0

/** @brief Output format: INT16. */
#define OUTPUT_SIGNED 1



/** @} */  // End of ADXL345_Init_Definitions

/**
 * @brief Structure for ADXL345 initialization parameters.
 */
typedef struct {
    /** @brief SPI mode (3-wire or 4-wire, affects DATA_FORMAT). */
	uint8_t SPIMode;
    /** @brief Interrupt polarity mode. */
	uint8_t IntMode;
    /** @brief Low power mode. */
	uint8_t LPMode;
    /** @brief Output data rate (bandwidth). */
	uint8_t Rate;
    /** @brief Measurement range. */
	uint8_t Range;
    /** @brief Resolution mode. */
	uint8_t Resolution;
    /** @brief Data justification. */
	uint8_t Justify;
    /** @brief Auto-sleep enable. */
	uint8_t AutoSleep;
    /** @brief Link mode enable. */
	uint8_t LinkMode;
} ADXL_InitTypeDef;

/**
 * @brief Enumeration for state (ON/OFF).
 */
typedef enum {
	ON,   /**< State ON. */
	OFF   /**< State OFF. */
} State;

typedef enum {
	INT1,   /**< INT1 Pin of ADXL345. */
	INT2    /**< INT2 Pin of ADXL345. */
} IntPins;

/**
 * @brief Reads the device ID from the ADXL345.
 * @param[out] devid Pointer to store the device ID (should be 0xE5).
 * @retval HAL status.
 */
HAL_StatusTypeDef ADXL345_I2C_ReadDEVID(uint8_t *devid);

/**
 * @brief Initializes the ADXL345 with provided configuration.
 * @param[in] adxl Pointer to initialization structure.
 * @retval HAL status.
 */
HAL_StatusTypeDef ADXL345_I2C_Init(ADXL_InitTypeDef * adxl);

/**
 * @brief Reads raw acceleration data for X, Y, Z axes.
 * @param[out] data_rec Buffer to store 6 bytes of data (X low/high, Y low/high, Z low/high).
 * @retval HAL status.
 */
HAL_StatusTypeDef ADXL345_I2C_ReadAccXYZ(void* data_acc_XYZ, uint8_t outputType);

/**
 * @brief Sets the data format register based on initialization parameters.
 * @param[in] adxl Pointer to initialization structure.
 * @retval HAL status.
 */
HAL_StatusTypeDef ADXL345_I2C_SetFormat(ADXL_InitTypeDef * adxl);

/**
 * @brief Sets the bandwidth rate register based on initialization parameters.
 * @param[in] adxl Pointer to initialization structure.
 * @retval HAL status.
 */
HAL_StatusTypeDef ADXL345_I2C_SetBW(ADXL_InitTypeDef * adxl);

/**
 * @brief Starts or stops measurement mode.
 * @param[in] state ON to start, OFF to stop.
 * @retval HAL status.
 */
HAL_StatusTypeDef ADXL345_I2C_StartMeasure(State state);

/**
 * @brief Enables DataReady interrupt and maps it to INT1 or INT2 pin of ADXL345.
 * @param[intPin] interrupt pin to map the DataReady interrupt INT1 or INT2.
 * @retval HAL status.
 */
HAL_StatusTypeDef ADXL345_I2C_EnableDataReadyInterrupt(uint8_t intPin);

HAL_StatusTypeDef ADXL345_I2C_ClearInterruptFlags(uint8_t *intSource);

/**
 * @brief Legacy initialization function with default settings.
 * @retval HAL status.
 * @note This function uses hardcoded defaults: 800Hz rate, ±4g range.
 */
HAL_StatusTypeDef ADXL345_Init(void);

#endif /* INC_ADXL345_I2C_H_ */
