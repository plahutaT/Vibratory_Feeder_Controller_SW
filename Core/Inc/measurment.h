/*
 * measurment.h
 *
 *  Created on: Sep 15, 2025
 *      Author: tilen
 */

#ifndef INC_MEASURMENT_H_
#define INC_MEASURMENT_H_

#include <math.h>
#include <string.h>  // For string functions like strcmp
#include <stdint.h>
#include "main.h"



static inline float adc_to_diff_voltage(uint32_t raw);
static inline float diff_voltage_to_current(float vdiff);
void process_rms(uint32_t *buf, uint32_t len);


#endif /* INC_MEASURMENT_H_ */
