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


void process_rms(uint32_t *buf, uint32_t len);
float find_peak_current(uint32_t *buf, uint32_t len);


#endif /* INC_MEASURMENT_H_ */
