/*
 * spwm.h
 *
 *  Created on: Sep 10, 2025
 *      Author: tilen
 */

#ifndef INC_SPWM_H_
#define INC_SPWM_H_

#include <math.h>
#include <string.h>  // For string functions like strcmp
#include <stdint.h>
#include "main.h"


#define ARR 499
#define F_CARRIER 10000
#define SPWM_LUT_MAX_NS 500

// Function declerations
int SPWM_GenerateLUTs(float f_carrier, float f_out, uint16_t arr, float amplitude, uint32_t *p_ns);
int SPWM_GenerateLUTs_Inactive(float f_carrier, float f_out, uint16_t arr, float amplitude, uint32_t *p_ns);



#endif /* INC_SPWM_H_ */
