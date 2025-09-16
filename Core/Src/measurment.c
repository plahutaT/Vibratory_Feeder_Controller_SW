/*
 * measurment.c
 *
 *  Created on: Sep 15, 2025
 *      Author: tilen
 */

#include "measurment.h"


#define LOAD_CURRENT_BUFFER_LENGHT 256
#define VREF          3.3f  // reference voltage
#define ADC_RES       65535.0f
#define CURR_FS_V     2050.0f   // full-scale differential voltage ±2.05 V, ±2050 mV
#define CURR_FS_A     4000.0f    // corresponding current ±4 A, ±4000 mA



float current_rms = 0.0f;



uint32_t load_current_buff[LOAD_CURRENT_BUFFER_LENGHT] __attribute__((aligned(32)));


// --- Function to convert ADC raw to differential voltage (Volts)
static inline float adc_to_diff_voltage(uint32_t raw)
{
    return VREF * ((2.0f * (float)raw / ADC_RES) - 1.0f);
}

// --- Function to map voltage to current (Amps)
static inline float diff_voltage_to_current(float vdiff)
{
    return (vdiff / CURR_FS_V) * CURR_FS_A;
}

// --- Processing: RMS over N samples
void process_rms(uint32_t *buf, uint32_t len)
{
    double sum_sq = 0.0;

    for (uint16_t i = 0; i < len; i++) {
        float vdiff = adc_to_diff_voltage(buf[i]);
        float i_val = diff_voltage_to_current(vdiff);
        sum_sq += (double)(i_val * i_val);
    }

    float mean_sq = (float)(sum_sq / (double)len);
    current_rms = sqrtf(mean_sq);
}
