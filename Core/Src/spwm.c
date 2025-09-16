/*
 * spwm.c
 *
 *  Created on: Sep 10, 2025
 *      Author: tilen
 */

#include "spwm.h"

/* To change the output voltage we need to scale the values inside the LUT    */

/*
 *   For data coherency we need to disable the MPU in this region, so we make the
 *   region sharable in the MPU configuration. DCache is disabled for this region
 *
 *   The other option is to invalidate the memory and clean DCache manually
 *
 *	 uint16_t sineLookupTable[NS] @0x2000200000
 *	 Then we set an MPU region that starts 0x20020000 and is SHARABLE
 *
 *	 MPU_InitStruct.Enable = MPU_REGION_ENABLE;
	 MPU_InitStruct.Number = MPU_REGION_NUMBER0;
	 MPU_InitStruct.BaseAddress = 0x2000200000;
	 MPU_InitStruct.Size = MPU_REGION_SIZE_1kB;
	 MPU_InitStruct.SubRegionDisable = 0x0;
	 MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
	 MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
	 MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
	 MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
	 MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
	 MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
 *
 *
 * */

// These need to be in Sharable and NonCachable region

/*
 * LUT Buffers for DMA
 * - sineLookupTable_dynamic1/2: Two 128-element buffers for double-buffering.
 * - Placed in non-cacheable, shareable MPU region (0x20020000) to ensure DMA
 *   data coherency, with manual D-cache cleaning for robustness.
 * - pdma_active_buf: Points to the buffer currently used by DMA.
 * - pdma_inactive_buf: Points to the buffer updated by CPU (e.g., in HandleChangeAmp).
 * - dma_buf_swap_f: Flag to request buffer swap on next DMA completion.
 */


volatile uint32_t g_ns_active;
volatile uint32_t g_ns_inactive;

uint16_t spwm_lut_a[SPWM_LUT_MAX_NS] __attribute__((aligned(32)));
uint16_t spwm_lut_b[SPWM_LUT_MAX_NS] __attribute__((aligned(32)));
uint16_t spwm_lut_inv_a[SPWM_LUT_MAX_NS] __attribute__((aligned(32)));
uint16_t spwm_lut_inv_b[SPWM_LUT_MAX_NS] __attribute__((aligned(32)));

uint16_t *pspwm_active      = spwm_lut_a;
uint16_t *pspwm_active_inv  = spwm_lut_inv_a;
uint16_t *pspwm_inactive    = spwm_lut_b;
uint16_t *pspwm_inactive_inv= spwm_lut_inv_b;

volatile uint8_t spwm_swap_request_f = 0;
volatile uint8_t spwm_start_request_f = 0;

/* Generates sine + complementary tables.
 * amplitude: 0.0–1.0 (use <1 to avoid rails, e.g. 0.98)
 * Returns 0 on success, -1 on error.
 */
int SPWM_GenerateLUTs(float f_carrier, float f_out, uint16_t arr, float amplitude, uint32_t *p_ns)
{
    if (!p_ns || f_carrier <= 0.f || f_out <= 0.f || amplitude < 0.f) return -1;

    uint32_t NS = (uint32_t)(f_carrier / f_out);   /* floor */
    if (NS == 0u || NS > SPWM_LUT_MAX_NS) return -1;
    if (amplitude > 1.f) amplitude = 1.f;

    float center = (float)arr * 0.5f;
    float scale  = center * amplitude;

    for (uint32_t i = 0; i < NS; ++i) {
        float phase = (2.f * M_PI * (float)i) / (float)NS;
        float s = sinf(phase);                   /* -1 .. +1 */
        float fval = center + scale * s;         /* center +/- scale */
        int32_t v = (int32_t)(fval + 0.5f);      /* round */
        if (v < 0) v = 0;
        if (v > (int32_t)arr) v = (int32_t)arr;
        pspwm_active[i]     = (uint16_t)v;
        pspwm_active_inv[i] = (uint16_t)(arr - v);
    }

    *p_ns = NS;

    /* If DCache enabled and region cacheable: */
    SCB_CleanDCache_by_Addr((uint32_t*)pspwm_active,     ((NS * sizeof(uint16_t) + 31) & ~31));
    SCB_CleanDCache_by_Addr((uint32_t*)pspwm_active_inv, ((NS * sizeof(uint16_t) + 31) & ~31));

    return 0;

}

/* Generate into inactive buffers */
int SPWM_GenerateLUTs_Inactive(float f_carrier, float f_out, uint16_t arr, float amplitude, uint32_t *p_ns)
{
	if (!p_ns || f_carrier <= 0.f || f_out <= 0.f || amplitude < 0.f) return -1;

    uint32_t NS = (uint32_t)(f_carrier / f_out);

    if (NS == 0u || NS > SPWM_LUT_MAX_NS) return -1;
    if (amplitude < 0.f) amplitude = 0.f;
    if (amplitude > 1.f) amplitude = 1.f;

    float center = (float)arr * 0.5f;
    float scale  = center * amplitude;

    for (uint32_t i = 0; i < NS; ++i) {
        float phase = (2.f * M_PI * (float)i) / (float)NS;
        float s = sinf(phase);
        float fval = center + scale * s;
        int32_t v = (int32_t)(fval + 0.5f);
        if (v < 0) v = 0;
        if (v > (int32_t)arr) v = (int32_t)arr;
        pspwm_inactive[i]      = (uint16_t)v;
        pspwm_inactive_inv[i]  = (uint16_t)(arr - v);
    }


    *p_ns = NS;


    /* Clean DCache for DMA visibility if needed */
    SCB_CleanDCache_by_Addr((uint32_t*)pspwm_inactive,     ((NS * sizeof(uint16_t) + 31) & ~31));
    SCB_CleanDCache_by_Addr((uint32_t*)pspwm_inactive_inv, ((NS * sizeof(uint16_t) + 31) & ~31));

    return 0;
}
