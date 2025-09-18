/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM7
#include "arm_math.h"
#include "usbd_cdc_if.h"
#include "string.h"
#include "fft.h"
#include "ADXL345_I2C.h"
#include "cli.h"
#include "spwm.h"
#include "measurment.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

#define NS 200               // Number of samples in sine LUT (128-point sine wave)
#define F_OUT 50             // Default output sine frequency (50 Hz)
#define LOAD_CURRENT_BUFFER_LENGHT 256

extern TIM_HandleTypeDef htim1;  // TIM1 handle for 20 kHz PWM generation
extern TIM_HandleTypeDef htim3;  // TIM3 handle for triggering DMA LUT updates
extern TIM_HandleTypeDef htim7;  // TIM7 handle for periodic tasks (currently disabled)
extern DMA_HandleTypeDef hdma_tim3_ch1;  // DMA handle for TIM3 CH1 to TIM1->CCR1 transfers

extern volatile uint32_t g_ns_active;
extern volatile uint32_t g_ns_inactive;

extern uint16_t *pspwm_active;
extern uint16_t *pspwm_active_inv;
extern uint16_t *pspwm_inactive;
extern uint16_t *pspwm_inactive_inv;

extern volatile uint8_t spwm_swap_request_f;
extern volatile uint8_t spwm_start_request_f;
volatile uint8_t adc1_sampl_ready_f = 0;
volatile uint8_t adc2_sampl_ready_f = 0;


volatile uint8_t adxl345_data_ready_f = 0;


// USB command buffer for CLI input (e.g., "CHANGE_FREQ 50")
uint8_t cmd_usb_buffer[64];
uint8_t cmd_length;


//uint16_t IFI_sample[FFT_BUFFER_SIZE];
//uint16_t IFQ_sample[FFT_BUFFER_SIZE];
//
//extern float32_t sampled_data[2 * FFT_BUFFER_SIZE];
//// Define two buffers for double buffering
//float32_t buffer1[2 * FFT_BUFFER_SIZE];
//float32_t buffer2[2 * FFT_BUFFER_SIZE];
//
//// Pointers to active (writing) and processing buffers
//float32_t *active_buffer = buffer1;
//float32_t *processing_buffer = buffer2;
//
//uint16_t acquired_sample_count = 0;
//uint16_t fft_buff_count = 0;
//
//// Generate a test sine wave
//float32_t max_value = 0.0f;
//float32_t peak_index = 0.0f;
//float32_t target_velocity = 0.0f;

extern uint32_t load_current_buff[LOAD_CURRENT_BUFFER_LENGHT];
extern float current_rms;


volatile uint8_t usb_print_f = 0;

uint32_t error_cnt = 0;
uint8_t data_ready_f = 0;

uint32_t i_adc_value = 0;
int32_t i_voltage_value = 0;
float load_current_value = 0.0f;

uint32_t v_adc_value = 0;
int32_t v_voltage_value = 0;
float dc_link_voltage_value = 0.0f;

uint8_t devID = 0;

// Buffer to store acceleration data
volatile int16_t accelData[3]; // X, Y, Z axes
volatile float faccelData[3];



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM7_Init();
  MX_TIM6_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
	//  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
	//  bgt60ltr11_HW_reset();

	/* ADXL345 Initialization Structure */
	ADXL_InitTypeDef adxlConfig = {
		.SPIMode = SPIMODE_4WIRE,   // Default for I2C compatibility
		.IntMode = INT_ACTIVEHIGH,  // Active high interrupt
		.LPMode = LPMODE_NORMAL,    // Normal power mode
		.Rate = BWRATE_800,         // 800 Hz data rate
		.Range = RANGE_4G,          // ±4g range
		.Resolution = RESOLUTION_10BIT, // 10-bit resolution
		.Justify = JUSTIFY_SIGNED,  // Signed justification
		.AutoSleep = AUTOSLEEPOFF,  // Disable auto-sleep
		.LinkMode = LINKMODEOFF     // Disable link mode
	};

	HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_DIFFERENTIAL_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET_LINEARITY, ADC_DIFFERENTIAL_ENDED);


	HAL_TIM_Base_Start(&htim6);
	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)load_current_buff, LOAD_CURRENT_BUFFER_LENGHT) != HAL_OK) {
	  error_cnt++;
	}

	HAL_TIM_Base_Start(&htim15);
	HAL_ADC_Start_IT(&hadc2);


	// Initialize LUT buffers with original sine values
	SPWM_GenerateLUTs(10000.0f, 50.0f, 499, 0.5, &g_ns_active);

	HAL_Delay(500);

	HAL_TIM_Base_Start_IT(&htim7);

	/* Initialize ADXL345 */
	if (ADXL345_I2C_Init(&adxlConfig) != HAL_OK)
	{
	  error_cnt++; // Handle initialization failure
	}
	/* Start measurement mode */
	if (ADXL345_I2C_StartMeasure(ON) != HAL_OK)
	{
		error_cnt++; // Handle measurement start failure
	}

	HAL_Delay(500);

	data_ready_f = 0;

	//HAL_TIM_Base_Start(&htim1);
	//HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	//HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	//HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	//HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);


	/* TODO check out DMA burst mode
	* DMA Burst on Update (hardware “both at once”)

	Enable preload for OC1/OC2.
	Configure TIM1 DCR to burst-write CCR1 then CCR2 on UDE (update DMA request).
	One DMA stream with interleaved buffer: [d1_0, d2_0, d1_1, d2_1, ...].
	Update event latches both simultaneously.
	*
	* */
	char response[100];

	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t*)pspwm_active, (uint16_t)g_ns_active);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);


	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_2, (uint32_t*)pspwm_active_inv, (uint16_t)g_ns_active);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_Delay(100);

	HAL_GPIO_WritePin(PE3_GPIO_Port, PE3_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		if (spwm_start_request_f)
		{

			/* Swap active / inactive pointers */
			uint16_t *t;
			t = pspwm_active;         pspwm_active        = pspwm_inactive;        pspwm_inactive        = t;
			t = pspwm_active_inv;     pspwm_active_inv    = pspwm_inactive_inv;    pspwm_inactive_inv    = t;

			uint32_t t_ns;
			t_ns = g_ns_active; g_ns_active = g_ns_inactive; g_ns_inactive = t_ns;


			SCB_CleanDCache_by_Addr((uint32_t*)pspwm_active,     ((g_ns_active*sizeof(uint16_t)+31U)&~31U));
			SCB_CleanDCache_by_Addr((uint32_t*)pspwm_active_inv, ((g_ns_active*sizeof(uint16_t)+31U)&~31U));

			/* Restart DMA on both channels with new buffers */
			HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t*)pspwm_active,     (uint16_t)g_ns_active);
			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_2, (uint32_t*)pspwm_active_inv, (uint16_t)g_ns_active);
			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

			spwm_start_request_f = 0;
		}
		else if (adc1_sampl_ready_f)
		{
			//i_adc_value = HAL_ADC_GetValue(&hadc1);

			// Conv value = (2^16-1 / 2) * (1 + ((Vinp - Vinn) / Vref))
			// Here we get the voltage value in mV
			//i_voltage_value = (3300*2*i_adc_value/65535) - 3300;
			// Map voltage to current (A), ±2.05 V = ±4 A
			//load_current_value = (i_voltage_value / (1000 * 2.05f)) * 4000.0f;

			adc1_sampl_ready_f = 0;
		}
		else if (adc2_sampl_ready_f)
		{
			v_adc_value = HAL_ADC_GetValue(&hadc2);

			// Conv value = (2^16-1 / 2) * (1 + ((Vinp - Vinn) / Vref))
			// Here we get the voltage value in mV
			v_voltage_value = (3300*2*v_adc_value/65535) - 3300;
			// Map voltage to current (A), ±2.05 V = ±4 A
			// I got 19000 for 19V do i need to devide the value by 1000????

			// DC LINK voltage in V
			dc_link_voltage_value = (v_voltage_value / (1000 * 2.0f)) * 400.0f;

			adc2_sampl_ready_f = 0;
		}
		else if (usb_print_f)
		{

			int32_t amps    = (int32_t)current_rms;                        // integer part
			int32_t deci_a   = (int32_t)(abs((current_rms - amps) * 10));       // first decimal digit

			int32_t volts    = (int32_t)dc_link_voltage_value;                        // integer part
			int32_t deci_v   = (int32_t)(abs((dc_link_voltage_value - volts) * 10));       // first decimal digit

			sprintf(response, "LC_RMS = %ld.%01ld mA, DCLINK = %ld.%01ld V \r\n", (long)amps, (long)deci_a, (long)volts, (long)deci_v);
			CDC_Transmit_FS((uint8_t*)response, strlen(response));

			SCB_InvalidateDCache_by_Addr((uint32_t*)load_current_buff, (LOAD_CURRENT_BUFFER_LENGHT * sizeof(uint32_t)+31U)&~31U);
			float peak_curr = find_peak_current((uint32_t*)load_current_buff, LOAD_CURRENT_BUFFER_LENGHT);
			int32_t peak_amp = (int32_t)peak_curr;
			int32_t deci_peak_amp = (int32_t)(abs((peak_curr - peak_amp) * 10));
			sprintf(response, "LC_PEAK = %ld.%01ld mA\r\n", (long)peak_amp, (long)deci_peak_amp);
			CDC_Transmit_FS((uint8_t*)response, strlen(response));

			usb_print_f = 0;
		}
		// Check if data has been received over USB VCP
		else if (cmd_length > 0)
		{
			cli_processCommand(cmd_usb_buffer);  // Process the command
			cmd_length = 0;  // Reset the buffer after processing
		}

		else if (adxl345_data_ready_f)
		{

			ADXL345_I2C_ReadAccXYZ(faccelData, OUTPUT_FLOAT);
			adxl345_data_ready_f = 0;

		}


	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 48;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 4;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 16;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


/**
  * @brief  EXTI Line Detection Callback
  * @param  GPIO_Pin Specifies the port pin connected to the EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == ADXL345_IT_Pin) {
      /* Read acceleration data when interrupt occurs */

	  adxl345_data_ready_f = 1;

  }
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim7)
	{


	usb_print_f = 1;

	 //HAL_GPIO_TogglePin(PE3_GPIO_Port, PE3_Pin);
//		 ADXL345_I2C_ReadAccXYZ(&recivedData);
//		 x = (recivedData[1]<<8) | recivedData[0];
//		 y = (recivedData[3]<<8) | recivedData[2];
//		 z = (recivedData[5]<<8) | recivedData[4];
//

		 // SEnsitivity or scale factor at 10bit mode with +-4g -> 7.8mg/LSB
//		 gx = x * 0.0078;
//		 gy = y * 0.0078;
//		 gz = z * 0.0078;



	}

	if (htim == &htim6)
	{
		error_cnt++;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	// Second half od the buffer is ready to process (buf[128-255])
	if (hadc == &hadc1)
	{
		SCB_InvalidateDCache_by_Addr((uint32_t*)&load_current_buff[LOAD_CURRENT_BUFFER_LENGHT/2], ((LOAD_CURRENT_BUFFER_LENGHT/2) * sizeof(uint32_t)+31U)&~31U);
		process_rms(&load_current_buff[LOAD_CURRENT_BUFFER_LENGHT/2], LOAD_CURRENT_BUFFER_LENGHT/2);
	}

	if (hadc == &hadc2)
	{
		adc2_sampl_ready_f = 1;
	}
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc == &hadc1)
    {
        // First half of buffer ready (adc1_buf[0..127])
		SCB_InvalidateDCache_by_Addr((uint32_t*)&load_current_buff[0], ((LOAD_CURRENT_BUFFER_LENGHT/2) * sizeof(uint32_t)+31U)&~31U);
		process_rms(&load_current_buff[0], LOAD_CURRENT_BUFFER_LENGHT/2);

    }
}





/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x90000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_8MB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.Size = MPU_REGION_SIZE_1MB;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	  HAL_Delay(500);
	  HAL_GPIO_TogglePin(PE3_GPIO_Port, PE3_Pin);

  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
