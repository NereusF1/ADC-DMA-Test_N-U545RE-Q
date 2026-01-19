/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC1ChannelCount 4
#define ADC4ChannelCount 2
#define AccValBufLen 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc4;
DMA_NodeTypeDef Node_GPDMA1_Channel1;
DMA_QListTypeDef List_GPDMA1_Channel1;
DMA_HandleTypeDef handle_GPDMA1_Channel1;
DMA_NodeTypeDef Node_GPDMA1_Channel0;
DMA_QListTypeDef List_GPDMA1_Channel0;
DMA_HandleTypeDef handle_GPDMA1_Channel0;

/* USER CODE BEGIN PV */

uint32_t ADC1_res_buffer[ADC1ChannelCount*2]; // create a buffer with length equal to the Number of Conversions x 2 ; Word size = uint32_t
uint16_t ADC1_res_bkp_buffer[ADC1ChannelCount];
uint32_t ADC1_acc_val_buffer[ADC1ChannelCount]; //accumulated value
uint16_t  ADC1AccValBufCount;  // accumulate 100 maybe 1000 samples
uint32_t ADC1_acc_val_batch_buffer[ADC1ChannelCount];
uint16_t ADC1AccValBufBatchCount;
float ADC1_avg_val_buffer[ADC1ChannelCount];
float ADC1_pin_Volts_buffer[ADC1ChannelCount];

uint32_t cal_vref_data;
uint32_t vref_voltage_mv;
uint32_t vrefint_data;

uint32_t ADC1_ch1_raw;
uint32_t ADC1_ch1_mV;

uint32_t ADC1_ch2_raw;
uint32_t ADC1_ch2_mV;

uint32_t temp_raw;
int32_t temp_celsius;

uint32_t ADC4_res_buffer[ADC4ChannelCount*2]; // create a buffer with length equal to the Number of Conversions x 2 ; Word size = uint32_t
uint16_t ADC4_res_bkp_buffer[ADC4ChannelCount];
uint32_t ADC4_acc_val_buffer[ADC4ChannelCount]; //accumulated value
uint16_t  ADC4AccValBufCount;  // accumulate 100 maybe 1000 samples
uint32_t ADC4_acc_val_batch_buffer[ADC4ChannelCount];
uint16_t ADC4AccValBufBatchCount;
float ADC4_avg_val_buffer[ADC4ChannelCount];
float ADC4_pin_Volts_buffer[ADC4ChannelCount];

uint32_t ADC4_ch1_raw;
uint32_t ADC4_ch1_mV;

uint32_t ADC4_ch2_raw;
uint32_t ADC4_ch2_mV;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void SystemPower_Config(void);
static void MX_GPIO_Init(void);
static void MX_GPDMA1_Init(void);
static void MX_ICACHE_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC4_Init(void);
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

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the System Power */
  SystemPower_Config();

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_GPDMA1_Init();
  MX_ICACHE_Init();
  MX_ADC1_Init();
  MX_ADC4_Init();
  /* USER CODE BEGIN 2 */
	HAL_Delay(2000);  // hopefully 2s is a long time in the MCU world for the ADC peripheral to power up and get ready for calibration
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED); // this is optional and uses internal calibration values programmed by the factory
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc4, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc4, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
	HAL_Delay(1000); // wait for things to settle down before starting the ADC process

	HAL_ADC_Start_DMA(&hadc1, ADC1_res_buffer, ADC1ChannelCount*2); // this actually starts the process: DMA-transfer the data stream from &hadc1 into ADC1_res_buffer, which holds 2 conversions
	HAL_ADC_Start_DMA(&hadc4, ADC4_res_buffer, ADC4ChannelCount*2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(ADC1AccValBufBatchCount>0)
		{
			for (int i=0; i<ADC1ChannelCount; i++)
			{
				ADC1_avg_val_buffer[i]=(ADC1_avg_val_buffer[i] + (((float)ADC1_acc_val_batch_buffer[i]/(float)ADC1AccValBufBatchCount)))/2.0;
				ADC1_acc_val_batch_buffer[i]=0;  // clear the accumulated value
			}
			ADC1AccValBufBatchCount=0;   // clear the accumulated count

	          // Calculate the VREF using VREFINT
			if(ADC1_avg_val_buffer[0]<1) // protect from divide by zero
			{
				vref_voltage_mv=0; //adcbufAvgValue[0] will ramp-up and VREF calc will ramp-down slowly due to ADC averaging delays
			}
			else
			{
				// Read calibration data into RAM first if ICache is on
				cal_vref_data = *(__IO uint32_t *)VREFINT_CAL_ADDR; // Example address

				vrefint_data = ADC1_avg_val_buffer[0];

				// Calculate voltage in mV
				vref_voltage_mv = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(cal_vref_data, vrefint_data, ADC_RESOLUTION_14B);

				ADC1_ch1_raw =  ADC1_avg_val_buffer[1]; // Get raw ch1 reading
				ADC1_ch1_mV = __HAL_ADC_CALC_DATA_TO_VOLTAGE(&hadc1, vref_voltage_mv, ADC1_ch1_raw, ADC_RESOLUTION_14B); // Example ch1 calculation

				ADC1_ch2_raw =  ADC1_avg_val_buffer[2]; // Get raw ch1 reading
				ADC1_ch2_mV = __HAL_ADC_CALC_DATA_TO_VOLTAGE(&hadc1, vref_voltage_mv, ADC1_ch2_raw, ADC_RESOLUTION_14B); // Example ch1 calculation

				// Or for temperature:
				temp_raw = ADC1_avg_val_buffer[3]; // Get raw temp sensor reading
				temp_celsius = __HAL_ADC_CALC_TEMPERATURE(&hadc1, vref_voltage_mv, temp_raw, ADC_RESOLUTION_14B); // Example Temp calculation
			}

		}

		if(ADC4AccValBufBatchCount>0)
		{
			for (int i=0; i<ADC4ChannelCount; i++)
			{
				ADC4_avg_val_buffer[i]=(ADC4_avg_val_buffer[i] + (((float)ADC4_acc_val_batch_buffer[i]/(float)ADC4AccValBufBatchCount)))/2.0;
				ADC4_acc_val_batch_buffer[i]=0;  // clear the accumulated value
			}
			ADC4AccValBufBatchCount=0;   // clear the accumulated count

	          // Calculate the VREF using VREFINT
			if(ADC4_avg_val_buffer[0]<1) // protect from divide by zero
			{
				vref_voltage_mv=0; //adcbufAvgValue[0] will ramp-up and VREF calc will ramp-down slowly due to ADC averaging delays
			}
			else
			{
				ADC4_ch1_raw =  ADC4_avg_val_buffer[0]; // Get raw ch1 reading
				ADC4_ch1_mV = __HAL_ADC_CALC_DATA_TO_VOLTAGE(&hADC4, vref_voltage_mv, ADC4_ch1_raw, ADC_RESOLUTION_12B); // Example ch1 calculation

				ADC4_ch2_raw =  ADC4_avg_val_buffer[1]; // Get raw ch1 reading
				ADC4_ch2_mV = __HAL_ADC_CALC_DATA_TO_VOLTAGE(&hADC4, vref_voltage_mv, ADC4_ch2_raw, ADC_RESOLUTION_12B); // Example ch1 calculation
			}

		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_0;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV4;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the common periph clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADCDAC;
  PeriphClkInit.AdcDacClockSelection = RCC_ADCDACCLKSOURCE_PLL2;
  PeriphClkInit.PLL2.PLL2Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLL2.PLL2M = 3;
  PeriphClkInit.PLL2.PLL2N = 8;
  PeriphClkInit.PLL2.PLL2P = 2;
  PeriphClkInit.PLL2.PLL2Q = 2;
  PeriphClkInit.PLL2.PLL2R = 32;
  PeriphClkInit.PLL2.PLL2RGE = RCC_PLLVCIRANGE_1;
  PeriphClkInit.PLL2.PLL2FRACN = 0;
  PeriphClkInit.PLL2.PLL2ClockOut = RCC_PLL2_DIVR;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Power Configuration
  * @retval None
  */
static void SystemPower_Config(void)
{

  /*
   * Switch to SMPS regulator instead of LDO
   */
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }
/* USER CODE BEGIN PWR */
/* USER CODE END PWR */
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_14B;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_LOW;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_814CYCLES;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC4_Init(void)
{

  /* USER CODE BEGIN ADC4_Init 0 */

  /* USER CODE END ADC4_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC4_Init 1 */

  /* USER CODE END ADC4_Init 1 */

  /** Common config
  */
  hadc4.Instance = ADC4;
  hadc4.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc4.Init.Resolution = ADC_RESOLUTION_12B;
  hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc4.Init.ScanConvMode = ADC4_SCAN_ENABLE;
  hadc4.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc4.Init.LowPowerAutoPowerOff = ADC_LOW_POWER_NONE;
  hadc4.Init.LowPowerAutoWait = DISABLE;
  hadc4.Init.ContinuousConvMode = ENABLE;
  hadc4.Init.NbrOfConversion = 2;
  hadc4.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc4.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc4.Init.DMAContinuousRequests = ENABLE;
  hadc4.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_LOW;
  hadc4.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc4.Init.SamplingTimeCommon1 = ADC4_SAMPLETIME_814CYCLES_5;
  hadc4.Init.SamplingTimeCommon2 = ADC4_SAMPLETIME_1CYCLE_5;
  hadc4.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC4_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC4_SAMPLINGTIME_COMMON_1;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC4_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC4_Init 2 */

  /* USER CODE END ADC4_Init 2 */

}

/**
  * @brief GPDMA1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPDMA1_Init(void)
{

  /* USER CODE BEGIN GPDMA1_Init 0 */

  /* USER CODE END GPDMA1_Init 0 */

  /* Peripheral clock enable */
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /* GPDMA1 interrupt Init */
    HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);

  /* USER CODE BEGIN GPDMA1_Init 1 */

  /* USER CODE END GPDMA1_Init 1 */
  /* USER CODE BEGIN GPDMA1_Init 2 */

  /* USER CODE END GPDMA1_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Called when first half of adc buffer is filled
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{

	if(hadc->Instance == ADC1)
	{
			for (int i=0; i<ADC1ChannelCount ; i++)
			{
			  ADC1_acc_val_buffer[i]=ADC1_acc_val_buffer[i]+ADC1_res_buffer[i]; // moved all heavy lifting to 100ms timer loop
			  ADC1_res_bkp_buffer[i]=(uint16_t)ADC1_res_buffer[i];  // for viewing the instantaneous value only
			}

			ADC1AccValBufCount++;
			if(ADC1AccValBufCount>=AccValBufLen)
			{
				for (int i=0; i<ADC1ChannelCount; i++)
				{
					ADC1_acc_val_batch_buffer[i]=ADC1_acc_val_buffer[i]; // hand-over for processing in less critical threads
					ADC1_acc_val_buffer[i]=0;  // clear the accumulated value
				}
				ADC1AccValBufBatchCount=ADC1AccValBufCount; // hand-over for processing  in less critical threads
				ADC1AccValBufCount=0;   // clear the accumulated count
			}
	}

	if(hadc->Instance == ADC4)
	{
			for (int i=0; i<ADC4ChannelCount ; i++)
			{
			  ADC4_acc_val_buffer[i]=ADC4_acc_val_buffer[i]+ADC4_res_buffer[i]; // moved all heavy lifting to 100ms timer loop
			  ADC4_res_bkp_buffer[i]=(uint16_t)ADC4_res_buffer[i];  // for viewing the instantaneous value only
			}

			ADC4AccValBufCount++;
			if(ADC4AccValBufCount>=AccValBufLen)
			{
				for (int i=0; i<ADC4ChannelCount; i++)
				{
					ADC4_acc_val_batch_buffer[i]=ADC4_acc_val_buffer[i]; // hand-over for processing in less critical threads
					ADC4_acc_val_buffer[i]=0;  // clear the accumulated value
				}
				ADC4AccValBufBatchCount=ADC4AccValBufCount; // hand-over for processing  in less critical threads
				ADC4AccValBufCount=0;   // clear the accumulated count
			}
	}

}

// Called when adc buffer is completely filled
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

	if(hadc->Instance == ADC1)
	{
		for (int i=0; i<ADC1ChannelCount; i++)
		{
		  ADC1_acc_val_buffer[i]=ADC1_acc_val_buffer[i]+ADC1_res_buffer[ADC1ChannelCount+i]; // moved all heavy lifting to 100ms timer loop
		  ADC1_res_bkp_buffer[i]=(uint16_t)ADC1_res_buffer[ADC1ChannelCount+i];  // for viewing the instantaneous value only
		}

		ADC1AccValBufCount++;
		if(ADC1AccValBufCount>=AccValBufLen)
		{
			for (int i=0; i<ADC1ChannelCount; i++)
			{
				ADC1_acc_val_batch_buffer[i]=ADC1_acc_val_buffer[i]; // hand-over for processing  in less critical threads
				ADC1_acc_val_buffer[i]=0;  // clear the accumulated value
			}
			ADC1AccValBufBatchCount=ADC1AccValBufCount; // hand-over for processing  in less critical threads
			ADC1AccValBufCount=0;   // clear the accumulated count
		}
	}

	if(hadc->Instance == ADC4)
	{
		for (int i=0; i<ADC4ChannelCount; i++)
		{
		  ADC4_acc_val_buffer[i]=ADC4_acc_val_buffer[i]+ADC4_res_buffer[ADC4ChannelCount+i]; // moved all heavy lifting to 100ms timer loop
		  ADC4_res_bkp_buffer[i]=(uint16_t)ADC4_res_buffer[ADC4ChannelCount+i];  // for viewing the instantaneous value only
		}

		ADC4AccValBufCount++;
		if(ADC4AccValBufCount>=AccValBufLen)
		{
			for (int i=0; i<ADC4ChannelCount; i++)
			{
				ADC4_acc_val_batch_buffer[i]=ADC4_acc_val_buffer[i]; // hand-over for processing  in less critical threads
				ADC4_acc_val_buffer[i]=0;  // clear the accumulated value
			}
			ADC4AccValBufBatchCount=ADC4AccValBufCount; // hand-over for processing  in less critical threads
			ADC4AccValBufCount=0;   // clear the accumulated count
		}
	}
}

/* USER CODE END 4 */

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
