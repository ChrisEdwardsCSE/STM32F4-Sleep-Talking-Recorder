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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "spi-driver.h"
#include "sdcard_audio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_LEN 4096
#define THRESHOLD 2400

#define DEBUG_MODE 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
spi_handler_t spi1_handler;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2S2_Init(void);
/* USER CODE BEGIN PFP */

uint16_t adc_buf[ADC_BUF_LEN]; // note this is 4096 words, so 8192 bytes

extern FATFS fatfs; // fatfs handler;
extern FIL fil; // file handler
extern uint32_t fil_size;

FRESULT sdcard_op_result;

FILINFO *file_info;

uint8_t off_listening_button,				// Button pushed
		start_stop_listening,		// Start/Stop' listening
		stop_recording_flag,		// Stop the recording
		start_recording_flag,		// For restarting the recording ***** CLEAN UP
		already_recorded, 			// Already recorded once ***** NEED TO CLEAN THIS UP
		noise_flag,					// Noise detected
		dma_half_full,				// DMA Half Full flag
		dma_full;					// DMA Full flag
uint16_t adc_buf_index;
uint8_t sdcard_write_en;

// testing speaker
uint8_t button_playback;

// file prepared
uint8_t file_prepared;

/* Enumerated type for device's state */
enum device_state_enum
{
	OFF,
	LISTENING,
	RECORDING,
	PLAYBACK
} device_state;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// DMA buffer half full - Write lower half to SD Card
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
	dma_half_full = 1;
}

// DMA buffer full - Write upper half to SD Card
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	dma_full = 1;
}

// Executes an ADC reading of microphone at sampling rate 44.1kHz
void TIM3_IT_Handler(void)
{
	__HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_CC1); // Reset interrupt flag bit
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, ADC_BUF_LEN); // Take one reading. Note that CubeIDE acknowledges adc_buf in half-words


	// If noise detected, enable SD Card Write
	if (adc_buf[adc_buf_index] >= THRESHOLD)
	{
		// Reset 5s silence detection timer if hear noise while in RECORDING state
		if (device_state == RECORDING)
		{
			TIM4->CNT = 0;
		}

		// Start 5s Silence Detection Timer if hear noise while in LISTENING state
		else if (device_state == LISTENING)
		{
			device_state = RECORDING;

			HAL_TIM_Base_Start_IT(&htim4); // Start 5s Silence Detection Timer

			start_recording_flag = 1; // **************** if listening and hear noise, set this high which prepares a file to write to
		}
	}

	adc_buf_index++; // Increment index following DMA buffer's current index

	// Reset index if DMA at the end of the buffer
	if (adc_buf_index >= ADC_BUF_LEN)
	{
		adc_buf_index = 0;
	}
}

/**
 * 5s Silence Detection Timer
 * Counts 5 seconds. Used for switching device_state RECORDING to LISTENING
 * if there's no noise for 5 seconds.
 */
void TIM4_IT_Handler(void)
{
	__HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_CC1); // Reset interrupt flag bit

	device_state = LISTENING;
	stop_recording_flag = 1; // Tell while(1) to stop the recording and call stop_recording()
//	already_recorded = 1; // We've just finished a recording, so could be the first one
	HAL_TIM_Base_Stop_IT(&htim4);
}

// Timer Period Elapsed controller
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// ADC Reading Timer
	if (htim == &htim3)
	{
		TIM3_IT_Handler();
	}
	// 5s Silence Detection Timer
	else if (htim == &htim4)
	{
		TIM4_IT_Handler();
	}
}

/**
 * Buttons controller
 * B1 - switches device between OFF and LISTENING states
 * PC12 - switches device between OFF and PLAYBACK states
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == (1 << 13))
	{
		off_listening_button = 1;
	}
	else if (GPIO_Pin == (1 << 12) && device_state != PLAYBACK)
	{
		device_state = PLAYBACK;
		button_playback = 1;
	}
}

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

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2S2_Init();
  /* USER CODE BEGIN 2 */

#if DEBUG_MODE
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM3_STOP;
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM4_STOP;
#endif

  // Configure and initialize SPI1 peripheral
  spi_init_t spi1_init_handler;
  spi1_init_handler.CLKPhase = 0;
  spi1_init_handler.CLKPolarity = 0;
  spi1_init_handler.Prescaler = 0b011; // SD Card's SPI CLK < 8MHz
  spi1_init_handler.DataSize = 0;
  spi1_init_handler.FirstBit = 0;
  spi1_init_handler.Mode = 1;
  spi1_init_handler.NSS = 1; // SW NSS Handling
  spi1_init_handler.Direction = 0;

  spi1_handler.Instance = SPI1;
  spi1_handler.Init = spi1_init_handler;
  SPI_Init(&spi1_handler);

  adc_buf_index = 0;
  device_state = OFF;
  stop_recording_flag = 0;
  already_recorded = 0;

  // Initialize SD Card
  sdcard_op_result = sdcard_init();
  if (sdcard_op_result != FR_OK) { goto error; }
  HAL_Delay(50);

  // Clear all .wav files from the previous night
  sdcard_op_result = sdcard_clear_files(file_info);
  if (sdcard_op_result != FR_OK) { goto error; }


  sdcard_op_result = sdcard_prepare_wav_file(44100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  // B1 Button pushed
	  if (off_listening_button)
	  {
		  // Stop listening
		  if (device_state != OFF)
		  {
			  HAL_ADC_Stop_DMA(&hadc1);
			  HAL_TIM_Base_Stop_IT(&htim3);
			  device_state = OFF;

			  sdcard_op_result = sdcard_close_wav_file(); // Catches if in RECORDING and button is push
			  if (sdcard_op_result != FR_OK) { goto error; }

			  // Reset DMA buffer
			  dma_half_full = 0;
			  dma_full = 0;
		  }
		  // Start listening
		  else
		  {
			  device_state = LISTENING;
			  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, ADC_BUF_LEN); // Take the first
			  HAL_TIM_Base_Start_IT(&htim3); // Start timer for ADC microphone sampling
		  }

		  off_listening_button = 0;
	  }

	  /**
	   * If we already recorded once
	   * don't use device = RECORDING here, we want a start recording command, not a "are we recording?" value
	   */
	  if (start_recording_flag == 1)// if (start_recording_flag == 1 && already_recorded == 1)
	  {
		  sdcard_op_result = sdcard_prepare_wav_file(44100);
		  if (sdcard_op_result != FR_OK) { goto error; }

		  start_recording_flag = 0;
		  file_prepared = 1;
	  }

	  // ************ PROBABLY gotta change this so that it creates the next file ready to write before we hear noise.
	  // If we don't end up using the file, delete it, no biggie, that's much better than what we got here

	  /**
	   * If in listening state, and there was noise detected (sdcard_write_en), and DMA half/completely full,
	   * write out the contents to the SD card
	   */
	  if (device_state == RECORDING && dma_half_full == 1 && file_prepared == 1)
	  {
		  if (file_prepared != 1)
		  {
			  sdcard_op_result = sdcard_prepare_wav_file(44100);
			  if (sdcard_op_result != FR_OK) { goto error; }
			  file_prepared = 1;
		  }
		  sdcard_op_result = sdcard_wav_write((uint8_t *)adc_buf, ADC_BUF_LEN); // Total bytes = ADC_BUF_LEN*2, we only want half here
		  if (sdcard_op_result != FR_OK) { goto error; }
		  dma_half_full = 0;
	  }
	  if (device_state == RECORDING && dma_full == 1 && file_prepared == 1)
	  {
		  if (file_prepared != 1)
		  {
			  sdcard_op_result = sdcard_prepare_wav_file(44100);
			  if (sdcard_op_result != FR_OK) { goto error; }
			  file_prepared = 1;
		  }
		  sdcard_op_result = sdcard_wav_write((uint8_t *)adc_buf + ADC_BUF_LEN, ADC_BUF_LEN);
		  if (sdcard_op_result != FR_OK) { goto error; }

		  dma_full = 0;
	  }



	  // No noise was detected for 5s during RECORDING, transition RECORDING -> LISTENING
	  if (stop_recording_flag == 1)
	  {
		  sdcard_op_result = sdcard_close_wav_file();
		  if (sdcard_op_result != FR_OK) { goto error; }
		  stop_recording_flag = 0;
		  file_prepared = 0;
	  }

	/**
	* Play recordings, will only get here if not listening/recording, maybe put another device state
	* for playing
	*/
	if (button_playback)
	{
		char file_name_read[] = "w_00.wav";
		uint8_t file_digits_read = 0;
		sdcard_op_result = f_open(&fil, file_name_read, FA_READ); // Open the first file
		if (sdcard_op_result != FR_OK) { goto error; }

		// Play back all files on SD Card
		while (sdcard_op_result != FR_NO_FILE)
		{
			UINT bytes_read;
			UINT bytes_to_read = ADC_BUF_LEN / 2;

			// Individual file loop
			do
			{
				/**
				 * Double buffer approach using halves of adc_buf.
				 * First 1st half of buffer with SD Card data. Once done, start sending to
				 * the amplifier over I2S + DMA. While that's going, start reading more SD Card
				 * data to 2nd half of buffer. Then send that over I2S + DMA to amplifier.
				 * Repeat.
				 */
				sdcard_op_result = f_read(&fil, (void *)adc_buf, bytes_to_read, (UINT *)&bytes_read);
				if (sdcard_op_result != FR_OK) { goto error; }
				HAL_I2S_Transmit_DMA(&hi2s2, adc_buf, bytes_to_read); // Send first half of adc_buf

				// If read all bytes, read the second half
				if (bytes_to_read == bytes_read)
				{
					sdcard_op_result = f_read(&fil, (void *)adc_buf + bytes_to_read, bytes_to_read, (UINT *)&bytes_read);
					if (sdcard_op_result != FR_OK) { goto error; }
					HAL_I2S_Transmit_DMA(&hi2s2, adc_buf + bytes_to_read, bytes_to_read); // Send 2nd half of adc_buf
				}
			} while (bytes_read == bytes_to_read);
			sdcard_op_result = f_close(&fil);
			if (sdcard_op_result != FR_OK) { goto error; }
			HAL_Delay(2000);
			// if here, then hit the end of the file. Increment file name
			file_digits_read++;
			file_name_read[2] = file_digits_read / 10 + 48;
			file_name_read[3] = file_digits_read % 10 + 48;

			sdcard_op_result = f_open(&fil, file_name_read, FA_READ); // Open the new file
			if (sdcard_op_result != FR_OK) { goto error; }
		}
		/* ******** MY CODE END ******** */
		HAL_I2S_DMAStop(&hi2s2);
		button_playback = 0;
		device_state = OFF;
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	error:
	return 1;
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_MSB;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_44K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 3-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 315;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 65535-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 6409;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PC13 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
