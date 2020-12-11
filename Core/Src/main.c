/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
const uint8_t HOUR_COLOR[24] = {1, 1, 1, 1, 1, 1, 1, 1, /*||*/ 1, 1, 1, 1, 1, 1, 1, 1, /*||*/ 0, 0, 0, 0, 0, 0, 0, 0,};
const uint8_t MIN_COLOR[24] =  {1, 1, 1, 1, 1, 1, 1, 1, /*||*/ 0, 0, 0, 0, 0, 0, 0, 0, /*||*/ 0, 0, 0, 0, 0, 0, 0, 0,};
const uint8_t SEC_COLOR[24] =  {0, 0, 0, 0, 0, 0, 0, 0, /*||*/ 1, 1, 1, 1, 1, 1, 1, 1, /*||*/ 0, 0, 0, 0, 0, 0, 0, 0,};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
volatile uint32_t last_button = 0;
uint8_t time_set_mode = 3;
RTC_TimeTypeDef current_time;
uint8_t all_led_grb[12][24];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void set_time() {
	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef DateToUpdate = {0};
	/** Initialize RTC and set the Time and Date
	*/
	sTime.Hours = 0x10;
	sTime.Minutes = 0x20;
	sTime.Seconds = 0x30;

	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
	{
	Error_Handler();
	}
	DateToUpdate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
	DateToUpdate.Month = RTC_MONTH_NOVEMBER;
	DateToUpdate.Date = 0x11;
	DateToUpdate.Year = 0x0;

	if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
	{
	Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2); // saves time to backup register
	/* USER CODE END RTC_Init 2 */
}

void time_backup_check() {
	if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x32F2) {
	  set_time();
	}
}

uint8_t is_btn_pressed() { // for dealing with the bounce effect
	uint8_t returnValue = 0;
	uint8_t buttonState = HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin);
	if (last_button == 0 && buttonState == 1) {
		returnValue = 1;
	}
	last_button = buttonState;
	return returnValue;

}

void onpress_change_time_set_mode() {
	if (is_btn_pressed()) {
		time_set_mode++;
		if (time_set_mode > 3) {
			time_set_mode = 0;
		}
		switch (time_set_mode) {
			case CLOCK_CHANGE_HOUR:
				TIM2->CNT = current_time.Hours << 2;
				break;
			case CLOCK_CHANGE_MIN:
				TIM2->CNT = current_time.Minutes << 2;
				break;
			case CLOCK_CHANGE_SEC:
				TIM2->CNT = current_time.Seconds << 2;
				break;
		}
	}
}

void handle_time_change() {
	RTC_TimeTypeDef new_time;
	switch (time_set_mode) {
		case CLOCK_CHANGE_HOUR:
			time_backup_check();
			HAL_RTC_GetTime(&hrtc, &new_time, RTC_FORMAT_BIN);
			new_time.Hours = ((TIM2->CNT)>>2) % 24;
			HAL_RTC_SetTime(&hrtc, &new_time, RTC_FORMAT_BIN);
			break;
		case CLOCK_CHANGE_MIN:
			time_backup_check();
			HAL_RTC_GetTime(&hrtc, &new_time, RTC_FORMAT_BIN);
			new_time.Minutes = ((TIM2->CNT)>>2) % 60;
			HAL_RTC_SetTime(&hrtc, &new_time, RTC_FORMAT_BIN);
			break;
		case CLOCK_CHANGE_SEC:
			time_backup_check();
			HAL_RTC_GetTime(&hrtc, &new_time, RTC_FORMAT_BIN);
			new_time.Seconds = ((TIM2->CNT)>>2) % 60;
			HAL_RTC_SetTime(&hrtc, &new_time, RTC_FORMAT_BIN);
			break;
	}
}

void set_grb_by_time() {
	memset(all_led_grb, 0, sizeof all_led_grb); // reset coloring array
	memcpy(all_led_grb[current_time.Hours % 12], HOUR_COLOR, sizeof(HOUR_COLOR));
	memcpy(all_led_grb[(uint8_t)(current_time.Minutes/5)], MIN_COLOR, sizeof(MIN_COLOR));
	memcpy(all_led_grb[(uint8_t)(current_time.Seconds/5)], SEC_COLOR, sizeof(SEC_COLOR));
	// TODO 5 shades of a color
	// TODO different overlap colors
	HAL_Delay(300);
}

uint8_t led_index = 0;
uint8_t max_led_index = 12;
uint8_t grb_index = 0;
uint8_t max_grb_index = 24;
uint16_t led_cycle_counter = 0;
uint16_t max_led_cycle_period = 288;
uint8_t reset_cycle_period = 40;

void handle_led_changes() {
	if (led_cycle_counter >= 0 && led_cycle_counter < max_led_cycle_period) {
		if (all_led_grb[led_index][grb_index] == 0) {
		  htim4.Instance-> CCR1 = htim4.Instance->ARR * (1/3);	// if grb bit is null
		}
		if (all_led_grb[led_index][grb_index] == 1) {
		  htim4.Instance-> CCR1 = htim4.Instance->ARR * (2/3);	// if grb bit is one
		}
		grb_index++;
		if (grb_index >= max_grb_index) {
			grb_index = 0;
			led_index++;
			if (led_index >= max_led_index) {
				led_index = 0;
			}
		}
		led_cycle_counter++;
	} else if (led_cycle_counter >= max_led_cycle_period && led_cycle_counter < (max_led_cycle_period + reset_cycle_period)) {
		htim4.Instance-> CCR1 = 0;
		led_cycle_counter++;
		if (led_cycle_counter == max_led_cycle_period + reset_cycle_period) {
			led_cycle_counter = 0;
		}
	}

	/*for (led_index = 0; led_index < max_led_index; led_index++) {
		for (grb_index = 0; grb_index < max_grb_index; grb_index++) {
			if (grb[led_index][grb_index] == 0) {
			  htim4.Instance-> CCR1 = htim4.Instance->ARR * (1/3);
			}
			if (grb[led_index][grb_index] == 1) {
			  htim4.Instance-> CCR1 = htim4.Instance->ARR * (2/3);
			}
		}
	}
	for (int i = 0; i < reset_cycle_period; i++) {
		htim4.Instance-> CCR1 = 0;
	}*/
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	char *outBufferUSB[512] = {0};
	uint16_t outBufferUSBSize = 0;
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
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_RTC_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  time_backup_check(); // restores time from backup register
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
	  HAL_RTC_GetTime(&hrtc, &current_time, RTC_FORMAT_BIN); // gets current time to display
	  onpress_change_time_set_mode(); // checks for button press and increments change_time_mod variable
	  handle_time_change(); // handles time change with the encoder
	  set_grb_by_time(); // sets grb array by the current time
	  handle_led_changes(); // supposed to change one leds color for now

	  // TODO cleanup (vars, comments, oddities) and functions to different files maybe
	  // TODO led blink on change time modes - alarm with if condition
	  // TODO readme

	  outBufferUSBSize = sprintf(outBufferUSB, "value: %d, count: %d, time: %02d:%02d:%02d%, led: %d, grb: %d, cycle: %d, hour_l: %d, min_l: %d, sec_l: %d\n\r",
	  			  time_set_mode, ((TIM2->CNT)>>2), current_time.Hours, current_time.Minutes, current_time.Seconds, led_index, grb_index, led_cycle_counter,
				  all_led_grb[0][0], all_led_grb[1][0], all_led_grb[2][8]);
	  CDC_Transmit_FS(outBufferUSB, outBufferUSBSize);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x10;
  sTime.Minutes = 0x20;
  sTime.Seconds = 0x30;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
  DateToUpdate.Month = RTC_MONTH_NOVEMBER;
  DateToUpdate.Date = 0x11;
  DateToUpdate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2); // saves time to backup register
  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 89;
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
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim4, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
