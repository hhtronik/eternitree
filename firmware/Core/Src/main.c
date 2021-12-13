/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
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
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim21;

/* USER CODE BEGIN PV */
int mode = -1;
bool switchMode = true;
long lastMostSwitch = 0;

// min. delay between 2 mode switches (in ms)
#define MODESWITCHDELAY 5
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM21_Init(void);
/* USER CODE BEGIN PFP */
void MX_GPIO_Deinit(void);
static void StopMode(void);
static void ModeBlink_Init(uint16_t brght, uint32_t wkup);
static void ModeBlink_Loop();
static void ModeFlake_Init(uint16_t brght, uint32_t wkup);
static void ModeFlake_Loop();
static void ModeStatic_Init(int arr, int wkup);
static void ModeStatic_Loop();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// simple 16bit PRNG
static uint16_t prng_lfsr16(void)
{
  static uint16_t cnt16 = 91;
  return (cnt16 = (cnt16 >> 1) ^ (-(cnt16 & 1) & 0xB400));
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

  /* Before we can access to every register of the PWR peripheral we must enable it */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_TIM21_Init();
  /* USER CODE BEGIN 2 */

  // hack to enter programming mode easily as backup...
  // hold the button to enter the wait loop for 20s
  if(HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin) == false)
  {
	  int i, j;

	  // blink the topmost LED 500ms on 500ms off
	  for(i = 40000; i > 0; i -= 1000)
	  {
		  for( j = 5000; j > 0; j--)
		  {
			  HAL_TIM_PWM_Start(&htim21, TIM_CHANNEL_1);
		  }

		  HAL_Delay(500);
	  }
  }

  // add half a second of wait time to allow for easier flashing of the device (before it
  // goes to STOP mode too quickly <:O
  HAL_Delay(500);

  // for some reason we have to start with tim21 to not bog down the power supply at
  // startup... so to be sure we do one short pulse here before going to main loop
  HAL_TIM_PWM_Start(&htim21, TIM_CHANNEL_1);
  HAL_Delay(20);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // do some mode switching
	  if(switchMode)
	  {
		  switchMode = false;
		  mode++;

		  // loop the mode counter...
		  if(mode > 8)
			  mode = 0;

		  switch(mode)
		  {
		  case 0:
			  ModeStatic_Init(3, 328);
			  break;
		  case 1:
			  ModeStatic_Init(2, 328);
			  break;
		  case 2:
			  ModeStatic_Init(1, 328);
			  break;
		  case 3:
			  ModeBlink_Init(800, 6500);
			  break;
		  case 4:
			  ModeBlink_Init(400, 6500);
			  break;
		  case 5:
			  ModeBlink_Init(75, 6500);
			  break;
		  case 6:
			  ModeFlake_Init(800, 6500);
			  break;
		  case 7:
			  ModeFlake_Init(400, 6500);
			  break;
		  case 8:
			  ModeFlake_Init(75, 6500);
			  break;
		  default:
			  ModeStatic_Init(3, 328);
			  break;

		  }

	  }

	  // do the actual loop-work
	  switch(mode)
	  {
	  case 0:
		  ModeStatic_Loop();
		  ModeStatic_Loop();
		  break;
	  case 1:
	  case 2:
		  ModeStatic_Loop();
		  break;
	  case 3:
	  case 4:
	  case 5:
		  // call that twice to give some chance of having two lights on at the same time ;)
		  ModeBlink_Loop();
		  ModeBlink_Loop();
		  break;
	  case 6:
	  case 7:
	  case 8:
		  ModeFlake_Loop();
		  break;
	  default:
		  ModeStatic_Loop();
		  break;

	  }

	  // go to low power for a while until the RTC's AWU wakes us
	  StopMode();


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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
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

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the WakeUp 
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 328, RTC_WAKEUPCLOCK_RTCCLK_DIV2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM21 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM21_Init(void)
{

  /* USER CODE BEGIN TIM21_Init 0 */

  /* USER CODE END TIM21_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM21_Init 1 */

  /* USER CODE END TIM21_Init 1 */
  htim21.Instance = TIM21;
  htim21.Init.Prescaler = 0;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = 2;
  htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim21.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim21) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim21, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim21, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim21, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM21_Init 2 */

  /* USER CODE END TIM21_Init 2 */
  HAL_TIM_MspPostInit(&htim21);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */


void StopMode(void)
{
  //MX_GPIO_Deinit();

  /* Configure User push-button as external interrupt generator */
  //MX_GPIO_Init();

  /* Suspend Tick increment to prevent wakeup by Systick interrupt.
     Otherwise the Systick interrupt will wake up the device within 1ms (HAL time base) */
  HAL_SuspendTick();

  /* disable the flash in power down mode */
  __HAL_FLASH_POWER_DOWN_DISABLE();

  /* We enable again the PWR peripheral */
  __HAL_RCC_PWR_CLK_ENABLE();
  /* Request to enter SLEEP mode */
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);

  /* Resume Tick interrupt if disabled prior to sleep mode entry*/
  HAL_ResumeTick();

  /* Reinitialize GPIOs */
  //MX_GPIO_Init();

  /* Reinitialize UART2 */
  //MX_USART2_UART_Init();
}


void MX_GPIO_Deinit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Configure all GPIO as analog to reduce current consumption on non used IOs */
  /* Enable GPIOs clock */
  /* Warning : Reconfiguring all GPIO will close the connection with the debugger */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Pin = GPIO_PIN_All;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Disable GPIOs clock */
  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_DISABLE();
  __HAL_RCC_GPIOC_CLK_DISABLE();
}

void BTN_Pressed(void)
{
	uint32_t now = HAL_GetTick();

	// dumb debouncing
	if(lastMostSwitch + MODESWITCHDELAY < now)
	{
		lastMostSwitch = now;
		switchMode = true;
	}
}

/*
void ModeBlink_Init(int speed);
void ModeBlink_Loop();
void ModeFlake_Init(int speed);
void ModeFlake_Loop();
*/

void ModeStatic_Init(int arr, int wkup)
{
	// make sure the GPIOs are in the expected state
	HAL_TIM_MspPostInit(&htim2);
	HAL_TIM_MspPostInit(&htim21);

	// wake the MCU every ~18ms or so
	HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, wkup, RTC_WAKEUPCLOCK_RTCCLK_DIV2);

	// limit the max intensity... this is SYSCLK dependent and
	// might short the supply to ground if the inductors on board
	// get saturated.
	if(arr > 6) arr = 6;

	__HAL_TIM_SET_AUTORELOAD(&htim2, arr);
	__HAL_TIM_SET_AUTORELOAD(&htim21, arr);

	  // Reset the OPM Bit to Single shot
	  htim2.Instance->CR1 &= ~TIM_CR1_OPM;
	  htim2.Instance->CR1 |= TIM_OPMODE_SINGLE;
	  // Reset the OPM Bit to Single shot
	  htim21.Instance->CR1 &= ~TIM_CR1_OPM;
	  htim21.Instance->CR1 |= TIM_OPMODE_SINGLE;
}

void ModeStatic_Loop()
{
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim21, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim21, TIM_CHANNEL_1);
}

uint16_t blink_brighness = 0;
uint16_t blink_step = 0;

void ModeBlink_Init(uint16_t brght, uint32_t wkup)
{
	HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, wkup, RTC_WAKEUPCLOCK_RTCCLK_DIV2);
	blink_brighness = brght;
}


static void ModeBlink_Loop()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	TIM_HandleTypeDef *timer;
	uint8_t channel;

	blink_step = prng_lfsr16();

	if(blink_step < (0xffff / 4))
	{
		timer = &htim2;
		channel = TIM_CHANNEL_1;

		// MOS 1 driven
	    GPIO_InitStruct.Pin = MOS1_Pin;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	    GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
	    HAL_GPIO_Init(MOS1_GPIO_Port, &GPIO_InitStruct);

	    // MOS 2 input
		GPIO_InitStruct.Pin = MOS2_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		HAL_GPIO_Init(MOS1_GPIO_Port, &GPIO_InitStruct);
	}
	else if(blink_step < 0xffff / 4 * 2)
	{
		timer = &htim2;
		channel = TIM_CHANNEL_2;

		// MOS 1 driven
		GPIO_InitStruct.Pin = MOS2_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
		HAL_GPIO_Init(MOS1_GPIO_Port, &GPIO_InitStruct);

		// MOS 2 input
		GPIO_InitStruct.Pin = MOS1_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		HAL_GPIO_Init(MOS1_GPIO_Port, &GPIO_InitStruct);
	}
	else if(blink_step < 0xffff / 4 * 3)
	{
		timer = &htim21;
		channel = TIM_CHANNEL_1;

		// MOS 4 driven
	    GPIO_InitStruct.Pin = MOS4_Pin;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	    GPIO_InitStruct.Alternate = GPIO_AF5_TIM21;
	    HAL_GPIO_Init(MOS4_GPIO_Port, &GPIO_InitStruct);

	    // MOS 3 input
	    GPIO_InitStruct.Pin = MOS3_Pin;
	    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	    GPIO_InitStruct.Alternate = GPIO_AF0_TIM21;
	    HAL_GPIO_Init(MOS3_GPIO_Port, &GPIO_InitStruct);
	}
	else
	{
		timer = &htim21;
		channel = TIM_CHANNEL_2;

		// MOS 4 input
	    GPIO_InitStruct.Pin = MOS4_Pin;
	    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	    GPIO_InitStruct.Alternate = GPIO_AF5_TIM21;
	    HAL_GPIO_Init(MOS4_GPIO_Port, &GPIO_InitStruct);

	    // MOS 3 driven
	    GPIO_InitStruct.Pin = MOS3_Pin;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	    GPIO_InitStruct.Alternate = GPIO_AF0_TIM21;
	    HAL_GPIO_Init(MOS3_GPIO_Port, &GPIO_InitStruct);
	}

	for(int i = 0; i < blink_brighness; i++)
	{
		HAL_TIM_PWM_Start(timer, channel);
	}
}


void ModeFlake_Init(uint16_t brght, uint32_t wkup)
{
	HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, wkup, RTC_WAKEUPCLOCK_RTCCLK_DIV2);
	blink_brighness = brght;
}


static void ModeFlake_Loop()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	TIM_HandleTypeDef *timer;
	uint8_t channel;

	blink_step++;

	if(blink_step > 3)
		blink_step = 0;

	if(blink_step == 0)
	{
		timer = &htim2;
		channel = TIM_CHANNEL_1;

		// MOS 1 driven
	    GPIO_InitStruct.Pin = MOS1_Pin;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	    GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
	    HAL_GPIO_Init(MOS1_GPIO_Port, &GPIO_InitStruct);

	    // MOS 2 input
		GPIO_InitStruct.Pin = MOS2_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		HAL_GPIO_Init(MOS1_GPIO_Port, &GPIO_InitStruct);
	}
	else if(blink_step == 1)
	{
		timer = &htim2;
		channel = TIM_CHANNEL_2;

		// MOS 1 driven
		GPIO_InitStruct.Pin = MOS2_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
		HAL_GPIO_Init(MOS1_GPIO_Port, &GPIO_InitStruct);

		// MOS 2 input
		GPIO_InitStruct.Pin = MOS1_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		HAL_GPIO_Init(MOS1_GPIO_Port, &GPIO_InitStruct);
	}
	else if(blink_step == 2)
	{
		timer = &htim21;
		channel = TIM_CHANNEL_1;

		// MOS 4 driven
	    GPIO_InitStruct.Pin = MOS4_Pin;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	    GPIO_InitStruct.Alternate = GPIO_AF5_TIM21;
	    HAL_GPIO_Init(MOS4_GPIO_Port, &GPIO_InitStruct);

	    // MOS 3 input
	    GPIO_InitStruct.Pin = MOS3_Pin;
	    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	    GPIO_InitStruct.Alternate = GPIO_AF0_TIM21;
	    HAL_GPIO_Init(MOS3_GPIO_Port, &GPIO_InitStruct);
	}
	else
	{
		timer = &htim21;
		channel = TIM_CHANNEL_2;

		// MOS 4 input
	    GPIO_InitStruct.Pin = MOS4_Pin;
	    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	    GPIO_InitStruct.Alternate = GPIO_AF5_TIM21;
	    HAL_GPIO_Init(MOS4_GPIO_Port, &GPIO_InitStruct);

	    // MOS 3 driven
	    GPIO_InitStruct.Pin = MOS3_Pin;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	    GPIO_InitStruct.Alternate = GPIO_AF0_TIM21;
	    HAL_GPIO_Init(MOS3_GPIO_Port, &GPIO_InitStruct);
	}

	for(int i = 0; i < blink_brighness; i++)
	{
		HAL_TIM_PWM_Start(timer, channel);
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
