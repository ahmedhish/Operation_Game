/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "ssd1306.h"
#include "fonts.h"
#include "DFPLAYER_MINI.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
	OLED1=0,
	OLED2,
	OLED3,
	OLED4
}OLED_type;

typedef struct{
	GPIO_TypeDef *	In_Port;
	uint16_t In_Pin;
}Input_H;

typedef enum{
	Interrupted,
	Timeout,
	Waiting
}TIM_State;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define OLED1_ADD	0x78
#define OLED2_ADD	0x7A
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

osThreadId LCDHandle;
osThreadId Play_SoundHandle;
osThreadId GamePlayHandle;
osSemaphoreId OLED1_semHandle;
osSemaphoreId OLED2_semHandle;
osSemaphoreId OLED3_semHandle;
osSemaphoreId OLED4_semHandle;
osSemaphoreId Music_semHandle;
/* USER CODE BEGIN PV */
uint32_t Score_Player[4];
Input_H Input_Position [] = {
		{Plate_1_GPIO_Port,Plate_1_Pin},{Plate_2_GPIO_Port,Plate_2_Pin},{Plate_3_GPIO_Port,Plate_3_Pin},{Plate_4_GPIO_Port,Plate_4_Pin},
		{Plate_5_GPIO_Port,Plate_5_Pin},{Plate_6_GPIO_Port,Plate_6_Pin},{Plate_7_GPIO_Port,Plate_7_Pin},{Plate_8_GPIO_Port,Plate_8_Pin},
		{Plate_9_GPIO_Port,Plate_9_Pin},{Plate_10_GPIO_Port,Plate_10_Pin},{Plate_11_GPIO_Port,Plate_11_Pin},{Plate_12_GPIO_Port,Plate_12_Pin},
		{Plate_13_GPIO_Port,Plate_13_Pin}
};
unsigned char Score_Plate[] = {
		1,2,3,4,
		5,6,7,8,
		9,10,11,12,
		13
};
uint16_t Twizzers_Pins[] = {	Twizzer_1_Pin , Twizzer_2_Pin , Twizzer_3_Pin , Twizzer_4_Pin};


char Start_Time = -1;
TIM_State State = Interrupted;

uint8_t PlatesCounter;

char* Colors[] = {"Blue","Red","Green", "Yellow"};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
void LCD_Function(void const * argument);
void Sound_Function(void const * argument);
void Game(void const * argument);

/* USER CODE BEGIN PFP */
uint8_t get_timeIn(void);
void print_OLED(OLED_type OLED, char* string);

void Start_Game(void);

void End_Game(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t get_timeIn(void)
{
	uint32_t adc_value =0;

	HAL_ADC_Start(&hadc1);
		  // ADC poll for conversion
	HAL_ADC_PollForConversion(&hadc1, 100);
		  // get the ADC conversion value
	adc_value = HAL_ADC_GetValue(&hadc1);
		  // end ADC conversion
	HAL_ADC_Stop(&hadc1);
	/*	edit conversion from ADC to Time	*/
	uint8_t time = (uint8_t) (adc_value/10);
	return time;

}

void print_OLED(OLED_type OLED, char* string)
{
	switch(OLED)
	{
	case OLED1:
		ssd1306_SetCursor(0, 0);
		  ssd1306_WriteString("Blue Player", Font_11x18, White);

		  ssd1306_SetCursor(0, 36);
		  ssd1306_WriteString(string, Font_11x18, White);

		  ssd1306_UpdateScreen(&hi2c1,OLED1_ADD);
		break;

	case OLED2:
		ssd1306_SetCursor(0, 0);
		ssd1306_WriteString("Red Player", Font_11x18, White);

		ssd1306_SetCursor(0, 36);
		ssd1306_WriteString(string, Font_11x18, White);

		ssd1306_UpdateScreen(&hi2c1,OLED2_ADD);
		break;

	case OLED3:
		ssd1306_SetCursor(0, 0);
		ssd1306_WriteString("Green Player", Font_11x18, White);

		ssd1306_SetCursor(0, 36);
		ssd1306_WriteString(string, Font_11x18, White);

		ssd1306_UpdateScreen(&hi2c2,OLED1_ADD);
		break;
	case OLED4:
		ssd1306_SetCursor(0, 0);
		ssd1306_WriteString("Yellow Player", Font_11x18, White);

		ssd1306_SetCursor(0, 36);
		ssd1306_WriteString(string, Font_11x18, White);

		ssd1306_UpdateScreen(&hi2c2,OLED2_ADD);
		break;

	default:
		break;

	}

}

void Start_Game(void)
{
	vTaskSuspend(GamePlayHandle);
	char *ttl = "To skip Tutorial, Press Start";
		  	clearLCD();
		  	setCursor(0,0);
		  	writeLCD(ttl);

		    Start_Time = 5;
		    State=Waiting;
		    while(State ==Waiting)
		    {
		    	if (HAL_GPIO_ReadPin(Button_Start_GPIO_Port, Button_Start_Pin))
		    	{
		    		State = Interrupted;
		    	}
		    }

		    if (State ==Interrupted)
		    {

		    }
		    else
		    {
		    	clearLCD();
		    	ttl = "Explain Game";
		    	writeLCD(ttl);

		    	Start_Time = 5;
		    	State = Waiting;
		    	while(State == Waiting)
		    	{
		    		if (HAL_GPIO_ReadPin(Button_Start_GPIO_Port, Button_Start_Pin))
		    		{
		    			State = Interrupted;
		    		}
		    	}


		    }

	clearLCD();
	writeLCD("Set Game Time");
	setCursor(10,1);
	writeLCD("Start");
	while(!HAL_GPIO_ReadPin(Button_Start_GPIO_Port, Button_Start_Pin))
	{
		setCursor(0,1);
		char Str_Buffer[6];
		sprintf(Str_Buffer, "%d", get_timeIn());
		Str_Buffer[5]='\0';
		writeLCD(Str_Buffer);
	}

	Start_Time=get_timeIn();
	vTaskResume(GamePlayHandle);

}

void End_Game(void)
{
	vTaskSuspend(LCDHandle);

	uint32_t max_score =0;
	Start_Time =-1;
	State = Interrupted;

	for (uint8_t player_num=0;player_num < 4 ; player_num++)
	{
		if(max_score > Score_Player[player_num])
		{
			max_score = Score_Player[player_num];
		}

	}

	clearLCD();

	if(max_score>0)
	{
		writeLCD("Winner:");
		setCursor(0,1);
		for (uint8_t player_num=0;player_num < 4 ; player_num++)
			{
				if(max_score == Score_Player[player_num])
				{
					print_OLED(player_num, "Winner!");

					writeLCD(Colors[player_num]);
					writeLCD(" ");

				}
				else
				{
					print_OLED(player_num, "Game Over");
				}

			}
	}
	else{
		for (uint8_t player_num=0;player_num < 4 ; player_num++)
					{

							print_OLED(player_num, "Game Over");

					}
		writeLCD("No Winners");
	}



	Start_Time = 15;
	State=Waiting;
	while(State ==Waiting)
	{
		if (HAL_GPIO_ReadPin(Button_Start_GPIO_Port, Button_Start_Pin))
		{
			State = Interrupted;
			Start_Time=-1;
		}
	}
	vTaskResume(LCDHandle);


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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start(&hadc1);
  DF_Init(10);
  initLCD();
  // Init lcd using one of the stm32HAL i2c typedefs
  if (ssd1306_Init(&hi2c1,OLED1_ADD) != 0) {
    Error_Handler();
  }

  if (ssd1306_Init(&hi2c1,OLED2_ADD) != 0) {
      Error_Handler();
    }

  if (ssd1306_Init(&hi2c2,OLED1_ADD) != 0) {
      Error_Handler();
    }

  if (ssd1306_Init(&hi2c2,OLED2_ADD) != 0) {
      Error_Handler();
    }
  HAL_Delay(1000);

  ssd1306_Fill(Black);
  ssd1306_UpdateScreen(&hi2c1,OLED1_ADD);

  ssd1306_Fill(Black);
  ssd1306_UpdateScreen(&hi2c1,OLED2_ADD);

  ssd1306_Fill(Black);
  ssd1306_UpdateScreen(&hi2c2,OLED1_ADD);

  ssd1306_Fill(Black);
  ssd1306_UpdateScreen(&hi2c2,OLED2_ADD);

  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of OLED1_sem */
  osSemaphoreDef(OLED1_sem);
  OLED1_semHandle = osSemaphoreCreate(osSemaphore(OLED1_sem), 1);

  /* definition and creation of OLED2_sem */
  osSemaphoreDef(OLED2_sem);
  OLED2_semHandle = osSemaphoreCreate(osSemaphore(OLED2_sem), 1);

  /* definition and creation of OLED3_sem */
  osSemaphoreDef(OLED3_sem);
  OLED3_semHandle = osSemaphoreCreate(osSemaphore(OLED3_sem), 1);

  /* definition and creation of OLED4_sem */
  osSemaphoreDef(OLED4_sem);
  OLED4_semHandle = osSemaphoreCreate(osSemaphore(OLED4_sem), 1);

  /* definition and creation of Music_sem */
  osSemaphoreDef(Music_sem);
  Music_semHandle = osSemaphoreCreate(osSemaphore(Music_sem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of LCD */
  osThreadDef(LCD, LCD_Function, osPriorityRealtime, 0, 128);
  LCDHandle = osThreadCreate(osThread(LCD), NULL);

  /* definition and creation of Play_Sound */
  osThreadDef(Play_Sound, Sound_Function, osPriorityNormal, 0, 128);
  Play_SoundHandle = osThreadCreate(osThread(Play_Sound), NULL);

  /* definition and creation of GamePlay */
  osThreadDef(GamePlay, Game, osPriorityBelowNormal, 0, 128);
  GamePlayHandle = osThreadCreate(osThread(GamePlay), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xffff-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Twizzer_1_Pin|Twizzer_2_Pin|Twizzer_3_Pin|Twizzer_4_Pin
                          |LCD_EN_Pin|LCD_RS_Pin|Plate_7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin|LCD_D8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Plate_11_Pin Plate_12_Pin Plate_13_Pin */
  GPIO_InitStruct.Pin = Plate_11_Pin|Plate_12_Pin|Plate_13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Plate_1_Pin Plate_2_Pin Plate_3_Pin Plate_6_Pin
                           Plate_5_Pin Plate_4_Pin */
  GPIO_InitStruct.Pin = Plate_1_Pin|Plate_2_Pin|Plate_3_Pin|Plate_6_Pin
                          |Plate_5_Pin|Plate_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Twizzer_1_Pin Twizzer_2_Pin Twizzer_3_Pin Twizzer_4_Pin
                           LCD_EN_Pin LCD_RS_Pin Plate_7_Pin */
  GPIO_InitStruct.Pin = Twizzer_1_Pin|Twizzer_2_Pin|Twizzer_3_Pin|Twizzer_4_Pin
                          |LCD_EN_Pin|LCD_RS_Pin|Plate_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Button_Start_Pin Buzzer_Event_Pin */
  GPIO_InitStruct.Pin = Button_Start_Pin|Buzzer_Event_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_Reset_Pin */
  GPIO_InitStruct.Pin = Button_Reset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_Reset_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_D5_Pin LCD_D6_Pin LCD_D7_Pin LCD_D8_Pin */
  GPIO_InitStruct.Pin = LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin|LCD_D8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Plate_8_Pin Plate_9_Pin Plate_10_Pin */
  GPIO_InitStruct.Pin = Plate_8_Pin|Plate_9_Pin|Plate_10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_LCD_Function */
/**
* @brief Function implementing the LCD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LCD_Function */
void LCD_Function(void const * argument)
{
  /* USER CODE BEGIN 5 */

	Start_Game();
  /* Infinite loop */
  for(;;)
  {
	  if(Start_Time >0)
	  {
		  setCursor(0,1);
		  writeLCD("                ");
		  setCursor(0,1);
		  char Str_Buffer[6];
		  sprintf(Str_Buffer, "%d", Start_Time);
		  Str_Buffer[5]='\0';
		  writeLCD(Str_Buffer);
	  }
	  else if (Start_Time == -1)
	  {
		  Start_Game();
	  }

	  osDelay(400);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Sound_Function */
/**
* @brief Function implementing the Play_Sound thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Sound_Function */
void Sound_Function(void const * argument)
{
  /* USER CODE BEGIN Sound_Function */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Sound_Function */
}

/* USER CODE BEGIN Header_Game */
/**
* @brief Function implementing the GamePlay thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Game */
void Game(void const * argument)
{
  /* USER CODE BEGIN Game */
	uint8_t value =0;

	/* Buffer to hold the string	*/
	char str[6];


  /* Infinite loop */
  for(;;)
  {

	  if(Start_Time >0)
	  {
		  for (uint8_t Twizzer =0;Twizzer<4;Twizzer++)
		  {
			  HAL_GPIO_WritePin(GPIOA, Twizzer_1_Pin|Twizzer_2_Pin|Twizzer_3_Pin|Twizzer_4_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOA,Twizzers_Pins[Twizzer],GPIO_PIN_SET);

			  HAL_Delay(5);		/* Till Twizzer is Charged */

			  for(uint8_t plate=0;plate<13;plate++)
			  {
				  value=HAL_GPIO_ReadPin(Input_Position[plate].In_Port,Input_Position[plate].In_Pin);
				  if (value)
				  {
					  Score_Player[Twizzer] += Score_Plate[plate];
					  if(Score_Plate[plate])
					  {
						  PlatesCounter++;
						  if (PlatesCounter >=13)
						  {
							  End_Game();
							  break;
						  }
					  }
					  Score_Plate[plate]=0;

					  if (State == Timeout)
					  {
						  End_Game();
						  break;
					  }

					  /* Converting integer to string using sprintf	*/
					  sprintf(str, "%d", Score_Player[Twizzer]);

					  str[5]='\0';

					  print_OLED(Twizzer, str);


				  }
			  }
			  HAL_GPIO_WritePin(GPIOA, Twizzer_1_Pin|Twizzer_2_Pin|Twizzer_3_Pin|Twizzer_4_Pin, GPIO_PIN_RESET);
		  }
	  }
    osDelay(200);
  }
  /* USER CODE END Game */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
