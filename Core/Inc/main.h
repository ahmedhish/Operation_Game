/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Plate_11_Pin GPIO_PIN_13
#define Plate_11_GPIO_Port GPIOC
#define Plate_12_Pin GPIO_PIN_14
#define Plate_12_GPIO_Port GPIOC
#define Plate_13_Pin GPIO_PIN_15
#define Plate_13_GPIO_Port GPIOC
#define Plate_1_Pin GPIO_PIN_0
#define Plate_1_GPIO_Port GPIOA
#define Plate_2_Pin GPIO_PIN_1
#define Plate_2_GPIO_Port GPIOA
#define Plate_3_Pin GPIO_PIN_2
#define Plate_3_GPIO_Port GPIOA
#define Twizzer_1_Pin GPIO_PIN_3
#define Twizzer_1_GPIO_Port GPIOA
#define Twizzer_2_Pin GPIO_PIN_4
#define Twizzer_2_GPIO_Port GPIOA
#define Twizzer_3_Pin GPIO_PIN_5
#define Twizzer_3_GPIO_Port GPIOA
#define Twizzer_4_Pin GPIO_PIN_6
#define Twizzer_4_GPIO_Port GPIOA
#define Slide_Pot_Pin GPIO_PIN_7
#define Slide_Pot_GPIO_Port GPIOA
#define Button_Start_Pin GPIO_PIN_0
#define Button_Start_GPIO_Port GPIOB
#define Button_Reset_Pin GPIO_PIN_1
#define Button_Reset_GPIO_Port GPIOB
#define LCD_D5_Pin GPIO_PIN_12
#define LCD_D5_GPIO_Port GPIOB
#define LCD_D6_Pin GPIO_PIN_13
#define LCD_D6_GPIO_Port GPIOB
#define LCD_D7_Pin GPIO_PIN_14
#define LCD_D7_GPIO_Port GPIOB
#define LCD_D8_Pin GPIO_PIN_15
#define LCD_D8_GPIO_Port GPIOB
#define LCD_EN_Pin GPIO_PIN_8
#define LCD_EN_GPIO_Port GPIOA
#define LCD_RS_Pin GPIO_PIN_9
#define LCD_RS_GPIO_Port GPIOA
#define Plate_6_Pin GPIO_PIN_10
#define Plate_6_GPIO_Port GPIOA
#define Plate_5_Pin GPIO_PIN_11
#define Plate_5_GPIO_Port GPIOA
#define Plate_4_Pin GPIO_PIN_12
#define Plate_4_GPIO_Port GPIOA
#define Plate_7_Pin GPIO_PIN_15
#define Plate_7_GPIO_Port GPIOA
#define Plate_8_Pin GPIO_PIN_3
#define Plate_8_GPIO_Port GPIOB
#define Plate_9_Pin GPIO_PIN_4
#define Plate_9_GPIO_Port GPIOB
#define Plate_10_Pin GPIO_PIN_5
#define Plate_10_GPIO_Port GPIOB
#define MP3_Tx_Pin GPIO_PIN_6
#define MP3_Tx_GPIO_Port GPIOB
#define Buzzer_Event_Pin GPIO_PIN_7
#define Buzzer_Event_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
