#ifndef INC_LCD_CONFIG_H_
#define INC_LCD_CONFIG_H_

#include "main.h"

/* CONFIG FOR LIBRARY USER */
#define GPIO_PORT		LCD_D5_GPIO_Port
#define GPIO_CMD_PORT	LCD_EN_GPIO_Port
//4 pin mode -> pins
#define DATA5_Pin LCD_D5_Pin
#define DATA6_Pin LCD_D6_Pin
#define DATA7_Pin LCD_D7_Pin
#define DATA8_Pin LCD_D8_Pin

#define RS_Pin LCD_EN_Pin
#define E_Pin  LCD_RS_Pin
//RW Pin not used,connect to GND

//if you want to work with 8 bit mode uncomment the area which is given below

/*
#define LCD8Bit
#define DATA1_Pin GPIO_PIN_1
#define DATA2_Pin GPIO_PIN_2
#define DATA3_Pin GPIO_PIN_3
#define DATA4_Pin GPIO_PIN_4
*/


#endif /* INC_LCD_CONFIG_H_ */
