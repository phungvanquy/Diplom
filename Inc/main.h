/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */


typedef enum
{
	RUNNING,
	NOT_RUNNING
}Motor_State_TypeDef;
typedef struct
{
	volatile uint16_t temp_step;				//temporary step
	volatile uint16_t stepNumb;
	volatile uint16_t speed;
	volatile uint8_t direction;
	volatile uint8_t startStop;
	Motor_State_TypeDef	state;
	GPIO_TypeDef* Port;
	uint16_t pin1;
	uint16_t pin2;
	uint16_t pin3;
	uint16_t pin4;
}stepMotor_typeDef;




typedef struct
{
	uint16_t coordX;
	uint16_t coordY;
	uint16_t speed_ref;
}G_CODE_Line_typeDef;

typedef struct
{
	uint16_t totalNumbOfLineGCode;
	G_CODE_Line_typeDef G_Line[20];
}G_CODE_typeDef;


volatile uint16_t stepNumb;			//desired Step numbers
volatile uint16_t speed;			//desired speed of motor
volatile uint16_t direction;		//direction of motor
volatile uint16_t startStop;		//Start(stop) motor
volatile uint16_t motor;
volatile uint16_t G_Code_String[200];			//Save several G-code Strings Received and then added to G_Code_File
volatile uint16_t MODE;
volatile uint16_t temp_coordX, temp_coordY, temp_speed, isDrilling;
volatile uint8_t ResponseFrameSize;

//volatile uint16_t* test[302] ={(uint16_t*)(&motor),
//									 (uint16_t*)(&stepNumb),
//									 (uint16_t*)(&speed),
//									 (uint16_t*)(&direction),
//									 (uint16_t*)(&startStop),
//									 (uint16_t*)(&MODE)
//};

volatile uint16_t* test[110];





/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define MOTOR_SPEED_5msPerStep		4
#define MOTOR_SPEED_10msPerStep		10
#define MOTOR_SPEED_20msPerStep		20
#define MOTOR_SPEED_50msPerStep		50
#define MOTOR_SPEED_100msPerStep	100

#define CLOCKWISE 		1
#define ANTICLOCKWISE 	0

//#define MaxFrameIndex 255u
#define fc3_HoldingRegMax 5000u
#define fc3_HoldingRegOffset 40001u

#define HoldingRegSize 5000u

volatile uint8_t data_in[5000];

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void delay_ms(uint16_t ms);



/*----API for ModBus Communication ---------*/
uint16_t CRC16 (volatile uint8_t *puchMsg, uint16_t usDataLen );
uint16_t MBRegisterCount(void);
void AppendDatatoMBRegister(uint16_t StAddr,uint16_t count,volatile uint16_t **inreg, volatile uint8_t *outreg);
uint16_t MBStartAddress(void);
void MBSendData(uint8_t count);
void AppendCRCtoMBRegister(uint8_t packtop);
void MBException(uint8_t exceptionCode);
void MBProcessRegisters(uint8_t fcCode);
void AppendBitsToRegisters(uint16_t StAddr, uint16_t count, uint8_t *inreg, volatile uint8_t *outreg);
void WriteToRegister(volatile uint16_t** arr);
void WriteToMultilRegisters(volatile uint16_t** arr);

/*----API for Step Motor---------*/
void OneStep(stepMotor_typeDef* motor);




/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LCD_DB5_Pin GPIO_PIN_0
#define LCD_DB5_GPIO_Port GPIOA
#define Motor1_Pin1_Pin GPIO_PIN_4
#define Motor1_Pin1_GPIO_Port GPIOA
#define Motor1_Pin2_Pin GPIO_PIN_5
#define Motor1_Pin2_GPIO_Port GPIOA
#define Motor1_Pin3_Pin GPIO_PIN_6
#define Motor1_Pin3_GPIO_Port GPIOA
#define Motor1_Pin4_Pin GPIO_PIN_7
#define Motor1_Pin4_GPIO_Port GPIOA
#define LCD_RS_Pin GPIO_PIN_5
#define LCD_RS_GPIO_Port GPIOC
#define Motor2_Pin4_Pin GPIO_PIN_0
#define Motor2_Pin4_GPIO_Port GPIOB
#define KeyBoard_2_Pin GPIO_PIN_1
#define KeyBoard_2_GPIO_Port GPIOB
#define KeyBoard_1_Pin GPIO_PIN_2
#define KeyBoard_1_GPIO_Port GPIOB
#define KeyBoard_4_Pin GPIO_PIN_10
#define KeyBoard_4_GPIO_Port GPIOB
#define LCD_EN_Pin GPIO_PIN_8
#define LCD_EN_GPIO_Port GPIOC
#define KeyBoard_3_Pin GPIO_PIN_8
#define KeyBoard_3_GPIO_Port GPIOA
#define KeyBoard_8_Pin GPIO_PIN_10
#define KeyBoard_8_GPIO_Port GPIOA
#define LCD_DB7_Pin GPIO_PIN_10
#define LCD_DB7_GPIO_Port GPIOC
#define LCD_DB6_Pin GPIO_PIN_12
#define LCD_DB6_GPIO_Port GPIOC
#define KeyBoard_7_Pin GPIO_PIN_3
#define KeyBoard_7_GPIO_Port GPIOB
#define KeyBoard_5_Pin GPIO_PIN_4
#define KeyBoard_5_GPIO_Port GPIOB
#define KeyBoard_6_Pin GPIO_PIN_5
#define KeyBoard_6_GPIO_Port GPIOB
#define Motor2_Pin3_Pin GPIO_PIN_6
#define Motor2_Pin3_GPIO_Port GPIOB
#define LCD_DB4_Pin GPIO_PIN_7
#define LCD_DB4_GPIO_Port GPIOB
#define Motor2_Pin1_Pin GPIO_PIN_8
#define Motor2_Pin1_GPIO_Port GPIOB
#define Motor2_Pin2_Pin GPIO_PIN_9
#define Motor2_Pin2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
