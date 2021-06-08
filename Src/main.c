/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "Lcd.h"
#include "LcdConfig.h"
#include <stdlib.h>
#include <ctype.h>
#include<stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define MY_SLAVE_ID  0x01

#define TRUE   1
#define FALSE  0

#define START 1
#define STOP 0

#define AUTO_MODE 1
#define HANDLE_MODE 0


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

//volatile uint8_t temp_step;

volatile uint8_t DataPos;

uint8_t CommandIsReceived = FALSE;

stepMotor_typeDef stepMotor1, stepMotor2;
int display_coordX = 0, display_coordY =0 , display_speed_ref = 0;
uint16_t Xk =0 , Yk = 0;

G_CODE_typeDef G_Code;

uint8_t AllGcodeIsReceived = FALSE;

volatile uint16_t G_Code_File[5000];			// Save all G-Code Strings Received

volatile uint8_t countG;

volatile int sensor_X, sensor_Y;	//*******sensor value***

typedef enum{
	None = 0,
	Clockwise = 1,
	AntiClockwise = 2

}dirMotor_typedef;

volatile dirMotor_typedef sensor_DirMotorX, sensor_DirMotorY;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

void Lcd_display_parameter(Lcd_HandleTypeDef* lcd);
void Process_GCode(volatile uint16_t* gcode);
void Start_Motors_ByGCode();
void runMotor(stepMotor_typeDef* motor, uint16_t steps, uint8_t dir);


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

	sensor_DirMotorX = None;
	sensor_DirMotorY = None;

	//*********** Set first-Mode: handle Mode **********
	MODE = 0;
	isDrilling = 0;
	temp_coordX = 0;			//this is used while parsing G-code
 	temp_coordY = 0;			//this is used while parsing G-code
	temp_speed  = 0;
	sensor_X = 0;
	sensor_Y = 0;
	countG = 0;

	memset(data_in, 0,300);


	//********* Configuring for motors ***************
	stepMotor1.Port 		= GPIOA;
	stepMotor1.pin1 		= GPIO_PIN_4;
	stepMotor1.pin2 		= GPIO_PIN_5;
	stepMotor1.pin3 		= GPIO_PIN_6;
	stepMotor1.pin4 		= GPIO_PIN_7;
	stepMotor1.temp_step 	= 0;
	stepMotor1.state 		= NOT_RUNNING;

	stepMotor2.Port 		= GPIOB;
	stepMotor2.pin1 		= GPIO_PIN_8;
	stepMotor2.pin2 		= GPIO_PIN_9;
	stepMotor2.pin3 		= GPIO_PIN_6;
	stepMotor2.pin4 		= GPIO_PIN_0;
	stepMotor2.temp_step 	= 0;
	stepMotor2.state		= NOT_RUNNING;


	//**************** Set G_Code_String in Mobus model ************
	test[0] = (uint16_t*)(&motor);
	test[1] = (uint16_t*)(&stepNumb);
	test[2] = (uint16_t*)(&speed);
	test[3] = (uint16_t*)(&direction);
	test[4] = (uint16_t*)(&startStop);
	test[5] = (uint16_t*)(&MODE);


	test[6] = (uint16_t*)(&AllGcodeIsReceived);

	for (int i = 7; i<107; i++)
	{
		test[i] = (uint16_t*)(&G_Code_String[i-7]);
	}

	test[107] = (uint16_t*)&sensor_X;
	test[108] = (uint16_t*)&sensor_Y;
	test[109] = (uint16_t*)&isDrilling;
	test[110] = (uint16_t*)&sensor_DirMotorX;
	test[111] = (uint16_t*)&sensor_DirMotorY;


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
  MX_TIM2_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim3);

  HAL_TIM_Base_Start_IT(&htim2);

  DataPos = 0;

  HAL_UART_Receive_IT(&huart6, (uint8_t*)data_in + DataPos, 1);

  //********* Initialize KeyPad *************
  KeyPad_Init();

  //********* Initialize LCD ****************
  Lcd_PortType ports[] = { D4_GPIO_Port, D5_GPIO_Port, D6_GPIO_Port, D7_GPIO_Port};

  Lcd_PinType pins[] = {D4_Pin, D5_Pin, D6_Pin, D7_Pin};

  Lcd_HandleTypeDef lcd;

  lcd = Lcd_create(ports, pins, RS_GPIO_Port, RS_Pin, EN_GPIO_Port, EN_Pin, LCD_4_BIT_MODE);

  Lcd_string(&lcd, "Handle Mode");

  Lcd_cursor(&lcd, 1, 0);

  Lcd_string(&lcd, "Press '#' to run");


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(MODE == AUTO_MODE)
	  {
		  // ****************** Automatic mode (Using data from PC ) **************
		  if (CommandIsReceived == TRUE)
		  {
			  if (MBRegisterCount()>=2)			// Is Data received G-Code?
			  {

				  if (AllGcodeIsReceived == TRUE)
				  {
					  for (int i = 0; i < MBRegisterCount() -2 ; i++)
					  {
						  G_Code_File[countG*100 + i] = G_Code_String[i];
					  }

					  Process_GCode(G_Code_File);
					  Start_Motors_ByGCode();
					  AllGcodeIsReceived == FALSE;
					  countG = 0;
					  memset(G_Code_File, 0, 5000);
					  memset(G_Code_String, 0, 200);
				  }
				  else
				  {
					  for (int i = 0; i < 100; i++)
					  {
						  G_Code_File[countG*100 + i] = G_Code_String[i];
					  }

					  countG++;
				  }

			  }
			  else
			  {

				  switch (motor)
				  {
				  case 1:
					  MODE = HANDLE_MODE;
					  if (htim3.State == HAL_TIM_STATE_READY)
						  HAL_TIM_Base_Start_IT(&htim3);
					  break;

				  case 2:
					  MODE = HANDLE_MODE;
					  if (htim4.State == HAL_TIM_STATE_READY)
						  HAL_TIM_Base_Start_IT(&htim4);
					  break;
				  }
			  }

			  CommandIsReceived = FALSE;
			  MODE = HANDLE_MODE;
		  }

	  }
	  else
	  {
		  //***************** Handle Mode (Using Keyboard) ************************

		  switch(KeyPad_WaitForKeyGetChar(0))
		  {
		  	  case '1':								// X+
		  		  display_coordX +=10;
		  		  if(display_coordX>100) display_coordX = 0;
		  		  Lcd_display_parameter(&lcd);
		  		  G_Code.G_Line[0].coordX = display_coordX*10;


		  		  break;

		  	  case '2':								// X-
		  		  display_coordX -= 10;
		  		  if(display_coordX<0) display_coordX=100;
		  		  Lcd_display_parameter(&lcd);
		  		  G_Code.G_Line[0].coordX = display_coordX*10;
		  		  break;


		  	  case '3':
		  		  runMotor(&stepMotor1,100, CLOCKWISE); 	// Let motor1 (X) run 100 steps clockwise and set current position X = 0
		  		  break;

		  	  case 'A':
		  		  runMotor(&stepMotor1,100, ANTICLOCKWISE); // Let motor1 (X) run 100 steps anti-clockwise and set current position X = 0
		  		  break;

		  	  case '4':								// Y+
		  		  display_coordY +=10;
		  		  if(display_coordY>100) display_coordY=0;
		  		  Lcd_display_parameter(&lcd);
		  		  G_Code.G_Line[0].coordY = display_coordY*10;
		  		  break;

		  	  case '5':								// Y-
		  		  display_coordY-= 10;
		  		  if(display_coordY<0) display_coordY=100;
		  		  Lcd_display_parameter(&lcd);
		  		  G_Code.G_Line[0].coordY = display_coordY*10;
		  		  break;

		  	  case '6':
		  		  runMotor(&stepMotor2,100, CLOCKWISE); 	// Let motor2 (Y) run 100 steps clockwise and set current position Y = 0
		  		  break;

		  	  case 'B':
		  		  runMotor(&stepMotor2,100, ANTICLOCKWISE); // Let motor2 (Y) run 100 steps anti-clockwise and set current position Y = 0
		  		  break;


		  	  case '7':								// F+
		  		  display_speed_ref +=10;
		  		  G_Code.G_Line[0].speed_ref += 1000 ;
		  		  if(display_speed_ref>50) display_speed_ref = 10;
		  		  Lcd_display_parameter(&lcd);
		  		  break;

		  	  case '8':								// F+
		  		  display_speed_ref -=10;
		  		  G_Code.G_Line[0].speed_ref--;
		  		  if(display_speed_ref< 10) display_speed_ref=50;
		  		  Lcd_display_parameter(&lcd);
		  		  break;

		  	  case '9':
		  		  break;

		  	  case '*':
		  		  Lcd_clear(&lcd);
		  		  Lcd_cursor(&lcd, 0, 4);
		  		  Lcd_string(&lcd, "Running... ");
		  		  G_Code.totalNumbOfLineGCode = 1;
		  		  Start_Motors_ByGCode();
		  		  while ((stepMotor1.state == RUNNING)||(stepMotor2.state == RUNNING)){};
		  		  Lcd_clear(&lcd);
		  		  Lcd_cursor(&lcd, 0, 4);
		  		  Lcd_string(&lcd, "FINISH");
		  		  Lcd_cursor(&lcd, 1, 0);
		  		  Lcd_string(&lcd, "Press 0 to exit");
		  		  break;

		  	  case '0':
		  		  Lcd_clear(&lcd);
		  		  Lcd_string(&lcd, "Handle Mode");
		  		  Lcd_cursor(&lcd, 1, 0);
		  		  Lcd_string(&lcd, "Press '#' to run");
		  		  break;

		  	  case '#':
		  		  Xk = 0;
		  		  Yk = 0;
		  		  temp_coordX = 0;
		  		  temp_coordY = 0;
		  		  Lcd_display_parameter(&lcd);
		  		  break;

		  	  case 'D':
		  		  break;
		  }

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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 4999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 599;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
  htim3.Init.Prescaler = 499;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 499;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_DB5_Pin|Motor1_Pin1_Pin|Motor1_Pin2_Pin|Motor1_Pin3_Pin 
                          |Motor1_Pin4_Pin|KeyBoard_8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LCD_RS_Pin|LCD_EN_Pin|LCD_DB7_Pin|LCD_DB6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Motor2_Pin4_Pin|KeyBoard_7_Pin|KeyBoard_5_Pin|KeyBoard_6_Pin 
                          |Motor2_Pin3_Pin|LCD_DB4_Pin|Motor2_Pin1_Pin|Motor2_Pin2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_DB5_Pin Motor1_Pin1_Pin Motor1_Pin2_Pin Motor1_Pin3_Pin 
                           Motor1_Pin4_Pin KeyBoard_8_Pin */
  GPIO_InitStruct.Pin = LCD_DB5_Pin|Motor1_Pin1_Pin|Motor1_Pin2_Pin|Motor1_Pin3_Pin 
                          |Motor1_Pin4_Pin|KeyBoard_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RS_Pin LCD_EN_Pin LCD_DB7_Pin LCD_DB6_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin|LCD_EN_Pin|LCD_DB7_Pin|LCD_DB6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Motor2_Pin4_Pin KeyBoard_7_Pin KeyBoard_5_Pin KeyBoard_6_Pin 
                           Motor2_Pin3_Pin LCD_DB4_Pin Motor2_Pin1_Pin Motor2_Pin2_Pin */
  GPIO_InitStruct.Pin = Motor2_Pin4_Pin|KeyBoard_7_Pin|KeyBoard_5_Pin|KeyBoard_6_Pin 
                          |Motor2_Pin3_Pin|LCD_DB4_Pin|Motor2_Pin1_Pin|Motor2_Pin2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : KeyBoard_2_Pin KeyBoard_1_Pin KeyBoard_4_Pin */
  GPIO_InitStruct.Pin = KeyBoard_2_Pin|KeyBoard_1_Pin|KeyBoard_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : KeyBoard_3_Pin */
  GPIO_InitStruct.Pin = KeyBoard_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KeyBoard_3_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if (htim2.State == HAL_TIM_STATE_BUSY)
		HAL_TIM_Base_Stop_IT(&htim2);

//	__HAL_TIM_SET_COUNTER(&htim2, 0);

	TIM2->SR &= ~1;

	if (htim2.State == HAL_TIM_STATE_READY)
		HAL_TIM_Base_Start_IT(&htim2);

	DataPos++;

	HAL_UART_Receive_IT(&huart6, (uint8_t*)data_in + DataPos, 1);

}


 void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if (htim->Instance == TIM2)
	{
		HAL_TIM_Base_Stop_IT(&htim2);

		if(data_in[0]== MY_SLAVE_ID)
		{

			switch(data_in[1])
			{
				case 0x03:
				{
					MBProcessRegisters(0x03);
				}
				break;

				case 0x06:
				{
					WriteToRegister(test);
				}
				break;

				case 0x10:
				{
					WriteToMultilRegisters(test);
					CommandIsReceived = TRUE;

					  switch (motor)
					  {
					  	  case 1:
					  		stepMotor1.stepNumb  = stepNumb;
					  		stepMotor1.speed 	 = speed;
					  		stepMotor1.direction = direction;
					  		stepMotor1.startStop = startStop;
					  		__HAL_TIM_SET_AUTORELOAD(&htim3, stepMotor1.speed*100);
						  break;

					  	  case 2:
							stepMotor2.stepNumb  = stepNumb;
							stepMotor2.speed     = speed;
							stepMotor2.direction = direction;
							stepMotor2.startStop = startStop;
							__HAL_TIM_SET_AUTORELOAD(&htim4, stepMotor2.speed*100);
						  break;

					  	  default:
					  		  break;
					  }

				}
				break;


				default:
				{
					MBException(0x01); //Illegal function code 01
					MBSendData(ResponseFrameSize);
				}
				break;
			}


		}

		DataPos = 0;

		HAL_UART_Receive_IT(&huart6, (uint8_t*)data_in + DataPos, 1);
	}



	//**************This part is used to Control motor*************************

	else if (htim->Instance == TIM3)					//Interrupt from Timer 3 (Control motor1)
	{
		if (stepMotor1.startStop == 1)
		{
			if (MODE == HANDLE_MODE)
			{
				OneStep(&stepMotor1);
				stepMotor1.stepNumb--;
				sensor_X += (stepMotor1.direction == CLOCKWISE)?(1):(-1);
				if (sensor_X <0 ) sensor_X = 0;
				if (stepMotor1.stepNumb == 0)
				{
					stepMotor1.startStop = 0;
					stepMotor1.state = NOT_RUNNING;   		//Motor finishes its own work
				}
			}
			else if (MODE == AUTO_MODE)
			{
				if ( (stepMotor1.stepNumb == 0) || (sensor_X == Xk) || (Xk == temp_coordX))
				{
					stepMotor1.startStop = 0;
					stepMotor1.state = NOT_RUNNING;   		//Motor finishes its own work
				}

				else if ( abs((sensor_X - temp_coordX)*(Yk -temp_coordY)) <= abs((Xk - temp_coordX)*(sensor_Y -temp_coordY)))
				{

					OneStep(&stepMotor1);
					stepMotor1.stepNumb--;
					sensor_X += (stepMotor1.direction == CLOCKWISE)?(1):(-1);
					if (sensor_X <0 ) sensor_X = 0;
				}

			}


		}
		else
		{
			HAL_GPIO_WritePin(stepMotor1.Port, stepMotor1.pin1, 0);
			HAL_GPIO_WritePin(stepMotor1.Port, stepMotor1.pin2, 0);
			HAL_GPIO_WritePin(stepMotor1.Port, stepMotor1.pin3, 0);
			HAL_GPIO_WritePin(stepMotor1.Port, stepMotor1.pin4, 0);
			HAL_TIM_Base_Stop_IT(&htim3);
		}
	}

	else if (htim->Instance == TIM4)					//Interrupt from Timer 4 (Control motor2)
	{
		if (stepMotor2.startStop == 1)
		{
			if (MODE == HANDLE_MODE)
			{
				OneStep(&stepMotor2);
				stepMotor2.stepNumb--;
				sensor_Y += (stepMotor2.direction == CLOCKWISE)?(1):(-1);
				if (sensor_Y <0 ) sensor_Y = 0;
				if (stepMotor2.stepNumb == 0)
				{
					stepMotor2.startStop = 0;
					stepMotor2.state = NOT_RUNNING;			//Motor finishes its own work
				}
			}
			else if (MODE == AUTO_MODE)
			{

				if ( (stepMotor2.stepNumb == 0) || (sensor_Y == Yk) || (Yk == temp_coordY))
				{
					stepMotor2.startStop = 0;
					stepMotor2.state = NOT_RUNNING;			//Motor finishes its own work
				}
				else if ( abs((sensor_X - temp_coordX)*(Yk -temp_coordY)) >= abs((Xk - temp_coordX)*(sensor_Y -temp_coordY)))
				{
					OneStep(&stepMotor2);
					stepMotor2.stepNumb--;
					sensor_Y += (stepMotor2.direction == CLOCKWISE)?(1):(-1);
					if (sensor_Y <0 ) sensor_Y = 0;
				}




			}
		}
		else
		{
			HAL_GPIO_WritePin(stepMotor2.Port, stepMotor2.pin1, 0);
			HAL_GPIO_WritePin(stepMotor2.Port, stepMotor2.pin2, 0);
			HAL_GPIO_WritePin(stepMotor2.Port, stepMotor2.pin3, 0);
			HAL_GPIO_WritePin(stepMotor2.Port, stepMotor2.pin4, 0);
			HAL_TIM_Base_Stop_IT(&htim4);
		}

	}

}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (isDrilling == 1){
		sensor_DirMotorX = None;
		sensor_DirMotorY = None;
		return;
	}

	if (GPIO_Pin == GPIO_PIN_0){

		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == 1){
			sensor_DirMotorX = None;
		}else{
			sensor_DirMotorX = None;
		}

	}
	else if (GPIO_Pin == GPIO_PIN_14)
	{

		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13 == 1)){
			sensor_DirMotorY = AntiClockwise;
		}else{
			sensor_DirMotorY = Clockwise;
		}
	}
}

 void delay_ms(uint16_t ms)
 {
	 for (int i = 0; i< ms ;i++)
	 {
		 __HAL_TIM_SET_COUNTER(&htim3,0);
		 while(__HAL_TIM_GET_COUNTER(&htim3) < 16000);
	 }
 }


 void Lcd_display_parameter(Lcd_HandleTypeDef* lcd)
 {
	  Lcd_clear(lcd);
	  Lcd_cursor(lcd, 0, 6);
	  Lcd_string(lcd, "G01 ");

	  Lcd_cursor(lcd, 1, 0);
	  Lcd_string(lcd, "X");
	  Lcd_int(lcd, (int)display_coordX/100);
	  Lcd_int(lcd, (int)(display_coordX/10)%10);
	  Lcd_int(lcd, (int)display_coordX%10);
	  Lcd_string(lcd, " ");

	  Lcd_string(lcd, "Y");
	  Lcd_int(lcd, (int)display_coordY/100);
	  Lcd_int(lcd, (int)(display_coordY/10)%10);
	  Lcd_int(lcd, (int)display_coordY%10);
	  Lcd_string(lcd, " ");

	  Lcd_string(lcd, "F");
	  Lcd_int(lcd, (int)display_speed_ref/100);
	  Lcd_int(lcd, (int)display_speed_ref/10);
	  Lcd_int(lcd, (int)display_speed_ref%10);
 }

 void Process_GCode(volatile uint16_t* g_code_string)
 {
	 char s[5000];
	 G_Code.totalNumbOfLineGCode = 0;
	 for (int i= 0; i< 5000;i++)
	 {
		 if(g_code_string[i] == 'G')
		 {
			 ++G_Code.totalNumbOfLineGCode;
		 }

		 s[i] = (char)g_code_string[i];

	 }

	 for (int i = 0; i<G_Code.totalNumbOfLineGCode; i++)
	 {
		 float i1, i2, i3, i4;
		 char buff_s[5000];
		 memset(buff_s,0,5000);
		 sscanf(s, "%*[^0123456789]%f %*[^0123456789]%f %*[^0123456789]%f %*[^0123456789]%f %[^\0]",
		                 &i1,
		                 &i2,
		                 &i3,
		 				 &i4,
						 buff_s);

		 for (int i=0; i<sizeof(buff_s); i++)
		 {
			 s[i] = buff_s[i];
		 }

		 G_Code.G_Line[i].coordX     = (uint16_t)i2;
		 G_Code.G_Line[i].coordY 	 = (uint16_t)i3;
		 G_Code.G_Line[i].speed_ref  = (uint16_t)i4;
	 }

 }

 void Start_Motors_ByGCode()
 {

	 for (int i=0; i<G_Code.totalNumbOfLineGCode; i++)
	 {

		 if (stepMotor1.state == NOT_RUNNING)
		 {
			stepMotor1.state = RUNNING;
			Xk = G_Code.G_Line[i].coordX*10;
			stepMotor1.stepNumb  = (Xk == temp_coordX)? 0: abs((Xk - temp_coordX));   		//multiple by 10
			stepMotor1.speed     = (uint8_t)(G_Code.G_Line[i].speed_ref/1000);
			stepMotor1.direction = (Xk >= temp_coordX)? CLOCKWISE : ANTICLOCKWISE;
			stepMotor1.startStop = 1;			//make it run
			__HAL_TIM_SET_AUTORELOAD(&htim3, stepMotor1.speed*100);

			 if (htim3.State == HAL_TIM_STATE_READY) 		//Start timer3 to control motor1
			 {
				HAL_TIM_Base_Start_IT(&htim3);
			 }

		 }

		 if (stepMotor2.state == NOT_RUNNING)
		 {
			stepMotor2.state = RUNNING;
			Yk = G_Code.G_Line[i].coordY*10;
			stepMotor2.stepNumb  = (Yk == temp_coordY)? 0: abs((Yk - temp_coordY));
			stepMotor2.speed     = (uint8_t)(G_Code.G_Line[i].speed_ref/1000);
			stepMotor2.direction = (Yk >= temp_coordY)?CLOCKWISE : ANTICLOCKWISE;
			stepMotor2.startStop = 1; 			//make it run
			__HAL_TIM_SET_AUTORELOAD(&htim4, stepMotor2.speed*100);


			  if (htim4.State == HAL_TIM_STATE_READY)		//Start timer4 to control motor2
			  {
				  HAL_TIM_Base_Start_IT(&htim4);
			  }

		 }

		 while ((stepMotor1.state == RUNNING)||(stepMotor2.state == RUNNING)){};
		 temp_coordX = Xk;
		 temp_coordY = Yk;

		 if (i!= G_Code.totalNumbOfLineGCode -1)
		 {
			 isDrilling = 1;
			 HAL_Delay(1000);						// Do anything desired
			 isDrilling = 0;
			 HAL_Delay(500);
		 }
	 }

 }

 void runMotor(stepMotor_typeDef* motor, uint16_t steps, uint8_t dir)
 {

		 while ((motor->state == RUNNING)){};
		 if (motor->state == NOT_RUNNING)
		 {
			 motor->stepNumb  = steps ;
			 motor->speed     = 2;
			 motor->direction = dir;
			 motor->startStop = 1;			//make it run


			 if (motor == &stepMotor1)
			 {
				__HAL_TIM_SET_AUTORELOAD(&htim3, stepMotor1.speed*100);
				temp_coordX = 0;

				 if (htim3.State == HAL_TIM_STATE_READY) 		//Start timer3 to control motor1
				 {
					HAL_TIM_Base_Start_IT(&htim3);
				 }
				 stepMotor1.state = RUNNING;

			 }
			 else if (motor == &stepMotor2 )
			 {
				__HAL_TIM_SET_AUTORELOAD(&htim4, stepMotor2.speed*100);
				temp_coordY = 0;

				  if (htim4.State == HAL_TIM_STATE_READY)		//Start timer4 to control motor2
				  {
					  HAL_TIM_Base_Start_IT(&htim4);
				  }
				  stepMotor2.state = RUNNING;
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
