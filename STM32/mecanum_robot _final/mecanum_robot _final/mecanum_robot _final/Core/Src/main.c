/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
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
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
uint8_t rx_data;
int rx_index = 0;
char rx_buffer[20];
char temp[20];
int check_var = 0;
//encoder variable
int pulse_counter1 = 0, pulse_counter2 = 0, pulse_counter3 = 0, pulse_counter4 = 0;

//velocity variable
float M_PI = 3.14;
float vx = 0, vy = 0, w_angle = 0;
float w_reference1_temp = 0, w_reference2_temp = 0, w_reference3_temp = 0, w_reference4_temp = 0; // v?n t?c b�nh xe (rad/s)
float w_reference1 = 0, w_reference2 = 0, w_reference3 = 0, w_reference4 = 0; // v?n t?c b�nh xe (rpm)
float w1 = 0, w2 = 0, w3 = 0, w4 = 0; // v?n t?c th?c t? c?a b�nh xe (rpm)
float e1 = 0, e2 = 0, e3 = 0, e4 = 0;
float pre_e1 = 0, pre_e2 = 0, pre_e3 = 0, pre_e4 = 0;
float const max_pwm = 1000;

//robot variable
float const r_wheel = 0.048;
float lx = 0.21919; // khoang cach tu tam banh xe den tam robot theo phuong x
float ly = 0.21647; // khoang cach tu banh xe den tam robot theo phuong y

//PID parameters
float kP = 1.2, kI = 0.08, kD = 0.002;
float pPart1 = 0, pPart2 = 0, pPart3 = 0, pPart4 = 0;
float iPart1 = 0, iPart2 = 0, iPart3 = 0, iPart4 = 0;
float dPart1 = 0, dPart2 = 0, dPart3 = 0, dPart4 = 0;
float PID_signal1 = 0, PID_signal2 = 0, PID_signal3 = 0, PID_signal4 = 0;
float signal_pos1 = 0, signal_neg1 = 0;
float signal_pos2 = 0, signal_neg2 = 0;
float signal_pos3 = 0, signal_neg3 = 0;
float signal_pos4 = 0, signal_neg4 = 0;

// Posion calcalate
float w1_rad = 0, w2_rad = 0, w3_rad = 0, w4_rad = 0;
float v_wheel1 = 0, v_wheel2 = 0, v_wheel3 = 0, v_wheel4 = 0;

// Sensor parameters
unsigned int flag = 0;
unsigned int Sensor1;
unsigned int Sensor2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM12_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Init_Motor1()
{

	////Enable H-bridge
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, 1);			//bridge1 EN_R
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, 1);			//bridge1 EN_L
	////Start PWM
	//PWM Motor 1
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);			//Motor1 +
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);			//Motor1 -

	__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2, 0);
}

void Init_Motor2()
{
	////Enable H-bridge
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 1);			//bridge1 EN_R
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, 1);			//bridge1 EN_L
	////Start PWM
	//PWM Motor 2
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);			//Motor2 +
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);			//Motor2 -

	__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_3, 0);
	__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_4, 0);

}

void Init_Motor3()
{
	////Enable H-bridge
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, 1);			//bridge1 EN_R
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, 1);			//bridge1 EN_L
	////Start PWM
	//PWM Motor 3
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);		//Motor3 +
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);		//Motor3 -

	__HAL_TIM_SetCompare(&htim12,TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim12,TIM_CHANNEL_2, 0);
}

void Init_Motor4()
{
	////Enable H-bridge
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);			//bridge1 EN_R
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15, 1);			//bridge1 EN_L
	////Start PWM
	//PWM Motor 4
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);			//Motor4 +
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);			//Motor4 -

	__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2, 0);
}

void check_w_ref()
{
	sprintf(temp, "vx_ref = %.2f\n", vx);
	HAL_UART_Transmit(&huart3, (uint8_t *)temp, strlen(temp),10);
	HAL_Delay(100);
	sprintf(temp, "vy_ref = %.2f\n", vy);
	HAL_UART_Transmit(&huart3, (uint8_t *)temp, strlen(temp),10);
	HAL_Delay(100);
	sprintf(temp, "w_ref = %.2f\n", w_angle);
	HAL_UART_Transmit(&huart3, (uint8_t *)temp, strlen(temp),10);
	HAL_Delay(100);
	HAL_Delay(2000);

	// sprintf(temp, "w1_temp = %.2f, w1_ref = %.2f\n", w_reference1_temp ,w_reference1);
	// HAL_UART_Transmit_DMA(&huart3, (uint8_t *)temp, strlen(temp));
	// HAL_Delay(100);
	// sprintf(temp, "w2_temp = %.2f, w2_ref = %.2f\n", w_reference2_temp ,w_reference2);
	// HAL_UART_Transmit_DMA(&huart3, (uint8_t *)temp, strlen(temp));
	// HAL_Delay(100);
	// sprintf(temp, "w3_temp = %.2f, w3_ref = %.2f\n", w_reference3_temp ,w_reference3);
	// HAL_UART_Transmit_DMA(&huart3, (uint8_t *)temp, strlen(temp));
	// HAL_Delay(100);
	// sprintf(temp, "w4_temp = %.2f, w4_ref = %.2f\n", w_reference4_temp ,w_reference4);
	// HAL_UART_Transmit_DMA(&huart3, (uint8_t *)temp, strlen(temp));
	// HAL_Delay(100);
	// HAL_Delay(2000);
}

// void signal_feedback_PID_parameter()
// {
// 	sprintf(temp, "P%.2fp\n", kP);
// 	HAL_UART_Transmit(&huart3, (uint8_t *)temp, strlen(temp), 10);
// 	HAL_Delay(100);
// 	sprintf(temp, "I%.2fi\n", kI);
// 	HAL_UART_Transmit(&huart3, (uint8_t *)temp, strlen(temp), 10);
// 	HAL_Delay(100);
// 	sprintf(temp, "D%.2fd\n", kD);
// 	HAL_UART_Transmit(&huart3, (uint8_t *)temp, strlen(temp), 10);
// 	HAL_Delay(2000);
// }

// void signal_feedback_w_wheel()
// {
// 	sprintf(temp, "w1 = %.2f rpm r\n", w1);
// 	HAL_UART_Transmit_DMA(&huart3, (uint8_t *)temp, strlen(temp), 10);
// 	HAL_Delay(100);
// 	sprintf(temp, "w2 = %.2f rpm r\n", w2);
// 	HAL_UART_Transmit_DMA(&huart3, (uint8_t *)temp, strlen(temp), 10);
// 	HAL_Delay(100);
// 	sprintf(temp, "w3 = %.2f rpm r\n", w3);
// 	HAL_UART_Transmit_DMA(&huart3, (uint8_t *)temp, strlen(temp), 10);
// 	HAL_Delay(100);
// 	sprintf(temp, "w4 = %.2f rpm r\n", w4);
// 	HAL_UART_Transmit_DMA(&huart3, (uint8_t *)temp, strlen(temp), 10);
// 	HAL_Delay(2000);
// }

// void signal_feedback_error()
// {
// 	sprintf(temp, "e1 = %.2f rpm r\n", e1);
// 	HAL_UART_Transmit(&huart3, (uint8_t *)temp, strlen(temp), 10);
// 	HAL_Delay(100);
// 	sprintf(temp, "e2 = %.2f rpm r\n", e2);
// 	HAL_UART_Transmit(&huart3, (uint8_t *)temp, strlen(temp), 10);
// 	HAL_Delay(100);
// 	sprintf(temp, "e3 = %.2f rpm r\n", e3);
// 	HAL_UART_Transmit(&huart3, (uint8_t *)temp, strlen(temp), 10);
// 	HAL_Delay(100);
// 	sprintf(temp, "e4 = %.2f rpm r\n", e4);
// 	HAL_UART_Transmit(&huart3, (uint8_t *)temp, strlen(temp), 10);
// 	HAL_Delay(2000);
// }

// void position_feedback()
// {
// }

void sensorCheck()
{	
		Sensor1 = HAL_GPIO_ReadPin(Sensor_1_GPIO_Port, Sensor1);
		Sensor2 = HAL_GPIO_ReadPin(Sensor_2_GPIO_Port, Sensor2);
    if ((Sensor1 == 1 && Sensor2 == 0))
    {
      flag =1;
			
    }
		else if((Sensor1 == 0 && Sensor2 == 1) ){
			
			flag = 0;
		
		}
    if (flag == 1)
    {
      HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, 1);
			
    }
    else
    {

      HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, 0);
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
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim4);
  ////Start UART Receive interupt
	HAL_UART_Receive_DMA(&huart3, &rx_data, 1);

  Init_Motor1();
  Init_Motor2();
  Init_Motor3();
  Init_Motor4();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	check_var = 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	sensorCheck();
  if(check_var == 0)
	  {
		  check_w_ref();
	  }
	//   if(check_var == 1)
	//   {
	// 	  signal_feedback_w_wheel();
	//   }
	//   if(check_var == 2)
	//   {
	// 	  signal_feedback_error();
	//   }
	//   if(check_var == 3)
	//   {
	// 	  signal_feedback_PID_parameter();
	//   }
	//   if(check_var == 4)
	//   {
	// 	  position_feedback();
	//   }
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
  htim3.Init.Prescaler = 16;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  htim4.Init.Prescaler = 16000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 16;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 16;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 999;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 16;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 999;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M01_EN_R_Pin|M01_EN_L_Pin|M02_EN_L_Pin|M02_EN_R_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, M03_EN_R_Pin|M03_EN_L_Pin|M04_EN_L_Pin|M04_EN_R_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : M01_EN_R_Pin M01_EN_L_Pin M02_EN_L_Pin M02_EN_R_Pin */
  GPIO_InitStruct.Pin = M01_EN_R_Pin|M01_EN_L_Pin|M02_EN_L_Pin|M02_EN_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : Sensor_1_Pin Sensor_2_Pin */
  GPIO_InitStruct.Pin = Sensor_1_Pin|Sensor_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Relay_Pin */
  GPIO_InitStruct.Pin = Relay_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Relay_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EN02_A_Pin */
  GPIO_InitStruct.Pin = EN02_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EN02_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN02_B_Pin EN01_B_Pin */
  GPIO_InitStruct.Pin = EN02_B_Pin|EN01_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : M03_EN_R_Pin M03_EN_L_Pin M04_EN_L_Pin M04_EN_R_Pin */
  GPIO_InitStruct.Pin = M03_EN_R_Pin|M03_EN_L_Pin|M04_EN_L_Pin|M04_EN_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : EN03_A_Pin */
  GPIO_InitStruct.Pin = EN03_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EN03_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN03_B_Pin EN04_B_Pin */
  GPIO_InitStruct.Pin = EN03_B_Pin|EN04_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : EN04_A_Pin */
  GPIO_InitStruct.Pin = EN04_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EN04_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EN01_A_Pin */
  GPIO_InitStruct.Pin = EN01_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EN01_A_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void cal_vel_ref(float vx, float vy, float w_angle)
{
	//calculate ref for each wheel
	w_reference1_temp = (1/r_wheel)*(vx - vy - (lx + ly)*w_angle);		    //(rad/s)
	w_reference2_temp = (1/r_wheel)*(vx + vy + (lx + ly)*w_angle);		//(rad/s)
	w_reference3_temp = (1/r_wheel)*(vx + vy - (lx + ly)*w_angle);		    //(rad/s)
	w_reference4_temp = (1/r_wheel)*(vx - vy + (lx + ly)*w_angle);	    //(rad/s)

	//convert to RPM
	w_reference1 = (w_reference1_temp)*60/(2*M_PI);
	w_reference2 = (w_reference2_temp)*60/(2*M_PI);
	w_reference3 = (w_reference3_temp)*60/(2*M_PI);
	w_reference4 = (w_reference4_temp)*60/(2*M_PI);
}

void PID_controller_wheel1(float w_reference1)
{
	w1 = (float)(pulse_counter1)*2.609;
	pulse_counter1 = 0;
	if(w_reference1 == 0)
	{
		signal_pos1 = 0;
		signal_neg1 = 0;
	}
	else
	{
		e1 = w_reference1 - w1;
		pPart1 = kP*e1;
		iPart1 += kI*e1*0.1;
		dPart1 = kD*(e1 - pre_e1)*10;
		PID_signal1 += pPart1 + iPart1 + dPart1;

		if (PID_signal1 > max_pwm) PID_signal1 = max_pwm;
		if (PID_signal1 < -max_pwm) PID_signal1 = -max_pwm;

		if (PID_signal1 >= 0)
		{	//quay thuan
			signal_pos1 = 0;
			signal_neg1 = PID_signal1;
		}
		else
		{	//quay nguoc
			signal_pos1 = -PID_signal1;
			signal_neg1 = 0;
		}
		pre_e1 = e1;
	}
}
void PID_controller_wheel2(float w_reference2)
{
	w2 = (float)(pulse_counter2)*2.609;
	pulse_counter2 = 0;
	if(w_reference2 == 0)
	{
		signal_pos2 = 0;
		signal_neg2 = 0;
	}
	else
	{
		e2 = w_reference2 - w2;
		pPart2 = kP*e2;
		iPart2 += kI*e2*0.1;
		dPart2 = kD*(e2 - pre_e2)*10;
		PID_signal2 += pPart2 + iPart2 + dPart2;

		if (PID_signal2 > max_pwm) PID_signal2 = max_pwm;
		if (PID_signal2 < -max_pwm) PID_signal2 = -max_pwm;

		if (PID_signal2 >= 0)
		{	//quay thuan
			signal_pos2 = PID_signal2;
			signal_neg2 = 0;
		}
		else
		{	//quay nguoc
			signal_pos2 = 0;
			signal_neg2 = -PID_signal2;
		}
		pre_e2 = e2;
	}
}
void PID_controller_wheel3(float w_reference3)
{
	w3 = (float)(pulse_counter3)*2.609;
	pulse_counter3 = 0;
	if(w_reference3 == 0)
	{
		signal_pos3 = 0;
		signal_neg3 = 0;
	}
	else
	{
		e3 = w_reference3 - w3;
		pPart3 = kP*e3;
		iPart3 += kI*e3*0.1;
		dPart3 = kD*(e3 - pre_e3)*10;
		PID_signal3 += pPart3 + iPart3 + dPart3;

		if (PID_signal3 > max_pwm) PID_signal3 = max_pwm;
		if (PID_signal3 < -max_pwm) PID_signal3 = -max_pwm;

		if (PID_signal3 >= 0)
		{	//quay thuan
			signal_pos3 = 0;
			signal_neg3 = PID_signal3;
		}
		else
		{	//quay nguoc
			signal_pos3 = -PID_signal3;
			signal_neg3 = 0;
		}
		pre_e3 = e3;
	}
}
void PID_controller_wheel4(float w_reference4)
{
	w4 = (float)(pulse_counter4)*2.609;
	pulse_counter4 = 0;
	if(w_reference4 == 0)
	{
		signal_pos4 = 0;
		signal_neg4 = 0;
	}
	else
	{
		e4 = w_reference4 - w4;
		pPart4 = kP*e4;
		iPart4 += kI*e4*0.1;
		dPart4 = kD*(e4 - pre_e4)*10;
		PID_signal4 += pPart4 + iPart4 + dPart4;

		if (PID_signal4 > max_pwm) PID_signal4 = max_pwm;
		if (PID_signal4 < -max_pwm) PID_signal4 = -max_pwm;

		if (PID_signal4 >= 0)
		{	//quay thuan
			signal_pos4 = PID_signal4;
			signal_neg4 = 0;
		}
		else
		{	//quay nguoc
			signal_pos4 = 0;
			signal_neg4 = -PID_signal4;
		}
		pre_e4 = e4;
	}
}

void position_cal()
{
	//read encoder RPM
	w1 = (float)(pulse_counter1)*2.609;
	w2 = (float)(pulse_counter2)*2.609;
	w3 = (float)(pulse_counter3)*2.609;
	w4 = (float)(pulse_counter4)*2.609;
	//convert to radian/s
	w1_rad = (w1)*(2*M_PI)/60;
	w2_rad = (w2)*(2*M_PI)/60;
	w3_rad = (w3)*(2*M_PI)/60;
	w4_rad = (w4)*(2*M_PI)/60;
	//convert to m/s
	v_wheel1 = w1_rad*r_wheel;
	v_wheel2 = w2_rad*r_wheel;
	v_wheel3 = w3_rad*r_wheel;
	v_wheel4 = w4_rad*r_wheel;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim4.Instance)
	{
		position_cal();
		cal_vel_ref(vx, vy, w_angle);			//calculate ref for eachwheel
		PID_controller_wheel1(w_reference1);
		PID_controller_wheel2(w_reference2);
		PID_controller_wheel3(w_reference3);
		PID_controller_wheel4(w_reference4);

		__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, signal_pos1);
		__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2, signal_neg1);
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, signal_pos2);
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, signal_neg2);
		__HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, signal_pos3);
		__HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2, signal_neg3);
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, signal_pos4);
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, signal_neg4);
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//Encoder moto1
	if(GPIO_Pin == GPIO_PIN_9)
	{
		if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1) == 0)
		{
			pulse_counter1--;
		}
		else
		{
			pulse_counter1++;
		}
	}
	//Encoder moto2
	if(GPIO_Pin == GPIO_PIN_14)
	{
		if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15) == 0)
		{
			pulse_counter2++;
		}
		else
		{
			pulse_counter2--;
		}
	}
	//Encoder moto3
	if(GPIO_Pin == GPIO_PIN_12)
	{
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) == 0)
		{
			pulse_counter3--;
		}
		else
		{
			pulse_counter3++;
		}
	}
	//Encoder moto4
	if(GPIO_Pin == GPIO_PIN_11)
	{
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1) == 0)
		{
			pulse_counter4++;
		}
		else
		{
			pulse_counter4--;
		}
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart3.Instance)
	{
		if(rx_index == 0)
		{
			for(int i=0;i<=25;i++)
			{
				rx_buffer[i] = 0;
			}
		}
		rx_buffer[rx_index] = rx_data;
		rx_index++;

		// if(rx_data=='p')
		// {
		// 	kP=atof(rx_buffer);
		// 	rx_index=0;
		// }
		// if(rx_data=='i')
		// {
		// 	kI=atof(rx_buffer);
		// 	rx_index=0;
		// }
		// if(rx_data=='d')
		// {
		// 	kD=atof(rx_buffer);
		// 	rx_index=0;
		// }
		if(rx_data=='x')
		{
			vx=atof(rx_buffer);
			rx_index=0;
		}
		if(rx_data=='y')
		{
			vy=atof(rx_buffer);
			rx_index=0;
		}
		if(rx_data=='w')
		{
			w_angle=atof(rx_buffer);
			rx_index=0;
		}
		// if(rx_data=='e')
		// {
		// 	check_var=atof(rx_buffer);
		// 	rx_index=0;
		// }
		HAL_UART_Receive_DMA(&huart3, &rx_data,1);
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

