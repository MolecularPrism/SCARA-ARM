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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "usbd_cdc_if.h"
#include "string.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define kp_x 25
//#define ki_x 1
//#define kd_x 0.6
//#define k_x 3

#define kp_x 25
#define ki_x 1
#define kd_x 0.4
#define k_x 3

#define kp_y 25
#define ki_y 1
#define kd_y 0.4
#define k_y 3

//#define kp_y 25
//#define ki_y 1
//#define kd_y 0.6
//#define k_y 3



#define stepperUnit 1.6666

//#define kp 19
//#define ki 0.8
//#define kd 0.5
//#define k 1.1


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
//extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Preset Shape Coordinates
int N = 1;
const float star_y[13] = {-10 * stepperUnit,-5* stepperUnit,-5* stepperUnit,0,5* stepperUnit,5* stepperUnit,10* stepperUnit,5* stepperUnit,5* stepperUnit,0,-5* stepperUnit,-5* stepperUnit,-10* stepperUnit}; //star
const float star_x[13] = {0,5* stepperUnit,10* stepperUnit,5* stepperUnit,10* stepperUnit,5* stepperUnit,0,-5* stepperUnit,-10* stepperUnit,-5* stepperUnit, -10* stepperUnit, -5* stepperUnit,0};

const float square_y[7] = {0, stepperUnit * -10, stepperUnit * -10, 0, stepperUnit * 10, stepperUnit * 10, 0}; //square
const float square_x[7] = {stepperUnit * -10, stepperUnit * -10, stepperUnit * 10, stepperUnit * 10, stepperUnit* 10, stepperUnit * -10, stepperUnit * -10};

const float diamond_y[5] = {0,-15,0,15, 0}; //diamond
const float diamond_x[5] = {0,15,30,15, 0};

const float C_six_x[31] = {-1* stepperUnit* 2, -2* stepperUnit* 2, -3* stepperUnit* 2, -3* stepperUnit* 2, -2* stepperUnit* 2, -1* stepperUnit* 2, 0, 2* stepperUnit* 2, 5* stepperUnit* 2, 3* stepperUnit* 2, 4* stepperUnit* 2, 5* stepperUnit* 2, 6* stepperUnit* 2, 4* stepperUnit* 2, 2* stepperUnit* 2, 5* stepperUnit* 2, 2* stepperUnit * 2, 4* stepperUnit* 2, 6* stepperUnit* 2, 5* stepperUnit* 2, 4* stepperUnit* 2, 3* stepperUnit* 2, 5* stepperUnit* 2, 2* stepperUnit* 2, 0* stepperUnit* 2, -1* stepperUnit* 2, -2* stepperUnit* 2, -3* stepperUnit* 2, -3* stepperUnit * 2, -2* stepperUnit * 2, -1* stepperUnit * 2}; //C6
const float C_six_y[31] = {-6* stepperUnit* 2, -5* stepperUnit* 2, -4* stepperUnit* 2, 1* stepperUnit* 2, 4* stepperUnit* 2, 3* stepperUnit* 2, 2* stepperUnit* 2, 1* stepperUnit* 2, -5* stepperUnit* 2, 2* stepperUnit* 2, 3* stepperUnit* 2, 2* stepperUnit* 2, 1* stepperUnit* 2, 0, 1* stepperUnit* 2, -5* stepperUnit* 2, 1* stepperUnit* 2, 0, 1*stepperUnit* 2, 2* stepperUnit* 2, 3* stepperUnit* 2, 2* stepperUnit* 2, -5* stepperUnit* 2, 1* stepperUnit* 2, 2* stepperUnit* 2, 3* stepperUnit* 2, 4* stepperUnit* 2, 1* stepperUnit* 2, -4* stepperUnit* 2, -5* stepperUnit* 2, -6* stepperUnit* 2};


//const float square_y[1] = {-25}; //test
//const float square_x[1] = {25};

//const float square_y[] = {-9,-9,-3, 3, 9, 9, 3, -3, -9};
//const float square_x[] = {};

//const float square_y[N] = {0};
//const float square_x[N] = {0};

// PID Control Vars
volatile float CF = 2000, deltaTime = 0, error_previous_x = 0, error_previous_y = 0, error_integral_x = 0, error_integral_y = 0, pid_out_x = 0, pid_out_y = 0;

const float integral_max = 2, integral_min = -2;
volatile float desired_angle_x = 20, desired_angle_y = 20;

//Decoder Vars
volatile float current_decoder_val_M1 = 0, current_decoder_val_M2 = 0, current_angle_x = 0, current_angle_y = 0;


// Drawing Shape Vars
volatile float x_coord = 1, y_coord = 1;
volatile float wall_distance = 1;
volatile float current_angle_rad = 0;

//WSF Vars
volatile float raw_deriv_samples_x[10] = { 0, 0, 0, 0, 0, 0, 0 };
volatile float raw_deriv_samples_y[10] = { 0, 0, 0, 0, 0, 0, 0 };
const int WSF_SAMPLE_COUNT = 10;
const float WSF_CONST_LOOKUP_TABLE[10] = { 0.36307, 0.2327935, 0.149263, 0.095704, 0.0613637, 0.0393452, 0.0252274, 0.0161753, 0.0103713, 0.006649864 };

//Homing Vars
volatile int is_limitSW_x = 0, is_limitSW_y = 0;
volatile int is_homing_x = 0;
volatile int is_homing_y = 0;
volatile int is_zeroing_x = 0;
volatile int is_zeroing_y = 0;
volatile int is_zeroed_x = 0;
volatile int is_zeroed_y = 0;

//Motor Stop Vars
volatile int is_stop_requested = 0;
volatile int is_SW_on = 0;

//test var
volatile int test1 = 0;
volatile int test2 = 0;
volatile float max_angle_x = 0.0;
volatile float max_angle_y = 0.0;
volatile float min_angle_x = 0.0;
volatile float min_angle_y = 0.0;
volatile float current_angle_test = 0;



GPIO_PinState bit0_two = 0;



//prototype
void limit_switch_trigger();

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  TIM1 -> CCR1 = 0;
  TIM2 -> CCR1 = 0;

  TIM1 -> CCR2 = 0;
  TIM2 -> CCR2 = 0;


  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); //NRST init on
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); //SEL off
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); //CLK_OUT off
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); //RST off





  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	//limit_switch_trigger();








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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim2.Init.Period = 255;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
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
  htim3.Init.Prescaler = 9;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4799;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 PA5 PA6
                           PA7 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB3
                           PB4 PB5 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB6 PB7 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



int binaryToDecimal(int bit3, int bit2, int bit1, int bit0) {
  return bit3 * 8 + bit2 * 4 + bit1 * 2 + bit0;
}

void readDecoder(){

	GPIO_PinState bit3_one = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
	GPIO_PinState bit2_one = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
	GPIO_PinState bit1_one = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
	GPIO_PinState bit0_one = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);

	GPIO_PinState bit3_two = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
	GPIO_PinState bit2_two = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
	GPIO_PinState bit1_two = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
	bit0_two = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);

	current_decoder_val_M1 = binaryToDecimal(bit3_one, bit2_one, bit1_one, bit0_one);
	current_decoder_val_M2 = binaryToDecimal(bit3_two, bit2_two, bit1_two, bit0_two);


}

void execute_reset_seq(){
	//SEL on
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

		// Delay loop for approximately 1 microsecond
		for (volatile uint32_t i = 0; i < 10; ++i) {
			__NOP(); // No Operation assembly instruction
		}



		//RST on
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);

		// Delay loop for approximately 1 microsecond
			for (volatile uint32_t i = 0; i < 10; ++i) {
				__NOP(); // No Operation assembly instruction
			}

		//NRST off
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

		// Delay loop for approximately 0.2 microsecond
			for (volatile uint32_t i = 0; i < 2; ++i) {
				__NOP(); // No Operation assembly instruction
			}


		//RST off
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

		// Delay loop for approximately 1 microsecond
			for (volatile uint32_t i = 0; i < 10; ++i) {
				__NOP(); // No Operation assembly instruction
			}


		//read decoder value
		readDecoder();

		//NRST ON
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

		// Delay loop for approximately 0.2 microsecond
		for (volatile uint32_t i = 0; i < 2; ++i) {
			__NOP(); // No Operation assembly instruction
		}

		//CLKOUT ON
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);

		//SEL OFF
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

		// Delay loop for approximately 1 microsecond
		for (volatile uint32_t i = 0; i < 10; ++i) {
			__NOP(); // No Operation assembly instruction
		}

		//CLKOUT OFF
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
}

void limit_switch_trigger(){
	if(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15)){
		is_limitSW_x = 1;

	}

	if(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3)){
		is_limitSW_y = 1;
	}
}

float FDD_WSF_PID(volatile float* raw_deriv_samples, float error_derivative) {
    float sum = 0.0;

    // Shift all the derivative samples to the right by 1 index
    for (int i = WSF_SAMPLE_COUNT - 1; i > 0; i--) {
        raw_deriv_samples[i] = raw_deriv_samples[i - 1];
    }

    // Insert new derivative data to the first index
    raw_deriv_samples[0] = error_derivative;

    // Sum all of them
    for (int i = 0; i < WSF_SAMPLE_COUNT; i++) {
        sum += raw_deriv_samples[i] * WSF_CONST_LOOKUP_TABLE[i];
    }

    // Update filtered error derivative
    return sum;
}

float pidControl(const float desired_angle, volatile float *error_integral, volatile float current_angle, volatile float *error_previous, volatile float* raw_derivative_samples, int is_x) {

  deltaTime = 1.0/CF;

  // error
  float error = desired_angle - current_angle;

  // derivative
  float error_derivative = (error - *error_previous) / (deltaTime);

  float filtered_error_derivative = FDD_WSF_PID(raw_derivative_samples, error_derivative);

  // integral
  *error_integral = *error_integral + error * deltaTime;

  if(*error_integral > integral_max){
    *error_integral = integral_max;
  }
  else if(*error_integral < integral_min){
    *error_integral = integral_min;
  }

  *error_previous = error;

  // control signal

  if(is_x){
	  return k_x * (kp_x * error + kd_x * filtered_error_derivative + ki_x * (*error_integral));
  }else{
	  return k_y * (kp_y * error + kd_y * filtered_error_derivative + ki_y * (*error_integral));
  }



}

void setMotorSpeed() {
  // Get motor speed
  int motor_speed_x = fabs(pid_out_x);
  int motor_speed_y = fabs(pid_out_y);

  if(is_stop_requested ){
	  motor_speed_x = 0;
	  motor_speed_y = 0;
  }
  else{
	  if (motor_speed_x > 255) {
	      motor_speed_x = 255;
	  }

	  if (motor_speed_y > 255){
	  	  motor_speed_y = 255;
	  }
  }



  // Get motor direction
  if (pid_out_x > 0) {
	  TIM1 -> CCR1 = 0;
	  TIM1 -> CCR2 = motor_speed_x;
  } else {
	  TIM1 -> CCR2 = 0;
	  TIM1 -> CCR1 = motor_speed_x;
  }

  if (pid_out_y > 0) {
	  TIM2 -> CCR1 = 0;
	  TIM2 -> CCR2 = motor_speed_y;
    } else {
    	TIM2 -> CCR2 = 0;
    	TIM2 -> CCR1 = motor_speed_y;
    }

}

void convertAngleToXY() {
	current_angle_rad = atan(x_coord/wall_distance);

	current_angle_test = current_angle_rad * M_PI/180;
}

int count = 0;

int shape_count = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if(shape_count == 0){ //square
		N = 7;
	}
	else if(shape_count == 1){ //diamond
		N = 5;
	}
	else if(shape_count == 2){ //star
		N = 13;
	}
	else if(shape_count == 3){ //C6
			N = 31;
	}



	if(count > (N-1)){ //>3
			count = 0;
			//count = 0;
		}

	if(!is_limitSW_x && !is_homing_x){
		if(!is_limitSW_y && !is_homing_y){

			if(shape_count == 0){ //square
				desired_angle_x = square_x[count];
				desired_angle_y = square_y[count];
			}
			else if(shape_count == 1){ //diamond
				desired_angle_x = diamond_x[count];
				desired_angle_y = diamond_y[count];
			}
			else if(shape_count == 2){ //star
				desired_angle_x = star_x[count];
				desired_angle_y = star_y[count];
			}
			else if(shape_count == 3){ //star
				desired_angle_x = C_six_x[count];
				desired_angle_y = C_six_y[count];
			}

		}

	}




//	else{
//		desired_angle_x = 0;
//		desired_angle_y = 0;
//	}


	if(fabs(desired_angle_y - current_angle_y) < 0.1){
		if(fabs(desired_angle_x - current_angle_x) < 0.1){
			count++;
		}

	}

	test1 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15);
	test2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);

//	if(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)) { //estop
//		is_SW_on = 1;
//
//
//		desired_angle_x = 0;
//		if(fabs(desired_angle_x - current_angle_x) < 0.1){
//			is_at_zero_x = 1;
//		}
//
//	}else{
//
//
//		is_homing_x = 1;
//		is_zeroing_x = 0;
//		is_zeroing_y = 0;
//
//
//	}
//
//	if(is_at_zero_x){
//		desired_angle_y = 10 * stepperUnit;
//
//	}

	//if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15)) { //motor-off & home switch
	//if(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)){ //e-stop to stop motors and point laser down

	if(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)){ //e-stop
		if(is_stop_requested){



			is_homing_x = 1;


			is_zeroing_x = 0;
			is_zeroing_y = 0;

			shape_count++;

			if(shape_count > 3){
				shape_count = 0;
			}
		}

		is_stop_requested = 0;

	}else{
		is_stop_requested = 1;

//		if(!is_stop_requested){
//			shape_count++;
//
//			if(shape_count > 2){
//				shape_count = 0;
//			}
//		}
	}


	execute_reset_seq();

	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)){
		current_angle_x += (current_decoder_val_M1 * 1.6666);
	}
	else{
		current_angle_x -= (current_decoder_val_M1 * 1.6666);
	}

	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)){
		current_angle_y += (current_decoder_val_M2 * 1.6666);
	}
	else{
		current_angle_y -= (current_decoder_val_M2 * 1.6666);
	}


	//homing
	if(is_homing_x){
		desired_angle_x += 0.04;
	}

	if(is_homing_y){
		desired_angle_y += 0.21;
	}

	if(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15)){
			is_limitSW_x = 1;
	}

	if(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3)){
			is_limitSW_y = 1;
	}

	if(is_limitSW_x){
		if(!is_zeroing_x){
			current_angle_x = 42 * stepperUnit;
		}

		is_zeroing_x = 1;
		desired_angle_x = 0;
		is_homing_x = 0;


		if(fabs(desired_angle_x - current_angle_x) < 0.1){
			current_angle_x = 0;
			is_limitSW_x = 0;
			is_homing_y = 1;
			//is_zeroed_x = 1; //is this needed?

		}
	}

	if(is_limitSW_y){
			if(!is_zeroing_y){
				current_angle_y = 46 * stepperUnit;
			}

			is_zeroing_y = 1;
			desired_angle_y = 0;
			is_homing_y = 0;


			if(fabs(desired_angle_y - current_angle_y) < 0.1){
				current_angle_y = 0;
				is_limitSW_y = 0;

				//is_SW_on = 0;

				//is_zeroed_y = 1; //is this needed?

			}
		}



	pid_out_x = pidControl(desired_angle_x, &error_integral_x ,current_angle_x, &error_previous_x, raw_deriv_samples_x, 1);
	pid_out_y = pidControl(desired_angle_y, &error_integral_y, current_angle_y, &error_previous_y, raw_deriv_samples_y, 0);

	setMotorSpeed();

	convertAngleToXY();

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
