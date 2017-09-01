/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "WheelSide.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern "C"{
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */

}
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
int pulseLeftFront, pulseLeftBack, pulseRightFront, pulseRightBack = 0;


bool posReached = 0;;
float linearPosition = 0;
float angularPosition = 0;
float linearSpeed = 0;
float angularSpeed = 0;
float errorDistance = 0;
float errorAngle = 0;
float errorLinearSpeed = 0;
float errorAngularSpeed = 0;
float rightSpeed = 0;
float leftSpeed = 0;
uint32_t rightPulse = 0;
uint32_t leftPulse = 0;
float leftCommand = 0;
float rightCommand = 0;
float commandLinear = 0;
float commandAngular = 0;
float speedLinearDesired = 0;
float speedAngularDesired = 0;
float speedLinearDesiredAf = 0;
float speedAngularDesiredAf = 0;

float accLimit = 30;
float speedLimit = 1000;

float distanceDes = 3000;
float angleDes = 0;
float orientationDes = angleDes  * 663*270/360/95;

PID linearPositionPID;
PID angularPositionPID;
PID linearSpeedPID;
PID angularSpeedPID;

/*
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if(GPIO_Pin==leftFront_pinA_Pin)
	{
		pulseLeftFront = countPulse(leftFront_pinB_GPIO_Port, leftFront_pinB_Pin, pulseLeftFront);
	}
	if(GPIO_Pin==rightFront_pinA_Pin)
	{
		pulseRightFront = countPulse(rightFront_pinB_GPIO_Port, rightFront_pinB_Pin, pulseRightFront);
	}
	if(GPIO_Pin==leftBack_pinA_Pin)
	{
		pulseLeftBack = countPulse(leftBack_pinB_GPIO_Port, leftBack_pinB_Pin, pulseLeftBack);
	}
	if(GPIO_Pin==rightBack_pinA_Pin)
	{
		pulseRightBack = countPulse(rightBack_pinB_GPIO_Port, rightBack_pinB_Pin, pulseRightBack);
	}
}*/


Wheel leftFrontWheel(leftFront_PWM_GPIO_Port, leftFront_PWM_Pin, leftFront_Dir_GPIO_Port, leftFront_Dir_Pin,
	  		leftFront_pinA_GPIO_Port, leftFront_pinA_Pin, leftFront_pinB_GPIO_Port, leftFront_pinB_Pin, &htim3, TIM_CHANNEL_1);
Wheel leftBackWheel(leftBack_PWM_GPIO_Port, leftBack_PWM_Pin, leftBack_Dir_GPIO_Port, leftBack_Dir_Pin,
	  		leftBack_pinA_GPIO_Port, leftBack_pinA_Pin, leftBack_pinB_GPIO_Port, leftBack_pinB_Pin, &htim3, TIM_CHANNEL_3);
Wheel rightFrontWheel(rightFront_PWM_GPIO_Port, rightFront_PWM_Pin, rightFront_Dir_GPIO_Port, rightFront_Dir_Pin,
	  		rightFront_pinA_GPIO_Port, rightFront_pinA_Pin, rightFront_pinB_GPIO_Port, rightFront_pinB_Pin, &htim3, TIM_CHANNEL_4);
Wheel rightBackWheel(rightBack_PWM_GPIO_Port, rightBack_PWM_Pin, rightBack_Dir_GPIO_Port, rightBack_Dir_Pin,
	  		rightBack_pinA_GPIO_Port, rightBack_pinA_Pin, rightBack_pinB_GPIO_Port, rightBack_pinB_Pin, &htim3, TIM_CHANNEL_2);

WheelSide leftWheel(leftFrontWheel, leftBackWheel);
WheelSide rightWheel(rightFrontWheel, rightBackWheel);


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if(GPIO_Pin==leftFront_pinA_Pin)
	{
		leftFrontWheel.countPulse();
	}
	if(GPIO_Pin==rightFront_pinA_Pin)
	{
		rightFrontWheel.countPulse();
	}
	if(GPIO_Pin==leftBack_pinA_Pin)
	{
		leftBackWheel.countPulse();
	}
	if(GPIO_Pin==rightBack_pinA_Pin)
	{
		rightBackWheel.countPulse();
	}
}

PID leftPID(0.5,0,0);
PID rightPID(0.5,0,0);
float leftSpeedMeas = 0;
float rightSpeedMeas = 0;
float speedDes = 200;
uint32_t time = 0.01;
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM3_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
  //Timer start
  HAL_TIM_Base_Start(&htim3);
  //PWM start
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);	 //leftFront
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  //rightBack
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); //leftBack
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);  //rightFront

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	uint8_t buffer[50];

	time = HAL_GetTick();

	/*leftFrontWheel.run(30);
	rightFrontWheel.run(30);
	leftBackWheel.run(30);
	rightBackWheel.run(30);*/

	/*leftWheel.run(22);
	rightWheel.run(22);*/

	leftWheel.run(leftPID.computePID(speedDes - leftWheel.getSpeedMeas()));
	rightWheel.run(rightPID.computePID(speedDes - rightWheel.getSpeedMeas()));

	if(time >= 0.05)
	{
		time = 0;

		//sprintf((char *)buffer, "%i\n\r", rightFrontWheel.getPulse())

		//sprintf((char *)buffer, "%i %i %i %i \n\r", leftFrontWheel.getPulse(), rightFrontWheel.getPulse(), leftBackWheel.getPulse(), rightBackWheel.getPulse());
		//sprintf((char *)buffer, "%i %i %i %i \n\r", (int)leftFrontWheel.getSpeedMeas(), (int)rightFrontWheel.getSpeedMeas(), (int)leftBackWheel.getSpeedMeas(), (int)rightBackWheel.getSpeedMeas());

		//sprintf((char *)buffer, "%i \t %i \n\r", (int)leftWheel.getPulse(), (int)rightWheel.getPulse());
		//sprintf((char *)buffer, "%i \t %i \n\r", (int)leftWheel.getSpeedMeas(), (int)rightWheel.getSpeedMeas());

		sprintf((char *)buffer, "%i \t %i \t %i \t %i \n\r", (int)leftWheel.getPulse(), (int)rightWheel.getPulse(), (int)leftWheel.getSpeedMeas(), (int)rightWheel.getSpeedMeas());
	}

	HAL_UART_Transmit(&huart3, buffer, strlen((char *)buffer), 0xFFFFF);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 2160;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 255;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, leftFront_Dir_Pin|rightFront_Dir_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(leftBack_Dir_GPIO_Port, leftBack_Dir_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(rightBack_Dir_GPIO_Port, rightBack_Dir_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : leftFront_pinB_Pin rightBack_pinA_Pin leftBack_pinB_Pin rightFront_pinA_Pin */
  GPIO_InitStruct.Pin = leftFront_pinB_Pin|rightBack_pinA_Pin|leftBack_pinB_Pin|rightFront_pinA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : leftFront_pinA_Pin */
  GPIO_InitStruct.Pin = leftFront_pinA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(leftFront_pinA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : leftFront_Dir_Pin rightFront_Dir_Pin */
  GPIO_InitStruct.Pin = leftFront_Dir_Pin|rightFront_Dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : leftBack_pinA_Pin rightFront_pinB_Pin */
  GPIO_InitStruct.Pin = leftBack_pinA_Pin|rightFront_pinB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : leftBack_Dir_Pin */
  GPIO_InitStruct.Pin = leftBack_Dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(leftBack_Dir_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : rightBack_Dir_Pin */
  GPIO_InitStruct.Pin = rightBack_Dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(rightBack_Dir_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : rightBack_pinB_Pin */
  GPIO_InitStruct.Pin = rightBack_pinB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(rightBack_pinB_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
