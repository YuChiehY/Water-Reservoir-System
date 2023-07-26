/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t byte;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
static void ADC_Select_CH(int CH);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t us100_Rx_flag = 00;
uint8_t rcv_intpt_flag = 0;
uint8_t cmd_dist = 0x55;
volatile uint16_t rpm_tick_count = 0;
int waterflag = 1;
int PWM_MODE_INLET;
int RUN_TIME_INLET;
int PWM_MODE_Z1;
int RUN_TIME_Z1;
int PWM_MODE_Z2;
int RUN_TIME_Z2;
int PWM_MODE_Z3;
int RUN_TIME_Z3;
int SET_UP_FLAG;
uint8_t txd_msg_buffer[256] = {0};
volatile int clock_secs;
float max_dist = 100;
void ADC_Select_CH(int CH){
	ADC_ChannelConfTypeDef sConfig = {0};

	switch(CH){
	case 9:
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = 1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){
		Error_Handler();
	}
	break;
	}
}
void DIGIT_A_Display(uint8_t DIGIT_A)
{
	 uint8_t DIGITA_VAL = 0x0F & DIGIT_A; //mask off higher4 bits

	 switch(DIGITA_VAL)
	 {
	 case 0:
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A0_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A1_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A2_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A3_Pin, GPIO_PIN_RESET);
	 break;
	 case 1:
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A0_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A1_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A2_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A3_Pin, GPIO_PIN_RESET);
	 break;
	 case 2:
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A0_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A1_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A2_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A3_Pin, GPIO_PIN_RESET);
	 break;
	 case 3:
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A0_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A1_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A2_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A3_Pin, GPIO_PIN_RESET);
	 break;
	 case 4:
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A0_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A1_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A2_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A3_Pin, GPIO_PIN_RESET);
	 break;
	 case 5:
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A0_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A1_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A2_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A3_Pin, GPIO_PIN_RESET);
	 break;
	 case 6:
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A0_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A1_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A2_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A3_Pin, GPIO_PIN_RESET);
	 break;
	 case 7:
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A0_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A1_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A2_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A3_Pin, GPIO_PIN_RESET);
	 break;
	 case 8:
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A0_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A1_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A2_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A3_Pin, GPIO_PIN_SET);
	 break;
	 case 9:
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A0_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A1_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A2_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A3_Pin, GPIO_PIN_SET);
	 break;
	 }
}
void DIGIT_B_Display(uint8_t DIGIT_B)
{
	 uint8_t DIGITB_VAL = 0x0F & DIGIT_B; //mask off higher4 buts

	 switch(DIGITB_VAL)
	 {
	 case 0:
		 HAL_GPIO_WritePin(GPIOB, DIGIT_B0_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_B1_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(DIGIT_B2_GPIO_Port, DIGIT_B2_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, DIGIT_B3_Pin, GPIO_PIN_RESET);
	 break;
	 case 1:
		 HAL_GPIO_WritePin(GPIOB, DIGIT_B0_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_B1_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(DIGIT_B2_GPIO_Port, DIGIT_B2_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, DIGIT_B3_Pin, GPIO_PIN_RESET);
	 break;
	 case 2:
		 HAL_GPIO_WritePin(GPIOB, DIGIT_B0_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_B1_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(DIGIT_B2_GPIO_Port, DIGIT_B2_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, DIGIT_B3_Pin, GPIO_PIN_RESET);
	 break;
	 case 3:
		 HAL_GPIO_WritePin(GPIOB, DIGIT_B0_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_B1_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(DIGIT_B2_GPIO_Port, DIGIT_B2_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, DIGIT_B3_Pin, GPIO_PIN_RESET);
	 break;
	 case 4:
		 HAL_GPIO_WritePin(GPIOB, DIGIT_B0_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_B1_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(DIGIT_B2_GPIO_Port, DIGIT_B2_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOC, DIGIT_B3_Pin, GPIO_PIN_RESET);
	 break;
	 case 5:
		 HAL_GPIO_WritePin(GPIOB, DIGIT_B0_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_B1_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(DIGIT_B2_GPIO_Port, DIGIT_B2_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOC, DIGIT_B3_Pin, GPIO_PIN_RESET);
	 break;
	 case 6:
		 HAL_GPIO_WritePin(GPIOB, DIGIT_B0_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_B1_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(DIGIT_B2_GPIO_Port, DIGIT_B2_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOC, DIGIT_B3_Pin, GPIO_PIN_RESET);
	 break;
	 case 7:
		 HAL_GPIO_WritePin(GPIOB, DIGIT_B0_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_B1_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(DIGIT_B2_GPIO_Port, DIGIT_B2_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOC, DIGIT_B3_Pin, GPIO_PIN_RESET);
	 break;
	 case 8:
		 HAL_GPIO_WritePin(GPIOB, DIGIT_B0_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_B1_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(DIGIT_B2_GPIO_Port, DIGIT_B2_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, DIGIT_B3_Pin, GPIO_PIN_SET);
	 break;
	 case 9:
		 HAL_GPIO_WritePin(GPIOB, DIGIT_B0_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOB, DIGIT_B1_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(DIGIT_B2_GPIO_Port, DIGIT_B2_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, DIGIT_B3_Pin, GPIO_PIN_SET);
	 break;
	 }
}
int choosespeed( int PWM_SIG){
	switch(PWM_SIG){
	case 0:
		return 0;
	break;
	case 1:
		return 10;
	break;
	case 2:
		return 20;
	break;
	case 3:
		return 30;
	break;
	case 4:
		return 40;
	break;
	case 5:
		return 50;
	break;
	case 6:
		return 60;
	break;
	case 7:
		return 70;
	break;
	case 8:
		return 85;
	break;
	case 9:
		return 99;
	break;
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_Base_Init(&htim4);
  HAL_TIM_Base_Init(&htim1);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  TIM2->PSC = 16-1;
  TIM2->ARR = 20000-1;
  TIM2->CCR1 = 0;
  TIM3->PSC = 160-1;
  TIM3->ARR = 100-1;
  TIM3->CCR1 = 0;
  uint16_t distance;
  uint8_t us100_buffer[2];
  void inletfill_up(float inlet_time, int inlet_speed){
	TIM2->CCR1 = 1100;
	HAL_GPIO_WritePin(GPIOB, BLUE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, RED_Pin, GPIO_PIN_SET);
	HAL_Delay(5000);
	HAL_GPIO_WritePin(GPIOA, RED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, BLUE_Pin, GPIO_PIN_RESET);
	if(inlet_speed == -1){
		ADC_Select_CH(9);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1000);
		uint8_t ADC_CH9 = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		inlet_speed = ((ADC_CH9)/255)*5000 - 1;
		sprintf((char*)txd_msg_buffer, "Manual Speed is: %d\n\r", inlet_speed);
		HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
	}
	clock_secs = 0;
	TIM1->CCR4 = 0;
	TIM4->CCR1 = inlet_speed;
  	while(clock_secs!=inlet_time){
  	}
  	TIM4->CCR1 = 0;
  	float mins = inlet_time/60;
  	float rpm = ((float)rpm_tick_count/mins)/40;
	sprintf((char*)txd_msg_buffer, "RPM Inlet is: %d\n\r", (int)rpm);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
	sprintf((char*)txd_msg_buffer, "Speed is: %d\n\r", inlet_speed);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
	rpm_tick_count = 0;
	HAL_UART_Receive_IT(&huart1,us100_buffer, 2);
	HAL_UART_Transmit(&huart1, &cmd_dist, 1, 500);
	while(us100_Rx_flag == (00)) {};
	distance = (us100_buffer[0]<<8);
	distance += us100_buffer[1];
	float waterleft = 99;
	sprintf((char*)txd_msg_buffer, "Percentage Full: %d\n\r", (int) waterleft);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
	DIGIT_A_Display(9);
	DIGIT_B_Display(9);
	us100_Rx_flag = 00;
  }
  int zone1run(float zone1_time, int zone1_speed){
	TIM2->CCR1 = 1225;
	HAL_GPIO_WritePin(GPIOA, RED_Pin, GPIO_PIN_SET);
	HAL_Delay(5000);
	HAL_GPIO_WritePin(GPIOA, RED_Pin, GPIO_PIN_RESET);
	if(zone1_speed == -1){
		ADC_Select_CH(9);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1000);
		uint8_t ADC_CH9 = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		zone1_speed = ((ADC_CH9)/255)*5000 - 1;
		sprintf((char*)txd_msg_buffer, "Manual Speed is: %d\n\r", zone1_speed);
		HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
	}
  	clock_secs = 0;
  	TIM4->CCR1 = 0;
  	TIM1->CCR4 = zone1_speed;
  	while(clock_secs!=zone1_time){
  	}
  	TIM1->CCR4 = 0;
  	float mins = zone1_time/60;
  	float rpm = (rpm_tick_count/mins)/40;
	sprintf((char*)txd_msg_buffer, "RPM Z1 is: %d\n\r", (int)rpm);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
	sprintf((char*)txd_msg_buffer, "Speed is: %d\n\r", zone1_speed);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
	rpm_tick_count = 0;
	HAL_UART_Receive_IT(&huart1,us100_buffer, 2);
	HAL_UART_Transmit(&huart1, &cmd_dist, 1, 500);
	while(us100_Rx_flag == (00)) {};
	distance = (us100_buffer[0]<<8);
	distance += us100_buffer[1];
	float waterleft = 100 - (distance / max_dist)*100;
	if (waterleft <= 0){
		waterleft = 0;
	}
	sprintf((char*)txd_msg_buffer, "Percentage Full: %d\n\r", (int)waterleft);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
	us100_Rx_flag = 00;
	if (waterleft <= 0){
		DIGIT_A_Display(0);
		DIGIT_B_Display(0);
		return 1;
	}
	DIGIT_A_Display((int)waterleft/10);
	DIGIT_B_Display((int)waterleft%10);
	return 0;
  }
  int zone2run(float zone2_time, int zone2_speed){
	TIM2->CCR1 = 950;
	HAL_GPIO_WritePin(GPIOC, GREEN_Pin, GPIO_PIN_SET);
	HAL_Delay(5000);
	HAL_GPIO_WritePin(GPIOC, GREEN_Pin, GPIO_PIN_RESET);
	if(zone2_speed == -1){
		ADC_Select_CH(9);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1000);
		uint8_t ADC_CH9 = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		zone2_speed = ((ADC_CH9)/255)*5000 - 1;
		sprintf((char*)txd_msg_buffer, "Manual Speed is: %d\n\r", zone2_speed);
		HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
	}
  	clock_secs = 0;
  	TIM4->CCR1 = 0;
  	TIM1->CCR4 = zone2_speed;
  	while(clock_secs!=zone2_time){
  	}
  	TIM1->CCR4 = 0;
  	float mins = zone2_time/60;
  	float rpm = (rpm_tick_count/mins)/40;
	sprintf((char*)txd_msg_buffer, "RPM Z2 is: %d\n\r", (int)rpm);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
	sprintf((char*)txd_msg_buffer, "Speed is: %d\n\r", zone2_speed);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
	rpm_tick_count = 0;
	HAL_UART_Receive_IT(&huart1,us100_buffer, 2);
	HAL_UART_Transmit(&huart1, &cmd_dist, 1, 500);
	while(us100_Rx_flag == (00)) {};
	distance = (us100_buffer[0]<<8);
	distance += us100_buffer[1];
	float waterleft = 100 - (distance / max_dist)*100;
	if (waterleft <= 0){
		waterleft = 0;
	}
		sprintf((char*)txd_msg_buffer, "Percentage Full: %d\n\r", (int)waterleft);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
	us100_Rx_flag = 00;
	if (waterleft <= 0){
		DIGIT_A_Display(0);
		DIGIT_B_Display(0);
		return 1;
	}
	DIGIT_A_Display((int)waterleft/10);
	DIGIT_B_Display((int)waterleft%10);
	return 0;
  }
  int zone3run(float zone3_time, int zone3_speed){
	TIM2->CCR1 = 840;
	HAL_GPIO_WritePin(GPIOB, BLUE_Pin, GPIO_PIN_SET);
	HAL_Delay(5000);
	HAL_GPIO_WritePin(GPIOB, BLUE_Pin, GPIO_PIN_RESET);
	if(zone3_speed == -1){
		ADC_Select_CH(9);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1000);
		uint8_t ADC_CH9 = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		zone3_speed = ((ADC_CH9)/255)*5000 - 1;
		sprintf((char*)txd_msg_buffer, "Manual Speed is: %d\n\r", zone3_speed);
		HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
	}
  	clock_secs = 0;
  	TIM4->CCR1 = 0;
  	TIM1->CCR4 = zone3_speed;
  	while(clock_secs!=zone3_time){
  	}
  	TIM1->CCR4 = 0;
  	float mins = zone3_time/60;
  	float rpm = (rpm_tick_count/mins)/40;
	sprintf((char*)txd_msg_buffer, "RPM Z3 is: %d\n\r", (int)rpm);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
	sprintf((char*)txd_msg_buffer, "Speed is: %d\n\r", zone3_speed);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
	rpm_tick_count = 0;
	HAL_UART_Receive_IT(&huart1,us100_buffer, 2);
	HAL_UART_Transmit(&huart1, &cmd_dist, 1, 500);
	while(us100_Rx_flag == (00)) {};
	distance = (us100_buffer[0]<<8);
	distance += us100_buffer[1];
	float waterleft = 100 - (distance / max_dist)*100;
	if (waterleft <= 0){
		waterleft = 0;
	}
		sprintf((char*)txd_msg_buffer, "Percentage Full: %d\n\r", (int)waterleft);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
	us100_Rx_flag = 00;
	if (waterleft <= 0){
		DIGIT_A_Display(0);
		DIGIT_B_Display(0);
		return 1;
	}
	DIGIT_A_Display((int)waterleft/10);
	DIGIT_B_Display((int)waterleft%10);
	return 0;
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)

  {
	  if (SET_UP_FLAG == 1){
		  int runs = 0;
			sprintf((char*)txd_msg_buffer, "RUN MODE\n\r");
			HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
			int inletspeed = ((choosespeed(PWM_MODE_INLET)*5000)/100)-1;
			sprintf((char*)txd_msg_buffer, "Speed Inlet: %d\n\r", inletspeed);
			HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
			float inlettime = ((float)RUN_TIME_INLET/10)*60;
			int zone1speed = ((choosespeed(PWM_MODE_Z1)*5000)/100)-1;
			sprintf((char*)txd_msg_buffer, "Speed 1: %d\n\r", zone1speed);
			HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
			float zone1time = ((float)RUN_TIME_Z1/10)*60;
			int zone2speed = ((choosespeed(PWM_MODE_Z2)*5000)/100)-1;
			sprintf((char*)txd_msg_buffer, "Speed 2: %d\n\r", zone2speed);
			HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
			float zone2time = ((float)RUN_TIME_Z2/10)*60;
			int zone3speed = ((choosespeed(PWM_MODE_Z3)*5000)/100)-1;
			sprintf((char*)txd_msg_buffer, "Speed 3: %d\n\r", zone3speed);
			HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
			float zone3time = ((float)RUN_TIME_Z3/10)*60;
		  while (1){
			  while (waterflag == 0){
				  waterflag = zone1run(zone1time, zone1speed);
				  if (waterflag == 1){
					  break;
				  }
				  waterflag = zone2run(zone2time, zone2speed);
				  if (waterflag == 1){
					  break;
				  }
				  waterflag = zone3run(zone3time, zone3speed);
			  }
			  inletfill_up(inlettime, inletspeed);
			  runs+=1;
			  if (runs == 2){
				  break;
			  }
			  waterflag = 0;
		  }
	  }
	  else{
		  sprintf((char*)txd_msg_buffer, "IN SETUP MODE\n\r");
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  sprintf((char*)txd_msg_buffer, "INLET SETUP:\n\r");
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  sprintf((char*)txd_msg_buffer, "MOTOR SPEED OPTIONS: 0 = MANUAL, 1 = 10%, 2 = 20%, 3 = 30%, 4 = 40%, 5 = 50%, 6 = 60%, 7 = 70%, 8 = 85%, 9 = 99%\n\r");
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == (00)){}
		  PWM_MODE_INLET = byte - 48;
		  sprintf((char*)txd_msg_buffer, "INLET PWM: %d\n\r", PWM_MODE_INLET);
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  sprintf((char*)txd_msg_buffer, "SCALED TIME OPTION: PICK HOURS FROM 1 TO 24 INCLUSIVE (MUST BE IN TWO-DIGIT FORM)\n\r");
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  rcv_intpt_flag = 00;
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == (00)){}
		  RUN_TIME_INLET = (byte -  48)*10;
		  rcv_intpt_flag = 00;
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == (00)){}
		  RUN_TIME_INLET += byte -  48;
		  rcv_intpt_flag = 00;
		  sprintf((char*)txd_msg_buffer, "INLET TIME: %d\n\r", RUN_TIME_INLET);
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  sprintf((char*)txd_msg_buffer, "ZONE1 SETUP:\n\r");
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  sprintf((char*)txd_msg_buffer, "MOTOR SPEED OPTIONS: 0 = MANUAL, 1 = 10%, 2 = 20%, 3 = 30%, 4 = 40%, 5 = 50%, 6 = 60%, 7 = 70%, 8 = 85%, 9 = 99%\n\r");
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == (00)){}
		  PWM_MODE_Z1 = byte - 48;
		  sprintf((char*)txd_msg_buffer, "ZONE1 PWM: %d\n\r", PWM_MODE_Z1);
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  sprintf((char*)txd_msg_buffer, "SCALED TIME OPTION: PICK HOURS FROM 1 TO 24 INCLUSIVE (MUST BE IN TWO-DIGIT FORM)\n\r");
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  rcv_intpt_flag = 00;
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == (00)){}
		  RUN_TIME_Z1 = (byte -  48)*10;
		  rcv_intpt_flag = 00;
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == (00)){}
		  RUN_TIME_Z1 += byte -  48;
		  rcv_intpt_flag = 00;
		  sprintf((char*)txd_msg_buffer, "ZONE1 TIME: %d\n\r", RUN_TIME_Z1);
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  sprintf((char*)txd_msg_buffer, "ZONE2 SETUP:\n\r");
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  sprintf((char*)txd_msg_buffer, "MOTOR SPEED OPTIONS: 0 = MANUAL, 1 = 10%, 2 = 20%, 3 = 30%, 4 = 40%, 5 = 50%, 6 = 60%, 7 = 70%, 8 = 85%, 9 = 99%\n\r");
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == (00)){}
		  PWM_MODE_Z2 = byte - 48;
		  sprintf((char*)txd_msg_buffer, "ZONE2 PWM: %d\n\r", PWM_MODE_Z2);
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  sprintf((char*)txd_msg_buffer, "SCALED TIME OPTION: PICK HOURS FROM 1 TO 24 INCLUSIVE (MUST BE IN TWO-DIGIT FORM)\n\r");
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  rcv_intpt_flag = 00;
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == (00)){}
		  RUN_TIME_Z2= (byte -  48)*10;
		  rcv_intpt_flag = 00;
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == (00)){}
		  RUN_TIME_Z2 += byte -  48;
		  rcv_intpt_flag = 00;
		  sprintf((char*)txd_msg_buffer, "ZONE2 TIME: %d\n\r", RUN_TIME_Z2);
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  sprintf((char*)txd_msg_buffer, "ZONE3 SETUP:\n\r");
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  sprintf((char*)txd_msg_buffer, "MOTOR SPEED OPTIONS: 0 = MANUAL, 1 = 10%, 2 = 20%, 3 = 30%, 4 = 40%, 5 = 50%, 6 = 60%, 7 = 70%, 8 = 85%, 9 = 99%\n\r");
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == (00)){}
		  PWM_MODE_Z3 = byte - 48;
		  sprintf((char*)txd_msg_buffer, "ZONE3 PWM: %d\n\r", PWM_MODE_Z3);
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  sprintf((char*)txd_msg_buffer, "SCALED TIME OPTION: PICK HOURS FROM 1 TO 24 INCLUSIVE (MUST BE IN TWO-DIGIT FORM)\n\r");
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  rcv_intpt_flag = 00;
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == (00)){}
		  RUN_TIME_Z3= (byte -  48)*10;
		  rcv_intpt_flag = 00;
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == (00)){}
		  RUN_TIME_Z3 += byte -  48;
		  rcv_intpt_flag = 00;
		  sprintf((char*)txd_msg_buffer, "ZONE3 TIME: %d\n\r", RUN_TIME_Z3);
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  SET_UP_FLAG = 1;
	  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim1.Init.Prescaler = 16-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 5000-1;
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
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.Pulse = 500-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim4.Init.Prescaler = 16-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 5000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500-1;
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

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 16000-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  huart6.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DEBUG_Pin|GREEN_Pin|DIGIT_B3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIGIT_A3_Pin|BLUE_Pin|DIGIT_B1_Pin|DIGIT_B0_Pin
                          |DIGIT_A2_Pin|DIGIT_A1_Pin|DIGIT_A0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIGIT_B2_GPIO_Port, DIGIT_B2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DEBUG_Pin GREEN_Pin DIGIT_B3_Pin */
  GPIO_InitStruct.Pin = DEBUG_Pin|GREEN_Pin|DIGIT_B3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RPM_TICK_Pin */
  GPIO_InitStruct.Pin = RPM_TICK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RPM_TICK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin RED_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DIGIT_A3_Pin BLUE_Pin DIGIT_B1_Pin DIGIT_B0_Pin
                           DIGIT_A2_Pin DIGIT_A1_Pin DIGIT_A0_Pin */
  GPIO_InitStruct.Pin = DIGIT_A3_Pin|BLUE_Pin|DIGIT_B1_Pin|DIGIT_B0_Pin
                          |DIGIT_A2_Pin|DIGIT_A1_Pin|DIGIT_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DIGIT_B2_Pin */
  GPIO_InitStruct.Pin = DIGIT_B2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIGIT_B2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1){
		us100_Rx_flag = 01;
	}

	if(huart->Instance == USART6)
	{
		rcv_intpt_flag = 01;
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == RPM_TICK_Pin){
		rpm_tick_count += 1;
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM5){
		clock_secs += 1;
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
