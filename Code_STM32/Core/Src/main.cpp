/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
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
  ******************************I************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>

#include "bmp388.h"
#include "usbd_cdc_if.h"

//includes for IMU
#include "icm42605.h"
#include "kalman.h"
#include "matrix_math.h"
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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi3_rx;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */
using namespace std;
int vector_size = 2;			//size of square matrix for Kalman Filter
//float n = 97530; 				//threshold to detect touch;

float kalman_roll, kalman_pitch;
float pressure_fp[8][3], pressure_bp[8][3];
int chip_select = 0;
int j = 0;						//Switch statement

uint8_t buf[40];

//Kalman Filter matrices
matrix Q = {{0.005, 0}, {0, 0.005}};				//Process noise covariance matrix
matrix A = {{1, 0}, {0, 1}};						//State transition matrix
matrix B = {{0.004, 0}, {0, 0.004}};				//Control input matrix, dt
matrix x_e = {{10, 0}, {0, 10}};					//initial value
matrix p_e = {{0.10, 0}, {0, 0.10}};				//State error variance
matrix I = {{1, 0}, {0, 1}};						//Identity matrix
matrix H = {{1, 0}, {0, 1}};						//Measurement matrix
matrix R(vector_size , matrix_1D (vector_size, 0));	//Measurement noise matrix

uint16_t CS_1 = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
uint16_t CS_2 = GPIO_PIN_1 | GPIO_PIN_2;
uint16_t CS_3 = GPIO_PIN_0 | GPIO_PIN_2;
uint16_t CS_4 = GPIO_PIN_2;
uint16_t CS_5 = GPIO_PIN_0 | GPIO_PIN_1;
uint16_t CS_6 = GPIO_PIN_1;
uint16_t CS_7 = GPIO_PIN_0;
uint16_t CS_8_1 = GPIO_PIN_3; //Decoder1 Enable Pin
uint16_t CS_8_2 = GPIO_PIN_4; //Decoder2 Enable Pin

//storing chip selects in arrays. fp-> front panel, bp-> back panel
uint16_t chip_select_fp[8] = {CS_1, CS_2, CS_3, CS_4, CS_5, CS_6, CS_7, CS_8_1};
uint16_t chip_select_bp[8] = {CS_1, CS_2, CS_3, CS_4, CS_5, CS_6, CS_7, CS_8_2};

//Class definitions for sensors on the front back panel
bmp388 bmp388_fp[8];
bmp388 bmp388_bp[8];


float cop_y;
class cop_t
{
public:
	float cop_panel;

	cop_t(){};
	~cop_t(){};

	void centre_of_pressure(float p[], float y[]);

	float distance_y_bp[3] = {1.5, 3.75, 6};
	float distance_y_fp[3] = {109, 111.25, 113.5};
	float cop_numerator, cop_denominator = 0;
};

cop_t cop_fp_y;	//front panel centre of pressure
cop_t cop_bp_y;	//back panel centre of pressure

//Class definitions for IMU
kf_math kf_math;
ICM42605 ICM42605;
Kalman kf(Q, A, B, x_e, p_e, I, H);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
extern "C" void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM6_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM7_Init(void);

/* USER CODE BEGIN PFP */
void front_panel_decoder_enable(void);
void back_panel_decoder_enable(void);
void shape_of_contact(float[]);
void delay_us (uint16_t us);
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
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM17_Init();
  MX_TIM6_Init();
  MX_USB_Device_Init();
  MX_I2C1_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  ICM42605.Init();
  ICM42605.GyroCalibrate();

  //Set all decoder outputs high
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);

  front_panel_decoder_enable();

  //Initialize and calculate coeffecients of the sensors on the front panel
  for (int i=0; i<8; i++)
  {
	  bmp388_fp[i].chip_select(chip_select_fp[i]);
	  bmp388_fp[i].spi(&hspi1, &hspi2, &hspi3);
	  bmp388_fp[i].init();
	  bmp388_fp[i].calculate_coeff();
	  HAL_Delay(1);
  }

  back_panel_decoder_enable();

  //Initialize and calculate coeffecients of the sensors on the back panel
  for (int i=0; i<8; i++)
  {
	  bmp388_bp[i].chip_select(chip_select_bp[i]);
	  bmp388_bp[i].spi(&hspi1, &hspi2, &hspi3);
	  bmp388_bp[i].init();
	  bmp388_bp[i].calculate_coeff();
	  HAL_Delay(1);
  }

  front_panel_decoder_enable();
  HAL_TIM_Base_Start(&htim7);				//1 us timer for gap between USB transmissions
  HAL_TIM_Base_Start_IT(&htim17);			//Timer for measurement cycle


  /* USER CODE END 2 */

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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 18;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_USB;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x20B0D9FF;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_SLAVE;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 512;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 30;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 144-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 65;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 2180;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim17)
	{
		HAL_TIM_Base_Start_IT(&htim6);
	}
	if (htim == &htim6)
	{
		switch (j)
		{

		//Sensors readout
		case 0:
			for(chip_select = 0; chip_select <8; chip_select++)
				bmp388_fp[chip_select].readout();

			back_panel_decoder_enable();

			for(chip_select = 0; chip_select <8; chip_select++)
				bmp388_bp[chip_select].readout();

			front_panel_decoder_enable();
			j++;
			break;

		//Compensate the sensors on the front panel and transmit data via USB
		case 1:
			delay_us(100);
			sprintf((char*)buf, "%s \r\n","Front Panel:");
			CDC_Transmit_FS((uint8_t*)buf, strlen((char*)buf));

			for (chip_select = 0; chip_select < 8; chip_select++)
			{
				bmp388_fp[chip_select].compensate();
				cop_fp_y.centre_of_pressure(bmp388_fp[chip_select].pressure_, cop_fp_y.distance_y_fp);

				//Uncomment for the shape of contact
				//shape_of_contact(bmp388_fp[chip_select].pressure_);

				sprintf((char*)buf, " %d %0.4f %0.4f %0.4f \r\n", chip_select+1, bmp388_fp[chip_select].pressure_[0], bmp388_fp[chip_select].pressure_[1], bmp388_fp[chip_select].pressure_[2]);
				CDC_Transmit_FS((uint8_t*)buf, strlen((char*)buf));
				pressure_fp[chip_select][0] = bmp388_fp[chip_select].pressure_[0];
				pressure_fp[chip_select][1] = bmp388_fp[chip_select].pressure_[1];
				pressure_fp[chip_select][2] = bmp388_fp[chip_select].pressure_[2];
				delay_us(100);
			}
			j++;
			break;

		//Compensate the sensors on the back panel and transmit data via USB
		case 2:
			delay_us(100);

			sprintf((char*)buf, "%s \r\n","Back Panel:");
			CDC_Transmit_FS((uint8_t*)buf, strlen((char*)buf));

			for (chip_select = 0; chip_select < 8; chip_select++)
			{
				bmp388_bp[chip_select].compensate();
				cop_bp_y.centre_of_pressure(bmp388_bp[chip_select].pressure_, cop_bp_y.distance_y_bp);

				//Uncomment for the shape of contact
				//shape_of_contact(bmp388_bp[chip_select].pressure_);

				sprintf((char*)buf, " %d %0.4f %0.4f %0.4f \r\n", chip_select+1, bmp388_bp[chip_select].pressure_[0], bmp388_bp[chip_select].pressure_[1], bmp388_bp[chip_select].pressure_[2]);
				CDC_Transmit_FS((uint8_t*)buf, strlen((char*)buf));

				pressure_bp[chip_select][0] = bmp388_bp[chip_select].pressure_[0];
				pressure_bp[chip_select][1] = bmp388_bp[chip_select].pressure_[1];
				pressure_bp[chip_select][2] = bmp388_bp[chip_select].pressure_[2];
				delay_us(100);
			}
			delay_us(100);
			cop_y = (cop_fp_y.cop_panel + cop_bp_y.cop_panel) / 2;
			sprintf((char*)buf, "%s %0.2f \r\n","CoP along the foot length:", cop_y);
			CDC_Transmit_FS((uint8_t*)buf, strlen((char*)buf));
			j++;
			break;

		//IMU interface
		case 3:
			 ICM42605.ReadAcc();
			 ICM42605.ReadGyro();

			 R = {{kf_math.variance(ICM42605.acc_[0][0], 100), 0},
			   {0, kf_math.variance(ICM42605.acc_[1][1], 100)}};

			 kf.Prediction(ICM42605.gyro_);	//Control vector
			 kf.KalmanGain(R);				//Measurement noise vector
			 kf.Update(ICM42605.acc_);

			 //Convert from radians to degrees
			 kalman_pitch = kf.x_e_[0][0]*57.296f;
			 kalman_roll = kf.x_e_[1][1]*57.296f;

			 sprintf((char*)buf, "%s %0.2f %s %0.2f \r\n","Roll:", kalman_roll, "Pitch:", kalman_pitch);
			 CDC_Transmit_FS((uint8_t*)buf, strlen((char*)buf));

			 j=0;

			 HAL_TIM_Base_Stop_IT(&htim6);
			 break;
		}
	}
}

void front_panel_decoder_enable()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); 	//Back panel Decoder disable
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET); 	//Front panel Decoder enable
}

void back_panel_decoder_enable()
{

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);	//Back panel Decoder enable
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); 	//Front panel Decoder disable
}


//1 microsecond delay
void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim7,0);  			// set the counter value at 0
	while (__HAL_TIM_GET_COUNTER(&htim7) < us);  // wait for the counter to reach the us input in the parameter
}

//Uncomment for the shape of contact
/*void shape_of_contact(float x[])
{
	for (int i = 0; i < 3; i ++)
	{
		if (x[i] > n)
			x[i] = 1;
		else
			x[i] = 0;
	}
}
*/

void cop_t::centre_of_pressure(float p[], float y[])
{
	cop_numerator = (p[0] * y[0] + p[1] * y[1] + p[2] * y[2]);
	cop_denominator =  p[0] + p[1] + p[2];
	cop_panel = cop_numerator / cop_denominator;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
