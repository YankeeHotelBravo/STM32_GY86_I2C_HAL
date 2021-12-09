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
#include "MPU6050.h"
#include "MadgwickAHRS.h"
#include "FS-iA6B.h"
#include "pid control.h"
#include "w25qxx.h"
#include "MS5611.h"

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
DMA_HandleTypeDef hdma_i2c1_rx;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */


// Timer variables
extern uint8_t tim7_1ms_flag;
extern uint8_t tim7_10ms_flag;
extern uint8_t tim7_100ms_flag;
extern uint8_t tim7_200ms_flag;
extern uint8_t tim7_500ms_flag;
extern uint8_t tim7_1000ms_flag;
extern float MPU6050_Roll;
extern float MPU6050_Yaw;
extern float MPU6050_Pitch;
extern uint8_t ibus_rx_cplt_flag;
extern uint8_t ibus_rx_buf[32];
extern uint8_t uart3_rx_data;

unsigned char failsafe_flag = 0;
unsigned int ccr1 ,ccr2, ccr3, ccr4;

unsigned char telemetry_tx_buf[35];

unsigned char Mag_Calib[6] = {0};

unsigned short iBus_SwA_Prev = 0;
unsigned char iBus_rx_cnt = 0;
unsigned char iBus_VrA_flag = 0;
unsigned char iBus_VrA_Prev_flag = 0;
unsigned char iBus_VrB_flag = 0;
unsigned char iBus_VrB_Prev_flag = 0;

unsigned char motor_arming_flag = 0;

unsigned char is_throttle_middle = 0;
unsigned char is_yaw_middle = 0;

float yaw_heading_reference = 0;
float altbaro = 0.0;



float alt_arr[10] = {0};
int alt_cnt = 0;

int mpu_read_busy = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM7_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
int _write(int file, char* p, int len)
{
	//	for(int i=0;i<len;i++)
	//	{
	//		while(!LL_USART_IsActiveFlag_TXE(USART1));
	//		LL_USART_TransmitData8(USART1, *(p+i));
	//	}
	HAL_UART_Transmit_DMA(&huart3, p, len);
}
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

	MPU6050.Gyro_X_Offset = -75;
	MPU6050.Gyro_Y_Offset = -25;
	MPU6050.Gyro_Z_Offset = 10;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	HAL_Delay(2000);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM7_Init();
  MX_UART5_Init();
  MX_TIM5_Init();
  MX_USART3_UART_Init();
  MX_SPI3_Init();
  MX_I2C1_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
	LL_USART_EnableIT_RXNE(UART5);
	LL_TIM_EnableCounter(TIM7); //10Hz, 50Hz, 1kHz loop
	LL_TIM_EnableIT_UPDATE(TIM7);

	HAL_UART_Receive_IT(&huart3, &uart3_rx_data, 1); // Telemetry

	LL_TIM_EnableCounter(TIM5); //Motor PWM
	LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH1); //Enable Timer Counting
	LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH2); //Enable Timer Counting
	LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH3); //Enable Timer Counting
	LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH4); //Enable Timer Counting

	/// BAROMETER
	MS5611_Reset(&hi2c1, &MS5611);
	MS5611_ReadProm(&hi2c1, &MS5611);

	// MPU6050 + HMC5883L
	MPU6050_Init(&hi2c1, 2, 2, 5);
	MPU6050_Bypass(&hi2c1);
	HMC5883L_Setup(&hi2c1);
	MPU6050_Master(&hi2c1);
	MPU6050_Slave_Read(&hi2c1);

	//EEPROM
	W25qxx_Init();
	HAL_Delay(100);
//	W25qxx_EraseChip();

	//Receiver Off
	while(Is_iBus_Received() == 0)
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
		HAL_Delay(500);
	}

	//Magnetometer Calibration
	if(iBus.SwC == 1500)
	{
		for(int i =0;i<20;i++)
		{
			MPU6050_Read_All_DMA(&hi2c1, &MPU6050);
			HAL_Delay(20);
			MPU6050_Parsing_NoOffest(&MPU6050);
		}
		MPU6050.Mag_X_Max = MPU6050.Mag_X_RAW;
		MPU6050.Mag_X_Min = MPU6050.Mag_X_RAW;
		MPU6050.Mag_Y_Max = MPU6050.Mag_Y_RAW;
		MPU6050.Mag_Y_Min = MPU6050.Mag_Y_RAW;
		MPU6050.Mag_Z_Max = MPU6050.Mag_Z_RAW;
		MPU6050.Mag_Z_Min = MPU6050.Mag_Z_RAW;

		while(iBus.SwC != 1000)
		{
			Is_iBus_Received();
			MPU6050_Read_All_DMA(&hi2c1, &MPU6050);
			HAL_Delay(20);
			MPU6050_Parsing_NoOffest(&MPU6050);
			if(MPU6050.Mag_X_RAW > MPU6050.Mag_X_Max) MPU6050.Mag_X_Max = MPU6050.Mag_X_RAW;
			if(MPU6050.Mag_X_RAW < MPU6050.Mag_X_Min) MPU6050.Mag_X_Min = MPU6050.Mag_X_RAW;

			if(MPU6050.Mag_Y_RAW > MPU6050.Mag_Y_Max) MPU6050.Mag_Y_Max = MPU6050.Mag_Y_RAW;
			if(MPU6050.Mag_Y_RAW < MPU6050.Mag_Y_Min) MPU6050.Mag_Y_Min = MPU6050.Mag_Y_RAW;

			if(MPU6050.Mag_Z_RAW > MPU6050.Mag_Z_Max) MPU6050.Mag_Z_Max = MPU6050.Mag_Z_RAW;
			if(MPU6050.Mag_Z_RAW < MPU6050.Mag_Z_Min) MPU6050.Mag_Z_Min = MPU6050.Mag_Z_RAW;
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
		}
		MPU6050.Mag_X_Offset = (MPU6050.Mag_X_Max + MPU6050.Mag_X_Min) / 2;
		MPU6050.Mag_Y_Offset = (MPU6050.Mag_Y_Max + MPU6050.Mag_Y_Min) / 2;
		MPU6050.Mag_Z_Offset = (MPU6050.Mag_Z_Max + MPU6050.Mag_Z_Min) / 2;

		Mag_Calib[0] = MPU6050.Mag_X_Offset >> 8;
		Mag_Calib[1] = MPU6050.Mag_X_Offset;
		Mag_Calib[2] = MPU6050.Mag_Y_Offset >> 8;
		Mag_Calib[3] = MPU6050.Mag_Y_Offset;
		Mag_Calib[4] = MPU6050.Mag_Z_Offset >> 8;
		Mag_Calib[5] = MPU6050.Mag_Z_Offset;

		W25qxx_WritePage(Mag_Calib, 0, 0, 6);
	}

	HAL_Delay(200);
	W25qxx_ReadPage(Mag_Calib, 0, 0, 6);
	MPU6050.Mag_X_Offset = Mag_Calib[0] << 8 | Mag_Calib[1];
	MPU6050.Mag_Y_Offset = Mag_Calib[2] << 8 | Mag_Calib[3];
	MPU6050.Mag_Z_Offset = Mag_Calib[4] << 8 | Mag_Calib[5];

	//ESC Calibration
	if(iBus.SwC == 2000)
	{
		ESC_Calibration();
		while(iBus.SwC != 1000)
		{
			Is_iBus_Received();
			LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
			TIM3->PSC = 1500;
			HAL_Delay(200);
			TIM3->PSC = 2000;
			HAL_Delay(200);
			LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
		}
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(ibus_rx_cplt_flag==1)
		{
			ibus_rx_cplt_flag =0;
			if(iBus_Check_CHKSUM(ibus_rx_buf, 32))
			{
				iBus_Parsing(ibus_rx_buf , &iBus);
			}
		}

		//if(tim7_10ms_flag==0 && tim7_1ms_flag==1)
		if(tim7_1ms_flag==1)
		{
			tim7_1ms_flag = 0;
			mpu_read_busy = 1;
			MPU6050_Read_All_DMA(&hi2c1, &MPU6050);
			MPU6050_Parsing(&MPU6050);
			MadgwickAHRSupdate(MPU6050.Gx, MPU6050.Gy, MPU6050.Gz, MPU6050.Ax, MPU6050.Ay, MPU6050.Az,MPU6050.Mag_X_RAW , MPU6050.Mag_Y_RAW,MPU6050.Mag_Z_RAW);

			Double_Roll_Pitch_PID_Calculation(&pitch, -(iBus.RV - 1500)*0.07f, MPU6050_Pitch, MPU6050.Gx);
			Double_Roll_Pitch_PID_Calculation(&roll, (iBus.RH - 1500)*0.07f, MPU6050_Roll, MPU6050.Gy);

			if(iBus.LH > 1485 && iBus.LH < 1515) is_yaw_middle = 1;
			else is_yaw_middle = 0;
			//			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);

			if(is_yaw_middle == 0)
			{
				yaw_heading_reference = MPU6050_Yaw;
				Single_Yaw_Rate_PID_Calculation(&yaw_rate, (iBus.LH-1500), MPU6050.Gz);
				ccr1 = 84000 + (iBus.LV - 1000) * 83.9 + pitch.in.pid_result - roll.in.pid_result - yaw_rate.pid_result;
				ccr2 = 84000 + (iBus.LV - 1000) * 83.9 - pitch.in.pid_result - roll.in.pid_result + yaw_rate.pid_result;
				ccr3 = 84000 + (iBus.LV - 1000) * 83.9 - pitch.in.pid_result + roll.in.pid_result - yaw_rate.pid_result;
				ccr4 = 84000 + (iBus.LV - 1000) * 83.9 + pitch.in.pid_result + roll.in.pid_result + yaw_rate.pid_result;
			}
			else
			{
				Double_Yaw_Heading_PID_Calculation(&yaw_heading, yaw_heading_reference, MPU6050_Yaw,  MPU6050.Gz);
				ccr1 = 84000 + (iBus.LV - 1000) * 83.9 + pitch.in.pid_result - roll.in.pid_result - yaw_heading.in.pid_result;
				ccr2 = 84000 + (iBus.LV - 1000) * 83.9 - pitch.in.pid_result - roll.in.pid_result + yaw_heading.in.pid_result;
				ccr3 = 84000 + (iBus.LV - 1000) * 83.9 - pitch.in.pid_result + roll.in.pid_result - yaw_heading.in.pid_result;
				ccr4 = 84000 + (iBus.LV - 1000) * 83.9 + pitch.in.pid_result + roll.in.pid_result + yaw_heading.in.pid_result;
			}
		}
		//		else if(tim7_10ms_flag==1 && mpu_read_busy == 0)
		if(tim7_10ms_flag==1 && mpu_read_busy == 0)
		{
			tim7_10ms_flag=0;

			if(alt_cnt == 0)
			{
				MS5611_RequestTemperature(&hi2c1,OSR_4096);
				alt_cnt = 1;
			}

			else if (alt_cnt == 1)
			{
				MS5611_ReadTemperature(&hi2c1,&MS5611);
				MS5611_CalculateTemperature(&MS5611);
				MS5611_RequestPressure(&hi2c1, OSR_4096);
				alt_cnt = 2;
			}
			else
			{
				MS5611_ReadPressure(&hi2c1,&MS5611);
				MS5611_CalculatePressure(&MS5611);
				MS5611_RequestTemperature(&hi2c1,OSR_4096);
				MS5611.Alt = (MS5611_getAltitude1((float)MS5611.P/100.f))*100;

				#define X 0.90f
				MS5611.Alt_Filt = MS5611.Alt_Filt * X + MS5611.Alt * (1.0f-X);
				alt_cnt = 1;

//				printf("%.1f \t %.1f \n", MS5611.Alt, MS5611.Alt_Filt);
					printf("%.1f \t %.1f %.1f \t \n", MPU6050_Roll, MPU6050_Pitch, MPU6050_Yaw);

				//	printf("%d \t %.d\t %d \t %d \t %d \t %d \t \n", MPU6050.Mag_X_RAW,MPU6050.Mag_Y_RAW,MPU6050.Mag_Z_RAW, MPU6050.Mag_X_Offset,MPU6050.Mag_Y_Offset, MPU6050.Mag_Z_Offset);
				//	printf("%d \t %d \t %d \t  \n", MPU6050.Mag_X_RAW,MPU6050.Mag_Y_RAW,MPU6050.Mag_Z_RAW);
				//	printf("%.1f \t %.1f %.1f \t %.1f \t \n", MPU6050_Roll, MPU6050_Pitch, MPU6050_Yaw , MS5611.Alt_Filt);
			}
		}

		if(iBus.LV < 1030 || motor_arming_flag == 0)
		{
			Reset_All_PID_Integrator();
		}

		if(iBus.SwA == 2000 && iBus_SwA_Prev != 2000)
		{
			if(iBus.LV < 1010)
			{
				motor_arming_flag = 1;
				yaw_heading_reference = MPU6050_Yaw;
			}
		}
		iBus_SwA_Prev = iBus.SwA;

		if(iBus.SwA != 2000)
		{
			motor_arming_flag = 0;
		}
		if(motor_arming_flag == 1)
		{
			if(failsafe_flag == 0)
			{
				if(iBus.LV > 1050)
				{
					TIM5->CCR1 = ccr1 > 167900 ? 167900 : ccr1 < 84000 ? 84000 : ccr1;
					TIM5->CCR2 = ccr2 > 167900 ? 167900 : ccr2 < 84000 ? 84000 : ccr2;
					TIM5->CCR3 = ccr3 > 167900 ? 167900 : ccr3 < 84000 ? 84000 : ccr3;
					TIM5->CCR4 = ccr4 > 167900 ? 167900 : ccr4 < 84000 ? 84000 : ccr4;

				}
				else
				{
					TIM5->CCR1 = 84000;
					TIM5->CCR2 = 84000;
					TIM5->CCR3 = 84000;
					TIM5->CCR4 = 84000;
				}
			}
			else
			{
				TIM5->CCR1 = 84000;
				TIM5->CCR2 = 84000;
				TIM5->CCR3 = 84000;
				TIM5->CCR4 = 84000;
			}
		}
		else
		{
			TIM5->CCR1 = 84000;
			TIM5->CCR2 = 84000;
			TIM5->CCR3 = 84000;
			TIM5->CCR4 = 84000;
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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 167999;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM7);

  /* TIM7 interrupt Init */
  NVIC_SetPriority(TIM7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM7_IRQn);

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  TIM_InitStruct.Prescaler = 41999;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 1;
  LL_TIM_Init(TIM7, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM7);
  LL_TIM_SetTriggerOutput(TIM7, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM7);
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART5);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
  /**UART5 GPIO Configuration
  PC12   ------> UART5_TX
  PD2   ------> UART5_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* UART5 interrupt Init */
  NVIC_SetPriority(UART5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(UART5_IRQn);

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(UART5, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(UART5);
  LL_USART_Enable(UART5);
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|FLASH_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA8 FLASH_CS_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_8|FLASH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Encode_Msg_Motor(unsigned char* telemetry_tx_buf)
{
	telemetry_tx_buf[0] = 0x88;
	telemetry_tx_buf[1] = 0x18;

	telemetry_tx_buf[2] = ccr1 >> 24;
	telemetry_tx_buf[3] = ccr1 >> 16;
	telemetry_tx_buf[4] = ccr1 >> 8;
	telemetry_tx_buf[5] = ccr1;

	telemetry_tx_buf[6] = ccr2 >> 24;
	telemetry_tx_buf[7] = ccr2 >> 16;
	telemetry_tx_buf[8] = ccr2 >> 8;
	telemetry_tx_buf[9] = ccr2;

	telemetry_tx_buf[10] = ccr3 >> 24;
	telemetry_tx_buf[11] = ccr3 >> 16;
	telemetry_tx_buf[12] = ccr3 >> 8;
	telemetry_tx_buf[13] = ccr3;

	telemetry_tx_buf[14] = ccr4 >> 24;
	telemetry_tx_buf[15] = ccr4 >> 16;
	telemetry_tx_buf[16] = ccr4 >> 8;
	telemetry_tx_buf[17] = ccr4;

	telemetry_tx_buf[18] = iBus.LV >> 8;
	telemetry_tx_buf[19] = iBus.LV;
	//
	//     telemetry_tx_buf[20] = ((int)pitch.in.pid_result) >> 24;
	//     telemetry_tx_buf[21] = ((int)pitch.in.pid_result) >> 16;
	//     telemetry_tx_buf[22] = ((int)pitch.in.pid_result) >> 8;
	//     telemetry_tx_buf[23] = ((int)pitch.in.pid_result);
	//
	//     telemetry_tx_buf[24] = ((int)roll.in.pid_result) >> 24;
	//     telemetry_tx_buf[25] = ((int)roll.in.pid_result) >> 16;
	//     telemetry_tx_buf[26] = ((int)roll.in.pid_result) >> 8;
	//     telemetry_tx_buf[27] = ((int)roll.in.pid_result);
	//
	//     telemetry_tx_buf[28] = ((int)yaw_heading.in.pid_result) >> 24;
	//     telemetry_tx_buf[29] = ((int)yaw_heading.in.pid_result) >> 16;
	//     telemetry_tx_buf[30] = ((int)yaw_heading.in.pid_result) >> 8;
	//     telemetry_tx_buf[31] = ((int)yaw_heading.in.pid_result);

	//     telemetry_tx_buf[32] = ((int)altitude_filt) >> 24;
	//     telemetry_tx_buf[33] = ((int)altitude_filt) >> 16;
	//     telemetry_tx_buf[34] = ((int)altitude_filt) >> 8;
	//     telemetry_tx_buf[35] = ((int)altitude_filt);

	//     telemetry_tx_buf[32] = ((int)actual_pressure_fast) >> 24;
	//     telemetry_tx_buf[33] = ((int)actual_pressure_fast) >> 16;
	//     telemetry_tx_buf[34] = ((int)actual_pressure_fast) >> 8;
	//     telemetry_tx_buf[35] = ((int)actual_pressure_fast);
}

void assign_int_buf(unsigned char* buf, int start, int var){
	for(int i = 0 ; i < 4; i++){
		buf[start + i] = var >>(24-8*i);
	}
}

int Is_iBus_Received(void)
{
	if(ibus_rx_cplt_flag==1)
	{
		ibus_rx_cplt_flag=0;
		if(iBus_Check_CHKSUM(&ibus_rx_buf[0],32)==1)
		{
			iBus_Parsing(&ibus_rx_buf[0], &iBus);
			return 1;
		}
	}
	return 0;
}

void ESC_Calibration(void)
{
	TIM5->CCR1 = 167999;
	TIM5->CCR2 = 167999;
	TIM5->CCR3 = 167999;
	TIM5->CCR4 = 167999;
	HAL_Delay(7000);

	TIM5->CCR1 = 84000;
	TIM5->CCR2 = 84000;
	TIM5->CCR3 = 84000;
	TIM5->CCR4 = 84000;
	HAL_Delay(8000);
}

void HAL_I2C_MemRxCpltCallback (I2C_HandleTypeDef * hi2c)
{
	if(hi2c->Instance == I2C1)
	{
		mpu_read_busy = 0;
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

