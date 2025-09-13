/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************<
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "nmea_parse.h"
#include "bno055.h"
#include "cc1101.h"
#include "quaternion.h"
#include "ibus.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RAD2DEG  180.0f / 3.1415f

#define ELEVATOR_TRIM (int16_t)1472
#define AILERON_TRIM  (int16_t)1437
#define RUDDER_TRIM   (int16_t)1462

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
char rx_buff_gps[83]; // sentence buffer - NMEA messages are at most 82 chars long
char rx_char_gps;
int rx_i_gps = 0;
char nmea_sentence[83]; // actual sentence

GPS gps;

uint8_t rx_buff_ibus[32]; // start - 14 channels - checksum (circular buffer)
uint16_t channels[14] = {1500, 1500, 1000, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

struct Packet p;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_I2C2_Init();
  MX_UART4_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  TIM8->CCR1 = 1500;
  TIM3->CCR2 = 2000;

  HAL_TIM_Base_Start(&htim2);  // Start TIM2 in polling mode

  BNO055_Handle_t bno055;
  BNO055_Quaternion_t quat;
  BNO055_LinearAccel_t accel;
  BNO055_Euler_t euler;
  BNO055_AngularVelocity_t ang_vel;

  // Initialize BNO055
  HAL_Delay(500); // NECESSARY delay
  HAL_StatusTypeDef ret;
  ret = BNO055_Init(&bno055, &hi2c2, BNO055_I2C_ADDR1);

  if(ret != HAL_OK)
  {
      // Handle error
  }

  uint32_t error;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  if (error == HAL_I2C_ERROR_NONE) {

  } else if (error == HAL_I2C_ERROR_BERR) {
    printf("HAL_I2C_ERROR_BERR\r\n");
  } else if (error == HAL_I2C_ERROR_ARLO) {
    printf("HAL_I2C_ERROR_ARLO\r\n");
  } else if (error == HAL_I2C_ERROR_AF) {
    printf("HAL_I2C_ERROR_AF\r\n");
  } else if (error == HAL_I2C_ERROR_OVR) {
    printf("HAL_I2C_ERROR_OVR\r\n");
  } else if (error == HAL_I2C_ERROR_DMA) {
    printf("HAL_I2C_ERROR_DMA\r\n");
  } else if (error == HAL_I2C_ERROR_TIMEOUT) {
    printf("HAL_I2C_ERROR_TIMEOUT\r\n");
  }



  HAL_UART_Receive_IT(&huart4, &rx_char_gps, 1);
  HAL_UART_Receive_DMA(&huart1, rx_buff_ibus, 32);

  uint8_t status;
  uint8_t marcstate;

  Power_up_reset();

  TI_init(&hspi1, GPIOB, GPIO_PIN_6);

  status = TI_read_status(CCxxx0_VERSION); // it is for checking only
  uint32_t last_transmission = 0; // when was the last packet sent?

  struct Quaternion quat_axis_remap =     {0.0,  1.0f, 0.0, 0.0}; //  z -> -z,  y -> -y
  struct Quaternion quat_axis_remap_inv = {0.0, -1.0f, 0.0, 0.0}; // -z ->  z, -y ->  y
  struct Quaternion quat_rotate_azimuth = {0.70710678118, 0, 0, 0.70710678118}; // rotate azimuth by 90 deg
  struct Quaternion quat_raw;
  struct Quaternion temp;

  // Example target location: Ljubljana Castle
  float target_lat = 46.048794785175716;
  float target_long = 14.508673701361634;

  // Example target location: Rijeka
  //float target_lat = 45.3366306682363;
  //float target_long = 14.395428799781492;

  // Example target location: Vienna
  // float target_lat = 48.205403696400346;
  // float target_long = 16.394818810035325;

  float bearing = 0.0f;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	uint32_t ms = __HAL_TIM_GET_COUNTER(&htim2);
	p.time = ms;

	if(ms - last_transmission > 30){
	    TI_strobe(CCxxx0_SFTX); // flush the buffer
	    status = TI_read_status(CCxxx0_TXBYTES);
	    TI_send_packet((void*) &p, sizeof(struct Packet));
	    last_transmission = ms;
	}

    HAL_Delay(3);

    status = TI_read_status(CCxxx0_TXBYTES);
    marcstate = TI_read_status(CCxxx0_MARCSTATE);

	if(BNO055_ReadQuaternion(&bno055, &quat) == HAL_OK)
	{
	    // Use quaternion data (quat.w, quat.x, quat.y, quat.z)
		quat_raw.w = quat.w;
		quat_raw.x = quat.x;
		quat_raw.y = quat.y;
		quat_raw.z = quat.z;

		// Transform vector into original space (a_inv), apply q, transform back into re-mapped space (a)
		// a * q * a_inv
		temp = quaternion_multiply(&quat_axis_remap, &quat_raw);
		temp = quaternion_multiply(&temp, &quat_axis_remap_inv);

		// Rotate azimuth by 90 degrees, to align vector to NED
		temp = quaternion_multiply(&quat_rotate_azimuth, &temp);

		// Set quaternion
		p.q = temp;

		BNO055_QuaternionToEuler(&p.q, &euler);

	}



    // Read linear acceleration
    if(BNO055_ReadLinearAccel(&bno055, &accel) == HAL_OK)
    {
        // Use acceleration data (accel.x, accel.y, accel.z in m/s^2)
    	p.ax = accel.x;
    	p.ay = accel.y;
    	p.az = accel.z;
    }


    if(BNO055_ReadAngularVelocity(&bno055, &ang_vel) == HAL_OK) {

    }

    // Parse GPS
	nmea_parse(&gps, nmea_sentence);

    if (gps.fix == 1){
    	p.latitude = gps.latitude;
    	p.longitude = gps.longitude;
    	p.altitude = gps.altitude;
    	p.satellites = gps.satelliteCount;

    	// Calculate bearing from target position and current position
    	bearing = RAD2DEG * atan2(
    			sin((target_long - p.longitude) / RAD2DEG) * cos(target_lat / RAD2DEG),
				cos(p.latitude / RAD2DEG) * sin(target_lat / RAD2DEG) - sin(p.latitude / RAD2DEG) * cos(target_lat / RAD2DEG) * cos((target_long - p.longitude)/RAD2DEG)
    	);

    }

    // Parse iBus
	parse_ibus(rx_buff_ibus, channels);

	// Choose control law
	// Mode 0 - Manual
	if      (channels[4] == 1000 && channels[5] == 1000 && channels[6] == 1000){
		p.mode = 0;
	}
	// Mode 1 - Take-off
	else if (channels[4] == 2000 && channels[5] == 1000 && channels[6] == 1000 && (p.mode == 0 || p.mode == 1)){
		p.mode = 1;

		// Also enter recovery mode if attitude control sticks are not centered
		if (channels[0] < 1400 || channels[0] > 1600){
			p.mode = 255;
		}
		if (channels[1] < 1400 || channels[1] > 1600){
			p.mode = 255;
		}
		if (channels[3] < 1400 || channels[3] > 1600){
			p.mode = 255;
		}
	}

	// Mode 255 - Recovery
	else {
		p.mode = 255;
	}


	// Respond to mode
	if (p.mode == 0 || p.mode == 255){
		// Map channels directly
	    for (int i = 0; i < 7; i++){
	    	p.channels[i] = channels[i];
	    }
	}else if (p.mode == 1){
		p.channels[0] = AILERON_TRIM  + (int16_t) (500.0f*((euler.roll*RAD2DEG  -  0.0f)*0.0075f + (ang_vel.x)*0.0020f));
		p.channels[1] = ELEVATOR_TRIM + (int16_t) (500.0f*((euler.pitch*RAD2DEG - 10.0f)*0.0100f - (ang_vel.y)*0.0020f));
		p.channels[3] = RUDDER_TRIM;

		// Map channels 4-7 directly
		for(int i = 4; i < 7; i++){
			p.channels[i] = channels[i];
		}
	}

	// Limit channel values
	for (int i = 0; i < 7; i++){
		if (p.channels[i] < 1000){
			p.channels[i] = 1000;
		}
		if (p.channels[i] > 2000){
			p.channels[i] = 2000;
		}
	}

    // Apply actuation values
	TIM8->CCR1 = p.channels[0] - 500;
    TIM8->CCR2 = p.channels[1] - 500;
    TIM8->CCR3 = p.channels[2] - 500;
    TIM8->CCR4 = p.channels[3] - 500;

    TIM3->CCR1 = p.channels[4] - 500;
    TIM3->CCR2 = p.channels[5] - 500;



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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim2.Init.Prescaler = 25000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 25-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
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
  sConfigOC.Pulse = 1500;
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
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 25-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 20000-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 1500;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

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
  huart4.Init.BaudRate = 9600;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if(huart->Instance == USART1){
		// Do nothig - we are using circular buffer
	}else if(huart->Instance == UART4){

		HAL_UART_Receive_IT(&huart4, &rx_char_gps, 1);

		// Handle GPS RX
		if(rx_char_gps == '$'){
			rx_buff_gps[rx_i_gps + 1] = '\r';
			rx_buff_gps[rx_i_gps + 2] = '\n';
			rx_buff_gps[rx_i_gps + 3] = '\0';

			rx_i_gps = 0;

			memcpy(nmea_sentence, rx_buff_gps, 83);

			for (int i = 0; i < 83; i++){
				rx_buff_gps[i] = '\0';
			}

		}

		if(rx_i_gps >= 83){
			rx_i_gps = 0;
		}

		rx_buff_gps[rx_i_gps] = rx_char_gps;
		rx_i_gps += 1;
	}
}

int __io_putchar(int ch)
{
 // Write character to ITM ch.0
 ITM_SendChar(ch);
 return(ch);
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
