/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct __potentio_rdc80 {
	double degree;
	int32_t half_rotation; //正方向に180degごとに1加算。
	double initial_degree;
	uint16_t phase_a;
	uint16_t phase_b;
} potentio_rdc80;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//セッティング用定数
//グリッパーの目標角度[deg]
#define TARGET_GRIPPER_DEG 30.0
//z軸の目標下げ変位[mm]
#define TARGET_Z_DISP 86.0


//ADC
#define ADC_ARR_SIZE 7
#define ADC_12BIT_MAX 4095
//RDC80
#define ADC_RDC80_DEG_CONST 0.0830 //12bitのADC値から、角度に変換するための定数。
#define ADC_RDC80_MIN 1023-10 //この値未満になった時に、計測する相を切り替える。
#define ADC_RDC80_MAX 3071+10 //この値より上になった時に、計測する相を切り替える。
#define ADC_INIT_VAL_RDC80_ZA 3492
#define ADC_INIT_VAL_RDC80_ZB 1329 //こちらが最初に計測する相
#define ADC_INIT_DEG_Z -29.84131 +180.0 //初期角度(ADの実測値からの計算)
#define ADC_INIT_VAL_RDC80_GA 164
#define ADC_INIT_VAL_RDC80_GB 2152 //こちらが最初に計測する相
#define ADC_INIT_DEG_G 8.63281 - 180.0//初期角度(ADの実測値からの計算)
//(本当は-172degだが、phaseの初期値を-1するためこの角度)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
__IO uint16_t adc_vals[ADC_ARR_SIZE];
potentio_rdc80 prdZ, prdG;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
//void adc2deg_rdc80(potentio_rdc80 *);
void adc2deg_rdc80Z(void);
void driveZ(double);
void adc2deg_rdc80G(void);
void driveG(double);
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	//タイマー割り込みの開始
	HAL_TIM_Base_Start_IT(&htim3);

	// Potentio RDc80 initialize
	prdZ.initial_degree = ADC_INIT_DEG_Z;
	prdZ.half_rotation = 1;//ADC_INIT_DEGの値により変える。
	prdG.initial_degree = ADC_INIT_DEG_G;
	prdG.half_rotation = -1;

	//ADCのDMA転送開始
	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adc_vals, ADC_ARR_SIZE);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	//Initialize A4988 pin
	//正の方向 : deg+ : グリッパ閉じる。
	HAL_GPIO_WritePin(A4988_DIR_G_GPIO_Port, A4988_DIR_G_Pin, GPIO_PIN_RESET);
	//正の方向 : deg+ : 下に下がる。
	HAL_GPIO_WritePin(A4988_DIR_Z_GPIO_Port, A4988_DIR_Z_Pin, GPIO_PIN_RESET);

	//Enable Motor PWM
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);

	while (1) {
		//ボタン入力待機
		while(HAL_GPIO_ReadPin(SW_B_GPIO_Port, SW_B_Pin) != GPIO_PIN_RESET){
			HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
			HAL_Delay(200);
		}
		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);

		//Z軸下げる
		HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET);
		driveZ(360.0*TARGET_Z_DISP);
		HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET);

		//ボタン入力待機
		while(HAL_GPIO_ReadPin(SW_B_GPIO_Port, SW_B_Pin) != GPIO_PIN_RESET){
			HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
			HAL_Delay(200);
		}
		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);

		//G軸閉じる
		HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET);
		driveG(TARGET_GRIPPER_DEG*20.0);
		HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET);
		HAL_Delay(2000);

		//ボタン入力待機
		while(HAL_GPIO_ReadPin(SW_B_GPIO_Port, SW_B_Pin) != GPIO_PIN_RESET){
			HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
			HAL_Delay(200);
		}
		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);

		//Z軸戻す
		HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET);
		driveZ(0.0);
		HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET);
		HAL_Delay(2000);

		//ボタン入力待機
		while(HAL_GPIO_ReadPin(SW_B_GPIO_Port, SW_B_Pin) != GPIO_PIN_RESET){
			HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
			HAL_Delay(200);
		}
		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);

		//G軸戻す
		HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET);
		driveG(0.0);
		HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET);
		HAL_Delay(2000);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 7;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);
  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 10000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 24;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, A4988_DIR_Z_Pin|A4988_DIR_G_Pin|LED_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_BUILTIN_Pin */
  GPIO_InitStruct.Pin = LED_BUILTIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_BUILTIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : A4988_DIR_Z_Pin A4988_DIR_G_Pin LED_R_Pin */
  GPIO_InitStruct.Pin = A4988_DIR_Z_Pin|A4988_DIR_G_Pin|LED_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_B_Pin SW_W_Pin */
  GPIO_InitStruct.Pin = SW_B_Pin|SW_W_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LIMIT_SW_Z_Pin LIMIT_SW_G_Pin */
  GPIO_InitStruct.Pin = LIMIT_SW_Z_Pin|LIMIT_SW_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	//Todo:リミットスイッチ用の割り込み
	if (GPIO_Pin == LIMIT_SW_G_Pin) {
	}
	if (GPIO_Pin == LIMIT_SW_Z_Pin) {
	}
}

//下に下げると負の値になる。
void adc2deg_rdc80Z(void){
	if(((prdZ.half_rotation) %2) == 0){
		//a相が計測範囲内
		prdZ.degree = (double) ((prdZ.phase_a * ADC_RDC80_DEG_CONST) - 170.0);
		if(prdZ.phase_a < 683){
			//計測範囲を下回ったので、b相で計測する。
			prdZ.degree = (double) ((prdZ.phase_b * ADC_RDC80_DEG_CONST) - 170.0);
			prdZ.half_rotation--;
		}else if (prdZ.phase_a > 3413){
			//計測範囲を上回ったので、b相で計測する。
			prdZ.degree = (double) ((prdZ.phase_b * ADC_RDC80_DEG_CONST) - 170.0);
			prdZ.half_rotation++;
		}
	}else{
		//b相が計測範囲内
		prdZ.degree = (double) ((prdZ.phase_b * ADC_RDC80_DEG_CONST) - 170.0);
		if(prdZ.phase_b < 683){
			//計測範囲を下回ったので、a相で再計測する。
			prdZ.degree = (double) ((prdZ.phase_a * ADC_RDC80_DEG_CONST) - 170.0);
			prdZ.half_rotation--;
		}else if(prdZ.phase_b > 3413){
			//計測範囲を超えたので、a相で再計測する。
			prdZ.degree = (double) ((prdZ.phase_a * ADC_RDC80_DEG_CONST) - 170.0);
			prdZ.half_rotation++;
		}
	}
	prdZ.degree += (double) ((prdZ.half_rotation) * 180.0);
	prdZ.degree -= prdZ.initial_degree;
}

void adc2deg_rdc80G(void){
	if(((prdG.half_rotation) %2) == 0){
		//a相が計測範囲内
		prdG.degree = (double) ((prdG.phase_a * ADC_RDC80_DEG_CONST) - 170.0);
		if(prdG.phase_a < 683){
			//計測範囲を下回ったので、b相で計測する。
			prdG.degree = (double) ((prdG.phase_b * ADC_RDC80_DEG_CONST) - 170.0);
			prdG.half_rotation--;
		}else if (prdG.phase_a > 3413){
			//計測範囲を上回ったので、b相で計測する。
			prdG.degree = (double) ((prdG.phase_b * ADC_RDC80_DEG_CONST) - 170.0);
			prdG.half_rotation++;
		}
	}else{
		//b相が計測範囲内
		prdG.degree = (double) ((prdG.phase_b * ADC_RDC80_DEG_CONST) - 170.0);
		if(prdG.phase_b < 683){
			//計測範囲を下回ったので、a相で再計測する。
			prdG.degree = (double) ((prdG.phase_a * ADC_RDC80_DEG_CONST) - 170.0);
			prdG.half_rotation--;
		}else if(prdG.phase_b > 3413){
			//計測範囲を超えたので、a相で再計測する。
			prdG.degree = (double) ((prdG.phase_a * ADC_RDC80_DEG_CONST) - 170.0);
			prdG.half_rotation++;
		}
	}
	prdG.degree += (double) ((prdG.half_rotation) * 180.0);
	prdG.degree -= prdG.initial_degree;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	prdZ.phase_a = adc_vals[0];
	prdZ.phase_b = adc_vals[1];
	prdG.phase_a = adc_vals[2];
	prdG.phase_b = adc_vals[3];
}

//Z軸モータを指定の角度に駆動する関数。
//プラスを指定すると、下に動く。
void driveZ(double command_deg){
	uint8_t degOK = 0;
	double tmpDeg = 0.0;
	while (!degOK){
		adc2deg_rdc80Z();
		tmpDeg = command_deg + prdZ.degree;
		//adc2deg_rdc80Zは下方向へ動かすとマイナス。しかし、入力は下をプラスにしたいのでこうした。
		if(tmpDeg>=3.0){
			if(tmpDeg>=120.0)__HAL_TIM_SET_PRESCALER(&htim3,72-1);//Change motor speed
			else __HAL_TIM_SET_PRESCALER(&htim3,1152-1);//Change motor speed
			HAL_GPIO_WritePin(A4988_DIR_Z_GPIO_Port, A4988_DIR_Z_Pin, GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 50);//Motor Z
		}else if (tmpDeg<= -3.0){
			if(tmpDeg<=-120.0)__HAL_TIM_SET_PRESCALER(&htim3,72-1);//Change motor speed
			else __HAL_TIM_SET_PRESCALER(&htim3,1152-1);//Change motor speed
			HAL_GPIO_WritePin(A4988_DIR_Z_GPIO_Port, A4988_DIR_Z_Pin, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 50);//Motor Z
		}else {
			HAL_GPIO_WritePin(A4988_DIR_Z_GPIO_Port, A4988_DIR_Z_Pin, GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);//Motor Z
			degOK = 1;
		}
		HAL_Delay(1);
	}
}

//プラスを指定すると閉じる。
void driveG(double command_deg){
	uint8_t degOK = 0;
	double tmpDeg = 0.0;
	while (!degOK){
		adc2deg_rdc80G();
		tmpDeg = command_deg - prdG.degree;
		if(tmpDeg>=5.0){
			if(tmpDeg>=120.0)__HAL_TIM_SET_PRESCALER(&htim3,72-1);//Change motor speed
			else __HAL_TIM_SET_PRESCALER(&htim3,1152-1);//Change motor speed
			HAL_GPIO_WritePin(A4988_DIR_G_GPIO_Port, A4988_DIR_G_Pin, GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 50);//Motor G
		}else if (tmpDeg<= -5.0){
			if(tmpDeg<=-120.0)__HAL_TIM_SET_PRESCALER(&htim3,72-1);//Change motor speed
			else __HAL_TIM_SET_PRESCALER(&htim3,1152-1);//Change motor speed
			HAL_GPIO_WritePin(A4988_DIR_G_GPIO_Port, A4988_DIR_G_Pin, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 50);//Motor G
		}else {
			HAL_GPIO_WritePin(A4988_DIR_G_GPIO_Port, A4988_DIR_G_Pin, GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);//Motor G
			degOK = 1;
		}
		HAL_Delay(1);
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
