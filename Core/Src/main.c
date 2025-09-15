/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
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
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
const uint8_t indicatorNumbers = 5U;
const uint8_t segmentNumbers = 8U;
const uint16_t pinMask = 0x1FE;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void receiveDataProceed(uint8_t *rxBuf);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */
typedef struct {
	GPIO_TypeDef *GPIOx;
	uint16_t GPIO_Pin;
}GPIOs_TypeDef;

const GPIOs_TypeDef Indicators[] = {
		{ IND1_GPIO_Port, IND1_Pin },
		{ IND2_GPIO_Port, IND2_Pin },
		{ IND3_GPIO_Port, IND3_Pin },
		{ IND4_GPIO_Port, IND4_Pin },
		{ IND5_GPIO_Port, IND5_Pin },
};

const GPIOs_TypeDef Segments[] = {
		{ SEG_A_GPIO_Port, SEG_A_Pin },
		{ SEG_B_GPIO_Port, SEG_B_Pin },
		{ SEG_C_GPIO_Port, SEG_C_Pin },
		{ SEG_D_GPIO_Port, SEG_D_Pin },
		{ SEG_E_GPIO_Port, SEG_E_Pin },
		{ SEG_F_GPIO_Port, SEG_F_Pin },
		{ SEG_G_GPIO_Port, SEG_G_Pin },
		{ SEG_DP_GPIO_Port, SEG_DP_Pin },
};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */
void clearAllIndicators() {
	for (int i = 0; i < indicatorNumbers; ++i) {
		Indicators[i].GPIOx->BSRR = Indicators[i].GPIO_Pin << 16u;
	}
}

void updateSegments() {
	for (size_t i = 0; i < segmentNumbers; ++i) {
		Segments[i].GPIOx->BSRR = Segments[i].GPIO_Pin << 16u;
	}
}

void chooseIndicator(uint8_t number) {

	switch(number) {
	case 1:
		IND1_GPIO_Port->BSRR = (IND1_Pin) | (IND2_Pin | IND3_Pin | IND4_Pin | IND5_Pin) << 16u;
		break;

	case 2:
		IND2_GPIO_Port->BSRR = (IND2_Pin) | (IND1_Pin | IND3_Pin | IND4_Pin | IND5_Pin) << 16u;
		break;

	case 3:
		IND3_GPIO_Port->BSRR = (IND3_Pin) | (IND1_Pin | IND2_Pin | IND4_Pin | IND5_Pin) << 16u;
		break;

	case 4:
		IND4_GPIO_Port->BSRR = (IND4_Pin) | (IND1_Pin | IND2_Pin | IND3_Pin | IND5_Pin) << 16u;
		break;

	case 5:
		IND5_GPIO_Port->BSRR = (IND5_Pin) | (IND1_Pin | IND2_Pin | IND3_Pin | IND4_Pin) << 16u;
		break;

	default:
		clearAllIndicators();
		break;
	}

}

void indicateNumber(uint16_t number, bool isDP) {

	enum {
		ZERO = 0x007E,
		ONE = 0x000C,
		TWO = 0x00B6,
		THREE = 0x009E,
		FOUR = 0x00CC,
		FIVE = 0x00DA,
		SIX = 0x00FA,
		SEVEN = 0x000E,
		EIGHTH = 0x00FE,
		NINE = 0x00DE,
		DP = 0x0100,

		SPACE = 0x0000, ///////////////////////////////////////////////////////////////////////////////////
		N_SYMBOL = 0x00A8,
		E_SYMBOL = 0x00F2,
		//O_SYMBOL = 0x00B8,
		O_SYMBOL = 0x00B8,
		MINUS = 0x0080

	};

	static const uint16_t numbers[] = { ZERO, 		ONE, 		TWO, 		THREE, 		FOUR, 		FIVE, 		SIX,		SEVEN, 		EIGHTH, 		NINE,
										ZERO | DP, 	ONE | DP,   TWO | DP,	THREE | DP, FOUR | DP,  FIVE | DP,	SIX | DP, 	SEVEN | DP, EIGHTH | DP, 	NINE | DP,
										N_SYMBOL,   E_SYMBOL,   O_SYMBOL,   MINUS,      SPACE};
	static const uint16_t numbersWithDP[] = { ZERO | DP, ONE | DP, TWO | DP, THREE | DP, FOUR | DP, FIVE | DP, SIX | DP, SEVEN | DP, EIGHTH | DP, NINE | DP, SPACE };

	if (number > sizeof(numbers)) {
		return;
	}

	//SEG_A_GPIO_Port->BSRR = isDP ? numbersWithDP[number] | (~(numbersWithDP[number]) & pinMask) << 16u :
	//																	numbers[number] | (~(numbers[number]) & pinMask) << 16u;
	SEG_A_GPIO_Port->BSRR = numbers[number] | (~(numbers[number]) & pinMask) << 16u;

}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t buff[5];
uint32_t show = 0;
uint32_t show_prev = 0;
uint8_t prev_screen = 0;
uint8_t screen = 0;
uint8_t flag_vibro = 0;
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
  MX_I2C1_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */

 // HAL_I2C_Slave_Receive(&hi2c1, Rxbuff, 5, 1000);
  clearAllIndicators();
  HAL_TIM_Base_Start_IT(&htim14);
  HAL_I2C_EnableListen_IT(&hi2c1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_Delay(10);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (flag_vibro == 1)  {
		  HAL_GPIO_WritePin(MOTOR_CONTROL_GPIO_Port, MOTOR_CONTROL_Pin, GPIO_PIN_SET);
		  HAL_Delay(200);
		  HAL_GPIO_WritePin(MOTOR_CONTROL_GPIO_Port, MOTOR_CONTROL_Pin, GPIO_PIN_RESET);
		  flag_vibro = 0;
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hi2c1.Init.Timing = 0x0010061A;
  hi2c1.Init.OwnAddress1 = 122;
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
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
	htim14.Instance = TIM14;
	  htim14.Init.Prescaler = 16 - 1;
	  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim14.Init.Period = 100 - 1;
	  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
	  HAL_GPIO_WritePin(GPIOA, SEG_A_Pin|SEG_B_Pin|SEG_C_Pin|SEG_D_Pin
	                          |SEG_E_Pin|SEG_F_Pin|SEG_G_Pin|SEG_DP_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOB, IND1_Pin|IND2_Pin|IND3_Pin|IND4_Pin
	                          |IND5_Pin|MOTOR_CONTROL_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pins : SEG_A_Pin SEG_B_Pin SEG_C_Pin SEG_D_Pin
	                           SEG_E_Pin SEG_F_Pin SEG_G_Pin SEG_DP_Pin */
	  GPIO_InitStruct.Pin = SEG_A_Pin|SEG_B_Pin|SEG_C_Pin|SEG_D_Pin
	                          |SEG_E_Pin|SEG_F_Pin|SEG_G_Pin|SEG_DP_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pins : IND1_Pin IND2_Pin IND3_Pin IND4_Pin
	                           IND5_Pin MOTOR_CONTROL_Pin */
	  GPIO_InitStruct.Pin = IND1_Pin|IND2_Pin|IND3_Pin|IND4_Pin
	                          |IND5_Pin|MOTOR_CONTROL_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */

  /* USER CODE BEGIN Callback 1 */
	static uint8_t counter = 0;
	uint32_t num = show;
  if (htim->Instance == TIM14) {
  	indicateNumber(24, false);
  	switch (counter) {
  	case 0:
			chooseIndicator(1);
			//indicateNumber(num / 10000, false);
			indicateNumber(buff[0], false);
		break;

  	case 1:
			chooseIndicator(2);
			//indicateNumber(num / 1000 % 10, false);
			indicateNumber(buff[1], false);
		break;

  	case 2:
			chooseIndicator(3);
			//indicateNumber(num / 100 % 10, false);
			indicateNumber(buff[2], false);
		break;

  	case 3:
			chooseIndicator(4);
			//indicateNumber(num / 10 % 10, false);
			indicateNumber(buff[3], false);
		break;

  	case 4:
			chooseIndicator(5);
			//indicateNumber(num % 10, false);
			indicateNumber(buff[4], false);
		break;

  	}
  	counter++;
  	counter = counter == 5 ? 0 : counter;

  	/*	if (!(counter % 16)) {
				chooseIndicator(1);
				indicateNumber(num / 10000);
  		}
  		if (!(counter % 32)) {
				chooseIndicator(2);
				indicateNumber(num / 1000% 10);
  		}
  		if (!(counter % 64)) {
				chooseIndicator(3);
				indicateNumber(num / 100% 10);
  		}

			if (!(counter % 128)) {
				chooseIndicator(4);
				indicateNumber(num / 10% 10);
			}
			if (!(counter % 256)) {
				chooseIndicator(5);
				indicateNumber(num %10);
			}
    	counter++;*/
  }
  /* USER CODE END Callback 1 */
}

/* USER CODE BEGIN 4 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *I2cHandle){
	//HAL_I2C_Mem_Read_IT(&hi2c1,122, 0, 1, buff, 5);

	 HAL_I2C_Slave_Receive_IT(&hi2c1, buff, 5);
	//HAL_I2C_Mem_Read_IT(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size)
}
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *I2cHandle)

{

	//buff[pReceive]=buffer;
	//HAL_I2C_Slave_Receive_IT(&hi2c1, buff, 5);

	 /*pReceive++;
	 if(pReceive==5)
		 pReceive=0;*/
	//HAL_I2C_Mem_Read_IT(&hi2c1,122, 0, 1, buff, 5);
	show  = buff[0] * 10000 + buff[1] * 1000 + buff[2] * 100 + buff[3] * 10 + buff[4];
	screen = buff[0];
	if (screen != prev_screen){
		flag_vibro = 1;
	}
	HAL_I2C_EnableListen_IT(&hi2c1);
	prev_screen = screen;
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
	//HAL_I2C_Slave_Receive_IT(&hi2c1, buff, 5);
	HAL_I2C_Slave_Seq_Receive_IT(&hi2c1, buff, sizeof(buff), I2C_FIRST_AND_LAST_FRAME);
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
