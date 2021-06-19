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
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define SLAVE_ADD 0x01
#define PUMP_OFF 		1	//PUMP IN OFF STATE
#define PUMP_ON_SP_CN 	2	//PUMP ON AND IN SPEED CONTROL STATAE
#define PUMP_ON_PC_CN	3	//PUMP ON AND IN PROCESS CONTROL STATE
#define PUMP_ERROR_MODE 4	//PUMP IN ERROR MODE,GOTO PUMP_OFF STATE AND THEN TO ANY OTHER STATE
#define PUMP_ON_SS_CN	7	//PUMP ON AND IN SPEED SAFETY CONTROL STATE
#define PUMP_ON_PS_CN	8	//PUMP ON AND IN PROCESS CONTROL STATE

#define SUCCESS 		1	//SUCCESSFULLY WRITTEN AND ACKNOWLEDGED DATA
#define SOFT_ERROR		0	//ERROR IN WRITING DATA
#define HARD_ERROR		2	//CHECK ERROR MAP FOR ERROR CODE AND COMPARE IN DOCUMENTATION

uint8_t rec_data[30];
uint8_t hold_reg_map[56]={0};
uint8_t inp_reg_map[66]={0};
int8_t write_reg_map[249]={-1};
uint8_t error_map[2]={0};
int status = 0;

int READ_HOLDING_REG(uint8_t Add_HI,uint8_t Add_LO,uint8_t Num_reg_HI,uint8_t Num_reg_LO)
{
	uint8_t buff[6] = {SLAVE_ADD,0x03,Add_HI,Add_LO,Num_reg_HI,Num_reg_LO};
	uint16_t N={Num_reg_HI||Num_reg_LO};


	HAL_Delay(24);//Silence on channel required for 24ms for baud rate 57600
	//HAL_UART_Transmit(&huart1,buff, sizeof(buff),500);
	HAL_UART_Transmit(&huart3,buff, sizeof(buff),5000);
	HAL_Delay(24);//Silence on channel required for 24ms for baud rate 57600


	HAL_UART_Receive (&huart1, rec_data, 2*sizeof(N) , 5000);
	if(rec_data[1] == 0x83)//error code returned check error map array for exact error
	{
		error_map[0] = rec_data[2];
		return 2;
	}
	if(rec_data[1]==0x03)	// successfully transmitted command and successfully received data
	{
		uint16_t add_full = (Add_HI&0xFF00)||(Add_HI&0x00FF);
		add_full = add_full - 0x3FFF ;
		for(int i=0;i<N;i++)
		{
			hold_reg_map[add_full] = rec_data[i+2];

		}
		return(1);
	}
	else
		{
			//HAL_UART_Transmit(&huart3,"error",6,500);
			return(0);
		}

}

int READ_INPUT_REG(uint8_t Add_HI,uint8_t Add_LO,uint8_t Num_reg)
{
		uint8_t buff[6] = {SLAVE_ADD,0x04,Add_HI,Add_LO,Num_reg};

		HAL_Delay(24);//Silence on channel required for 24ms for baud rate 57600
		HAL_UART_Transmit(&huart3,buff, sizeof(buff),500);
		HAL_Delay(24);//Silence on channel required for 24ms for baud rate 57600

		HAL_UART_Receive (&huart1, rec_data, Num_reg , 5000);
		if(rec_data[1] == 0x84)
			{
				error_map[1] = rec_data[2];
				return 2;
			}
		if(rec_data[1]==0x04)
		{
			uint16_t add_full = (Add_HI&0xFF00)||(Add_HI&0x00FF);
			add_full = add_full - 0x3FE0 ;

			for(int i=1;i<Num_reg;i++)
			{
				inp_reg_map[add_full] = rec_data[i+2];
			}
			return(1);

		}
		else
			{
				return(0);
			}
}

int WRITE_SINGLE_REG(uint8_t Add_HI,uint8_t Add_LO,uint16_t value)
{
	uint8_t buff[] = {SLAVE_ADD,0x06,Add_HI,Add_LO,(value&0xFF00),(value&0x00FF)};

	HAL_Delay(24);//Silence on channel required for 24ms for baud rate 57600
	HAL_UART_Transmit(&huart3,buff,sizeof(buff),500);
	HAL_Delay(24);//Silence on channel required for 24ms for baud rate 57600

	HAL_UART_Receive(&huart1,rec_data,sizeof(buff)+2,5000);
	if(rec_data[1] == 0x86)
		{
			error_map[2] = rec_data[2];
			return 2;
		}
	if(rec_data[1]==0x06)
	{
		if(rec_data[2]==buff[2]&&rec_data[3]==buff[3]&&rec_data[4]==buff[4]&&rec_data[5]==buff[5])
		{
			return 1;
		}
		else return 2;
	}
	else
		return 2;
}

int WRITE_MULTI_REG(uint8_t Add_HI, uint8_t Add_LO, uint8_t Num_of_reg_HI, uint8_t Num_of_reg_LO)
{
	uint8_t N = 2* (((Num_of_reg_HI&0xFF00)||(Num_of_reg_LO&0x00FF))&(0x00FF));
	uint8_t multi_send[N];
	for (int i=0;(write_reg_map[i]!=-1 || i<249);i++)
	{
		multi_send[i]=write_reg_map[i];
	}
	uint8_t buff[] = {SLAVE_ADD,0x10,Add_HI,Add_LO,Num_of_reg_HI,Num_of_reg_LO,N,multi_send};

		HAL_Delay(24);//Silence on channel required for 24ms for baud rate 57600
		HAL_UART_Transmit(&huart3,buff,sizeof(buff),500);
		HAL_Delay(24);//Silence on channel required for 24ms for baud rate 57600

		HAL_UART_Receive(&huart1,rec_data,6,5000);
		if(rec_data[1] == 0x86)
			{
				error_map[3] = rec_data[2];
				for(int k =0;k<249;k++)
				write_reg_map[k]=-1;
				return 2;
			}
		if(rec_data[1]==0x10)
		{
			if(rec_data[2]==buff[2]&&rec_data[3]==buff[3]&&rec_data[4]==buff[4]&&rec_data[5]==buff[5])
			{
				for(int k =0;k<249;k++)
				write_reg_map[k]=-1;
				return 1;
			}
			else
				{
				for(int k =0;k<249;k++)
				write_reg_map[k]=-1;
					return 2;
				}
		}
		else
		{
			for(int k =0;k<249;k++)
			write_reg_map[k]=-1;
			return 2;
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
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //char msg[]= "TESTING";
	 // HAL_UART_Transmit(&huart3, "TEST", 4, HAL_MAX_DELAY);
	 // HAL_UART_Transmit(&huart1, "TEST", 4, HAL_MAX_DELAY);
	  int status = READ_HOLDING_REG(0x40,0x00,0x00,0x01);
	  HAL_Delay(30000);
	  status = READ_INPUT_REG(0x46,0x00,0x08);
	  HAL_Delay(30000);
	  status = WRITE_SINGLE_REG(0x40,0x32,0x4432);
	  HAL_Delay(30000);
	  write_reg_map[1] = 20;
	  write_reg_map[2] = 30;
	  write_reg_map[3] = 40;
	  write_reg_map[4] = 50;
	  write_reg_map[5] = 60;
	  write_reg_map[6] = 70;
	  write_reg_map[7] = 80;
	  write_reg_map[8] = 90;
	  status = WRITE_MULTI_REG(0x46,0x00,0x00,0x08);
	  HAL_Delay(30000);
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART3;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
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
  huart3.Init.BaudRate = 38400;
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

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
