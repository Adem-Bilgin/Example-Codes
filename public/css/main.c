/* USER CODE BEGIN Header */
/*******************************************************************************
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

#include "i2c-lcd.h"
#include "stdio.h"

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

TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

void delay(uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim6, 0);
	while(__HAL_TIM_GET_COUNTER(&htim6) < time);
}
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){ // A2 pinini yeri geldiğinde bi giriş bir çıkış yapıyoruz.

	GPIO_InitTypeDef DHT11_DATA={0};				// Sensörle iletişim kuracak port pin struct ırı.

	DHT11_DATA.Pin=GPIO_Pin;
	DHT11_DATA.Mode=GPIO_MODE_OUTPUT_PP;
	DHT11_DATA.Pull=GPIO_NOPULL;
	DHT11_DATA.Speed=GPIO_SPEED_FREQ_LOW;

	HAL_GPIO_Init(GPIOx, &DHT11_DATA);

}
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){

	GPIO_InitTypeDef DHT11_DATA={0};

	DHT11_DATA.Pin=GPIO_Pin;
	DHT11_DATA.Mode=GPIO_MODE_INPUT;
	DHT11_DATA.Pull=GPIO_NOPULL;
	DHT11_DATA.Speed=GPIO_SPEED_FREQ_LOW;

	HAL_GPIO_Init(GPIOx, &DHT11_DATA);
}

#define DHT11_PORT GPIOB
#define DHT11_PIN GPIO_PIN_9

uint8_t durum=0;
uint16_t tempVal=0,humVal=0;
uint8_t dhtVal[2];
uint8_t mData[40];
uint16_t mTime1 = 0, mTime2 = 0;
uint16_t mbit = 0;
uint8_t  parityVal = 0, genParity = 0;
uint8_t isi_maks = 0, isi_min = 0, nem_maks = 0, nem_min = 0;
char nem_yazi[16] = " ";

double Humidity=0,Temperature=0;

uint8_t DHT11_Read (void){

  for(int a=0;a<40;a++) mData[a]=0;
   mTime1 = 0, mTime2 = 0, durum=0, tempVal=0, humVal=0, parityVal = 0, genParity = 0,  mbit = 0;

     Set_Pin_Output(DHT11_PORT,DHT11_PIN);
	 HAL_GPIO_WritePin(DHT11_PORT,DHT11_PIN,GPIO_PIN_RESET);
    delay(18000);
   	Set_Pin_Input(DHT11_PORT,DHT11_PIN);

	 __HAL_TIM_SET_COUNTER(&htim6, 0);				//set timer counter to zero
	while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET) if((uint16_t)__HAL_TIM_GET_COUNTER(&htim6) > 500) return 0;

	__HAL_TIM_SET_COUNTER(&htim6, 0);
	while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_RESET) if((uint16_t)__HAL_TIM_GET_COUNTER(&htim6) > 500) return 0;
	mTime1 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim6);

	__HAL_TIM_SET_COUNTER(&htim6, 0);
	while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET) if((uint16_t)__HAL_TIM_GET_COUNTER(&htim6) > 500) return 0;
    mTime2 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim6);

	if(mTime1 < 75 && mTime1 > 85 && mTime2 < 75 && mTime2 > 85)
	{
		return 0;
	}

	for(int j = 0; j < 40; j++)
	{
		__HAL_TIM_SET_COUNTER(&htim6, 0);
		while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_RESET) if((uint16_t)__HAL_TIM_GET_COUNTER(&htim6) > 500) return 0;
		__HAL_TIM_SET_COUNTER(&htim6, 0);
		while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET) if((uint16_t)__HAL_TIM_GET_COUNTER(&htim6) > 500) return 0;
		mTime1 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim6);

		//check pass time in high state
		//if pass time 25uS set as LOW
		if(mTime1 > 20 && mTime1 < 30)
		{
			mbit = 0;
		}
		else if(mTime1 > 60 && mTime1 < 80) //if pass time 70 uS set as HIGH
		{
			 mbit = 1;
		}

		//set i th data in data buffer
		mData[j] = mbit;
	}

	//get hum value from data buffer
	for(int i = 0; i < 8; i++)
	{
		humVal += mData[i];
		humVal = humVal << 1;
	}

	//get temp value from data buffer
	for(int i = 16; i < 24; i++) // Burda neden 8 ile 16 arası yok, çünkü bu aralıktaki gelen değerler 0000 değerindedir. Bu yüzden almamıza gerek yok.
	{
		tempVal += mData[i];
		tempVal = tempVal << 1;
	}

	//get parity value from data buffer
	for(int i = 32; i < 40; i++)
	{
		parityVal += mData[i];
		parityVal = parityVal << 1;
	}

	parityVal = parityVal >> 1;
	humVal = humVal >> 1;
	tempVal = tempVal >> 1;

	genParity = humVal + tempVal;

	dhtVal[0]= tempVal;
	dhtVal[1] = humVal;

	return 1;
}

void Deger_Araligi()
{
	  lcd_send_cmd(0x80);
	  lcd_send_string("NEM Min. = %0   ");
	  lcd_send_cmd(0xC0);
	  lcd_send_string("                ");

	while(1)
	{
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == GPIO_PIN_SET)	// 1 numaralı tuş
		{
			break;
		}

		while(1) 												// Nem min. değerini ayarlıyoruz
		{
			if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == GPIO_PIN_SET)
			{
				break;
			}

			if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_6) == GPIO_PIN_SET) // 4 numaralı tuş
			{
				HAL_Delay(100);
				nem_min++;
				if(nem_min>100)
					nem_min=100;
				sprintf(nem_yazi, "%d", nem_min);

				lcd_send_cmd(0x80);
				lcd_send_string("NEM Min. = %    ");
				lcd_send_cmd(0xC0);
				lcd_send_string("                ");
				lcd_send_cmd(0x8C);
				lcd_send_string(nem_yazi);
			}
			if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5) == GPIO_PIN_SET)	// 3 numaralı tuş
			{
				HAL_Delay(100);
				nem_min--;
				if(nem_min>100)
					nem_min=0;
				sprintf(nem_yazi, "%d", nem_min);

				lcd_send_cmd(0x80);
				lcd_send_string("NEM Min. = %    ");
				lcd_send_cmd(0xC0);
				lcd_send_string("                ");
				lcd_send_cmd(0x8C);
				lcd_send_string(nem_yazi);
			}
		}

		HAL_Delay(400);
		lcd_send_cmd(0x80);
		lcd_send_string("NEM Max. = %0   ");
		while(1)											// Nem max. değerini ayarlıyoruz
		{

			if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == GPIO_PIN_SET)
			{
				break;
			}
			if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_6) == GPIO_PIN_SET)
			{
				HAL_Delay(100);
				nem_maks++;
				if(nem_maks>100)
					nem_maks=100;
				sprintf(nem_yazi, "%d", nem_maks);

				lcd_send_cmd(0x80);
				lcd_send_string("NEM Max. = %    ");
				lcd_send_cmd(0xC0);
				lcd_send_string("                ");
				lcd_send_cmd(0x8C);
				lcd_send_string(nem_yazi);
			}
			if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5) == GPIO_PIN_SET)
			{
				HAL_Delay(100);
				nem_maks--;
				if(nem_maks>100)
					nem_maks=0;
				sprintf(nem_yazi, "%d", nem_maks);

				lcd_send_cmd(0x80);
				lcd_send_string("NEM Max. = %    ");
				lcd_send_cmd(0xC0);
				lcd_send_string("                ");
				lcd_send_cmd(0x8C);
				lcd_send_string(nem_yazi);
			}
		}

		HAL_Delay(400);
		lcd_send_cmd(0x80);
		lcd_send_string("ISI Min. = 0    ");
		while(1)										// Isı min. değerini ayarlıyoruz
		{

			if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == GPIO_PIN_SET)
			{
				break;
			}
			if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_6) == GPIO_PIN_SET)
			{
				HAL_Delay(100);
				isi_min++;
				if(isi_min>100)
					isi_min=100;
				sprintf(nem_yazi, "%d", isi_min);

				lcd_send_cmd(0x80);
				lcd_send_string("ISI Min. =      ");
				lcd_send_cmd(0xC0);
				lcd_send_string("                ");
				lcd_send_cmd(0x8B);
				lcd_send_string(nem_yazi);
			}
			if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5) == GPIO_PIN_SET)
			{
				HAL_Delay(100);
				isi_min--;
				if(isi_min>100)
					isi_min=0;
				sprintf(nem_yazi, "%d", isi_min);

				lcd_send_cmd(0x80);
				lcd_send_string("ISI Min. =      ");
				lcd_send_cmd(0xC0);
				lcd_send_string("                ");
				lcd_send_cmd(0x8B);
				lcd_send_string(nem_yazi);
			}
		}

		HAL_Delay(400);
		lcd_send_cmd(0x80);
		lcd_send_string("ISI Max. = 0    ");
		while(1)											// Isı max. değerini ayarlıyoruz
		{
			if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == GPIO_PIN_SET)
			{
				break;
			}
			if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_6) == GPIO_PIN_SET)
			{
				HAL_Delay(100);
				isi_maks++;
				if(isi_maks>50)
					isi_maks=50;
				sprintf(nem_yazi, "%d", isi_maks);

				lcd_send_cmd(0x80);
				lcd_send_string("ISI Max. =      ");
				lcd_send_cmd(0xC0);
				lcd_send_string("                ");
				lcd_send_cmd(0x8B);
				lcd_send_string(nem_yazi);
			}
			if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5) == GPIO_PIN_SET)
			{
				HAL_Delay(100);
				isi_maks--;
				if(isi_maks>50)
					isi_maks=0;
				sprintf(nem_yazi, "%d", isi_maks);

				lcd_send_cmd(0x80);
				lcd_send_string("ISI Max. =      ");
				lcd_send_cmd(0xC0);
				lcd_send_string("                ");
				lcd_send_cmd(0x8B);
				lcd_send_string(nem_yazi);
			}
		}
		break;
	}
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
  MX_TIM6_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim6);

  lcd_init();

  char yazdir[16] = " ";
  char cumle[16] = " ";

  lcd_send_string("BitirmeProjesi B");
  lcd_send_cmd(0xC0);
  HAL_Delay(1000);
  lcd_send_string("Adem Bilgin  ;) ");
  HAL_Delay(2500);

  lcd_send_cmd(0x80);
  lcd_send_string("   ISI VE NEM   ");
  lcd_send_cmd(0xC0);
  lcd_send_string("     CIHAZI     ");
  HAL_Delay(3000);

  Deger_Araligi();

  lcd_send_cmd(0x80);
  lcd_send_string("NEM = %         "); // 6 karakterlik yazının devam adresi 0x86
  lcd_send_cmd(0xC0);
  lcd_send_string("SICAKLIK =      "); // 11 karakterlik yazının devam adresi 0xCB




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  durum = DHT11_Read();
	  if(durum == 1)
	  {
		  Temperature=tempVal; Humidity = humVal;
	  }

	  sprintf(yazdir, "%.1f", Humidity); // yazdir dizisine Vadc değişkenini string olarak atıyor.
	  sprintf(cumle, "%.1fC", Temperature);

	  if((Temperature >= isi_maks ||Temperature <= isi_min || Humidity <= nem_min ||  Humidity >= nem_maks) && Humidity>0 && Temperature>0)
	  {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	  }

	  else if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4) == GPIO_PIN_SET)
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

	  HAL_Delay(500);
	  lcd_send_cmd(0x87);
	  lcd_send_string(yazdir);
	  lcd_send_cmd(0xCB);
	  lcd_send_string(cumle);
	  HAL_Delay(500);
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
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  htim6.Init.Prescaler = 84-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0xffff-1;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD3 PD4 PD5 PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
