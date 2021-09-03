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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "radio_mod.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVO_0 400
#define SERVO_90 1500
#define SERVO_180 2600

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void USB_Reset(void);
long map(long x, long in_min, long in_max, long out_min, long out_max);
void receive(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char str[129];
volatile uint8_t flag = 1; // для DMA
volatile uint16_t adc[2] = {0,}; // у нас два канала поэтому массив из двух элементов
const float kd = 4.33; //коэффициент  делителя напряжения для (14.3 вольт максимально)
int subtrim = 1600, sens1 = 300, sens2 = 300;
float vbat1, vbat2;
bool wing = false;
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
	DWT_Init(); // счётчик для микросекундных пауз
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	
	// Запуск ШИМ
	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_4);
	
	// Yачальное положение
	if (wing){
	TIM4->CCR1 = SERVO_90 - 1100;
	TIM4->CCR2 = SERVO_90 + 1100;
	}else{
		subtrim = 700;
		TIM4->CCR1 = SERVO_90 + 300;
		TIM4->CCR2 = SERVO_90;
		TIM4->CCR4 = SERVO_90;
	}
	// Запуск АЦП1
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc, 2);

  const uint64_t pipe1 = 0xE8E8F0F0E2LL; // адрес нашего канала

	uint8_t connected = isChipConnected(); // проверка на подключените модуля к SPI
	
	uint8_t res = NRF_Init(); // инициализация
	
/////////////////////////////// УСТАНОВКИ ////////////////////////////////
  enableAckPayload();
  setChannel(11);
  openReadingPipe(1, pipe1);
  startListening();
/////////////////////////////////////


///////////////////////////// DEBUG TERMINAL /////////////////////////////
	
	
//HAL_Delay(2000);
//char str[128] = {0,};
//snprintf(str, 64, "Connected: %s\r\n", connected ? "OK" : "NOT OK");
//CDC_Transmit_FS((unsigned char*)str, strlen(str));
//HAL_Delay(500);

//snprintf(str, 64, "Init: %s\r\n", res > 0 && res < 255 ? "OK" : "NOT OK");
//CDC_Transmit_FS((unsigned char*)str, strlen(str));

//if (res > 0 && res < 255) 
//{
//	uint8_t status = get_status();
//	snprintf(str, 64, "Status: 0x%02x \r\n", status);
//	CDC_Transmit_FS((unsigned char*)str, strlen(str));
//	HAL_Delay(500);

//	status = getPALevel();
//	snprintf(str, 64, "PA_Level: 0x%02x \n", status);
//	CDC_Transmit_FS((unsigned char*)str, strlen(str));
//	HAL_Delay(500);

//	if(status == 0x00)
//	{
//		CDC_Transmit_FS((unsigned char*)"RF24_PA_MIN\r\n", strlen("RF24_PA_MIN\r\n"));
//	}
//	else if(status == 0x01)
//	{
//		CDC_Transmit_FS((unsigned char*)"RF24_PA_LOW\r\n", strlen("RF24_PA_LOW\r\n"));
//	}
//	else if(status == 0x02)
//	{
//		CDC_Transmit_FS((unsigned char*)"RF24_PA_HIGH\r\n", strlen("RF24_PA_HIGH\r\n"));
//	}
//	else if(status == 0x03)
//	{
//		CDC_Transmit_FS((unsigned char*)"RF24_PA_MAX\r\n", strlen("RF24_PA_MAX\r\n"));
//	}
//	HAL_Delay(500);
//	
//	status = getChannel();
//	snprintf(str, 64, "Channel: 0x%02x № %d \r\n", status, status);
//	CDC_Transmit_FS((unsigned char*)str, strlen(str));
//	HAL_Delay(500);

//	status = getDataRate();
//	snprintf(str, 64, "Data_Rate: 0x%02x \r\n", status);
//	CDC_Transmit_FS((unsigned char*)str, strlen(str));
//	HAL_Delay(500);

//	if(status == 0x02)
//	{
//		CDC_Transmit_FS((unsigned char*)"RF24_250KBPS\r\n", strlen("RF24_250KBPS\r\n"));
//	}
//	else if(status == 0x01)
//	{
//		CDC_Transmit_FS((unsigned char*)"RF24_2MBPS\r\n", strlen("RF24_2MBPS\r\n"));
//	}
//	else
//	{
//		CDC_Transmit_FS((unsigned char*)"RF24_1MBPS\r\n", strlen("RF24_1MBPS\r\n"));
//	}
//	HAL_Delay(500);

//	status = getPayloadSize();
//	snprintf(str, 64, "Payload_Size: %d \r\n", status);
//	CDC_Transmit_FS((unsigned char*)str, strlen(str));
//	HAL_Delay(500);

//	status = getCRCLength();
//	snprintf(str, 64, "CRC_Length: 0x%02x \r\n", status);
//	CDC_Transmit_FS((unsigned char*)str, strlen(str));
//	HAL_Delay(500);

//	if(status == 0x00)
//	{
//		CDC_Transmit_FS((unsigned char*)"RF24_CRC_DISABLED\r\n", strlen("RF24_CRC_DISABLED\r\n"));
//	}
//	else if(status == 0x01)
//	{
//		CDC_Transmit_FS((unsigned char*)"RF24_CRC_8\r\n", strlen("RF24_CRC_8\r\n"));
//	}
//	else if(status == 0x02)
//	{
//		CDC_Transmit_FS((unsigned char*)"RF24_CRC_16\r\n", strlen("RF24_CRC_16\r\n"));
//	}
//	HAL_Delay(500);
//}
//else
//{
//	HAL_Delay(500);
//	snprintf(str, 64, "Module is not connected or defective");
//	CDC_Transmit_FS((unsigned char*)str, strlen(str));
//	HAL_Delay(500);
//}


/////////////////////////////////////THE END DEBUG TERMINAL///////////////////////////////////////////////////


  maskIRQ(true, true, true); // маскируем прерывания
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		receive();
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

  /** Initializes the CPU, AHB and APB busses clocks
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
  /** Initializes the CPU, AHB and APB busses clocks
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
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
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
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 20000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CE_Pin|CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : IRQ_Pin */
  GPIO_InitStruct.Pin = IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CE_Pin CSN_Pin */
  GPIO_InitStruct.Pin = CE_Pin|CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//для сброса USB
void USB_Reset(void)
{
	 GPIO_InitTypeDef GPIO_InitStruct = {0};

	 // инициализируем пин DP как выход
	 GPIO_InitStruct.Pin = GPIO_PIN_12;
	 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	 HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET); // прижимаем DP к "земле"
	 for(uint16_t i = 0; i < 2000; i++) {}; // немного ждём

	 // переинициализируем пин для работы с USB
	 GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	 GPIO_InitStruct.Pull = GPIO_NOPULL;
	 HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	 for(uint16_t i = 0; i < 2000; i++) {}; // немного ждём
}
// изменение одного диапазона в другой
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//вызов по завершению конвертации
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1)
	{
			flag = 1;
	}
}

//обработка приема
void receive(void)
{
	///////////////////////////////////// ПРИЁМ /////////////////////////////////////////////
	uint8_t nrf_data[32] = {0,}; // Размер буфера (32-MAX)
	static uint8_t remsg = 0;
	uint8_t pipe_num = 0;
	
	if(flag)
	{
		flag = 0;

		//расчёт заряда батареи1 (0 канал АЦП1)
		vbat1 = (3.3/4095)*adc[0]*kd * 10; // 3,3 В на делителе при 14,3 В, дальше делитель не имее смысла
		//расчёт заряда батареи2 (1 канал АЦП1) умножаю на 10, чтобы передать целое число
		vbat2 = (3.3/4095)*adc[1]*kd * 10; // 3,3 В на делителе при 14,3 В, дальше делитель не имее смысла
					
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc, 2);

		adc[0] = 0;
		adc[1] = 0;
	}
	
	if(available(&pipe_num)) // проверяем пришло ли что-то
	{
		remsg = vbat2;
		
		writeAckPayload(pipe_num, &remsg, sizeof(remsg)); // отправляем полезную нагрузку вместе с подтверждением
		
		if(pipe_num == 0) // проверяем куда пришли данные
		{
		//	CDC_Transmit_FS((unsigned char*)"pipe 0 \r\n", strlen("pipe 0 \r\n"));
		}

		else if(pipe_num == 1)
		{
			//CDC_Transmit_FS((unsigned char*)"Pipe 1 \r\n", strlen("Pipe 1 \r\n"));

			uint8_t count = getDynamicPayloadSize(); // смотрим сколько байт прилетело

			read(&nrf_data, count); // Читаем данные в массив nrf_data и указываем сколько байт читать

			if(nrf_data[0] == 77 && nrf_data[1] == 86 && nrf_data[2] == 97) // проверяем правильность данных
			{

				// изменяя это значение, смещаем нейтральное положение
				if (nrf_data[6] == 100)
				{
					subtrim = nrf_data[7]*10; // умножаем на 10 т.к. получаем число от 1 до 255
					sens1 = nrf_data[8]*10; // умножаем на 10 т.к. получаем число от 1 до 255
					sens2 = nrf_data[9]*10; // умножаем на 10 т.к. получаем число от 1 до 255
				}
				
				uint16_t motor = map(nrf_data[5], 1, 255, 800, 2200);
				// Если летательное крыло
				if (wing){
				uint16_t	min = 400 + subtrim + sens1; // минимальное положение с учётом добавок
				uint16_t	max = 2600 - sens1; //вычитаем расходы 
				
				uint16_t pitch = map(nrf_data[3], 1, 255, min, max); // 400 - 2600 крайние положения
				uint16_t roll = map(nrf_data[4], 1, 255, min, max);
						
			// управление сервами
			// проверка на среднее значение
			if (((400+subtrim)+((2600-(400+subtrim))/2)-100 < roll) && (roll < ((400+subtrim)+((2600-(400+subtrim))/2)+100)))
			{ // если поворотный стик на месте, то управляем стиком высоты
			TIM4->CCR1 = 3000 - pitch; // управление сервы с противоположной стороны
			TIM4->CCR2 = pitch;
			} else 
				{ // иначе управляем стиком поворота
				TIM4->CCR1 = roll - 1600; // инвертирование сервы
				TIM4->CCR2 = roll;
				}
				}else{
				uint16_t pitch = map(nrf_data[3], 1, 255, SERVO_0 + sens1 + subtrim, SERVO_180 - sens1);
				uint16_t roll = map(nrf_data[4], 1, 255, SERVO_0 + sens2 + 80, SERVO_180 - sens2);//80 чтобы ровно выставить качалку
				uint16_t raw = map(nrf_data[7], 1, 255, SERVO_0 + 300, SERVO_180 - 300);
					
				// В отличие от крыла тут все проще, стик по X  - отвечает за крен, Y - за тангаж
				TIM4->CCR1 = pitch;	// тангаж
				TIM4->CCR2 = roll; 	// крен
				TIM4->CCR4 = raw; 	// рысканье
					
				snprintf(str, 128, "pitch=%d roll=%d raw=%d m=%d sb=%d sn=%d \r\n", pitch, roll, raw, motor, subtrim, sens1);
				CDC_Transmit_FS((unsigned char*)str, strlen(str));
			}
				TIM4->CCR3 = motor; // управляем мотором

//			snprintf(str, 128, "d3=%d d4=%d d5=% pitch=%d roll=%d raw=%d m=%d sb=%d sn=%d \r\n", nrf_data[3], nrf_data[4], nrf_data[5],
//			pitch, roll, raw, motor, subtrim, sens1);
//			CDC_Transmit_FS((unsigned char*)str, strlen(str));
			}

		}

	}else {
		uint32_t timme = HAL_GetTick();
		while (!available(&pipe_num))
		{ // если потеряли связь более 3с
			if(((HAL_GetTick() - timme) > 1000) && ((HAL_GetTick() - timme) < 4000)) // Проверка на связь
			{				
			TIM4->CCR3 = 800; // обрубаем мотор
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			//CDC_Transmit_FS((unsigned char*)"No connection!\r\n", strlen("No connection!\r\n"));
			}else
				{
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
				}
		}
	} 
}

// установка субтриммера

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
