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
#include "nokia5110_LCD.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
char str_tx[121] = {0,};
volatile uint16_t adc[5] = {0,}; // у нас пять каналов поэтому массив из пяти элементов
volatile uint8_t flag = 0; // для DMA
uint8_t adc1, adc2, adc3, adc4, adc5;
const float kd = 3.52;
char tx[21], x = 0, m = 1, y = 0;
bool a = true, button = false;
int i = 0;
unsigned char subtrim = 70, sens1 = 30, sens2 = 30;

unsigned char but_right_prev;// переменная для хранения предыдущего состояния стика R
unsigned char but_left_prev;// переменная для хранения предыдущего состояния стика L
unsigned char but_r_prev;// переменная для хранения предыдущего состояния кнопки R
unsigned char but_l_prev;// переменная для хранения предыдущего состояния кнопки L

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
bool twobuttons(void);
void button_r(void);
void button_l(void);
void led_lcd(void);
// кнопка правого стика 
bool right_button(void);
// кнопка левого стика 
bool left_button(void);

////////////////MENU////////////////////
void main_menu(unsigned char x);
void flight(void);
void settings(void);
void menu(unsigned char x);
void sensors(void);
void test(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
        flag = 1;
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
	// Настраиваем порты LCD
	LCD_setRST(LCD_RST_GPIO_Port, LCD_RST_Pin);
	LCD_setCE(LCD_CE_GPIO_Port, 	 LCD_CE_Pin);
	LCD_setDC(LCD_DC_GPIO_Port, 	 LCD_DC_Pin);
	LCD_setDIN(LCD_DIN_GPIO_Port, LCD_DIN_Pin);
	LCD_setCLK(LCD_CLK_GPIO_Port, LCD_CLK_Pin);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	DWT_Init(); // счётчик для микросекундных пауз
	
	but_right_prev = HAL_GPIO_ReadPin(GPIOC, Push_R_Pin);// переменная для хранения предыдущего состояния стика R
	but_left_prev = HAL_GPIO_ReadPin(GPIOB, Push_L_Pin); // переменная для хранения предыдущего состояния стика L
	
	but_r_prev = HAL_GPIO_ReadPin(R_But_GPIO_Port, R_But_Pin);// переменная для хранения предыдущего состояния кнопки R
	but_l_prev = HAL_GPIO_ReadPin(L_But_GPIO_Port, L_But_Pin);;// переменная для хранения предыдущего состояния кнопки L
	
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
	
	const uint64_t pipe1 = 0xE8E8F0F0E2LL; // адрес приемника
	
	uint8_t connected = isChipConnected(); // проверка на подключените модуля к SPI
	
	uint8_t res = NRF_Init(); // инициализация

  ////////////// УСТАНОВКИ ////////////////
  enableAckPayload();
  setChannel(11); // 11 канал
  openWritingPipe(pipe1); // 1 труба
  ///////////////////////////////////
	

// ################################# DEBUG_RC OUT ############################


//HAL_Delay(2000);
//char str[64] = {0,};
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


// ############################# END OF DEBUG_RC OUT #########################
	
  maskIRQ(true, true, true); // маскируем прерывания

	// Запуск АЦП
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc, 4);
	HAL_ADC_Start_IT(&hadc1); // это здесь не нужно
	
	// Запуск ШИМ
	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_4);
	HAL_Delay(80);
	HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_4);

	LCD_init(); // Инициализация LCD

	HAL_GPIO_WritePin(LCD_LED_GPIO_Port, LCD_LED_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		menu(x);	
		//test();
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
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
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
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_5;
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
  htim4.Init.Period = 250;
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
  sConfigOC.Pulse = 125;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
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
	
	USB_Reset();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CE_Pin|CSN_Pin|LCD_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IRQ_Pin|LCD_CLK_Pin|LCD_DIN_Pin|LCD_DC_Pin
                          |LCD_CE_Pin|LCD_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Push_R_Pin */
  GPIO_InitStruct.Pin = Push_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Push_R_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CE_Pin CSN_Pin LCD_LED_Pin */
  GPIO_InitStruct.Pin = CE_Pin|CSN_Pin|LCD_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IRQ_Pin LCD_CLK_Pin LCD_DIN_Pin LCD_DC_Pin
                           LCD_CE_Pin LCD_RST_Pin */
  GPIO_InitStruct.Pin = IRQ_Pin|LCD_CLK_Pin|LCD_DIN_Pin|LCD_DC_Pin
                          |LCD_CE_Pin|LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Push_L_Pin */
  GPIO_InitStruct.Pin = Push_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Push_L_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : L_But_Pin R_But_Pin */
  GPIO_InitStruct.Pin = L_But_Pin|R_But_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
// для сброса USB
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

// функция для отображения текущего меню
void menu(unsigned char x)
{
	switch(x)
	{
		case 0: // находимся в главном меню
		while(x == 0) // 
		{
			// чтение текущего (нового) состояния кнопки
			unsigned char but_right_cur = HAL_GPIO_ReadPin(GPIOC, Push_R_Pin);
			unsigned char but_left_cur = HAL_GPIO_ReadPin(GPIOB, Push_L_Pin);
			
			unsigned char but_r_cur = HAL_GPIO_ReadPin(R_But_GPIO_Port, R_But_Pin);
			unsigned char but_l_cur = HAL_GPIO_ReadPin(L_But_GPIO_Port, L_But_Pin);
			
			// кнопка была нажата, если
			// прежнее состояние - отпущена (бит = 1), 
			// а новое состояние - нажата (бит = 0)
			if ((but_right_prev == GPIO_PIN_RESET) && (but_right_cur == GPIO_PIN_SET))
			{
				m++;
				if (m > 4)
				{
					m = 1;
				}
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			} 
				// запоминаем текущее состояние кнопки
				but_right_prev = but_right_cur;
			
		if ((but_left_prev == GPIO_PIN_RESET) && (but_left_cur == GPIO_PIN_SET))	
			{ 
				m--;
				if (m < 1)
				{
					m = 4;
				}
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			} 
				// запоминаем текущее состояние кнопки
			but_left_prev = but_left_cur;
			
			main_menu(m);
			led_lcd();
			
			if (twobuttons())
			{
				x = m;
				LCD_clrScr();
			}
		}
//		break;
		
		case 1: // в режиме полёта
		while(x == 1)
		{		 
			flight();
			if (twobuttons()) // если зажали две кнопки, то выходим
			{
				x = 0; // возвращаемся в главное меню
				m = 1;
				button = false; // Чтобы зайти с выкл мотором, если вышли с вкл
				LCD_clrScr();
			}
			led_lcd();
		}			
//		break;
		
		case 2: // находимся в настройках
		while(x == 2)
		{			// 
			settings();
			if (twobuttons()) // если зажали две кнопки, то выходим
			{
				x = 0; // возвращаемся в главное меню
				m = 2; // возвращаемся в главное меню на позицию настроек
				LCD_clrScr();
			}
			led_lcd();
		}			
//		break;
		
		case 3: // находимся в .....
		while(x == 3)
		{			
//			// чтение текущего (нового) состояния кнопки
//			unsigned char but_right_cur = HAL_GPIO_ReadPin(GPIOC, Push_R_Pin);
//			unsigned char but_left_cur = HAL_GPIO_ReadPin(GPIOB, Push_L_Pin);
//			
//			// кнопка была нажата, если
//			// прежнее состояние - отпущена (бит = 1), 
//			// а новое состояние - нажата (бит = 0)
//			if ((but_right_prev == GPIO_PIN_RESET) && (but_right_cur == GPIO_PIN_SET))
//			{
//				subtrim++;
//				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//			} 
//				// запоминаем текущее состояние кнопки
//				but_right_prev = but_right_cur;
//			
//			if ((but_left_prev == GPIO_PIN_RESET) && (but_left_cur == GPIO_PIN_SET))
//			{ 
//				subtrim--;
//				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//			} 
//				// запоминаем текущее состояние кнопки
//			but_left_prev = but_left_cur;
			
			sensors();
			if (twobuttons()) // если зажали две кнопки, то выходим
			{
				x = 0; // возвращаемся в главное меню
				m = 3; // позиция сенсоров
				LCD_clrScr();
			}
			led_lcd();
		}			
		break;
	}
}

//MENU
void main_menu(unsigned char x)
{
	LCD_print("MENU", 30, 0);
	switch(x)
	{
		case 1:
		LCD_invertText(true);
		LCD_print("Flight", 23, 1);
		LCD_invertText(false);
		LCD_print("Settings", 23, 2);
		LCD_print("Sensors", 23, 3);
		LCD_print("Somethings", 23, 4);
		break;
		
		case 2:
		LCD_print("Flight", 23, 1);
		LCD_invertText(true);
		LCD_print("Settings", 23, 2);
		LCD_invertText(false);
		LCD_print("Sensors", 23, 3);
		LCD_print("Somethings", 23, 4);
		break;
		
		case 3:
		LCD_print("Flight", 23, 1);
		LCD_print("Settings", 23, 2);
		LCD_invertText(true);
		LCD_print("Sensors", 23, 3);
		LCD_invertText(false);
		LCD_print("Somethings", 23, 4);
		break;
		
		case 4:
		LCD_print("Flight", 23, 1);
		LCD_print("Settings", 23, 2);
		LCD_print("Sensors", 23, 3);
		LCD_invertText(true);
		LCD_print("Somethings", 23, 4);
		LCD_invertText(false);
		break;
	}
}

// Режим полёта
void flight(void)
{
	LCD_print("Flight Mode", 10, 0);
	
	uint8_t nrf_data[32] = {0,}; // буфер указываем максимального размера
	
	if(flag)
	{
		flag = 0;

		HAL_ADC_Stop_DMA(&hadc1); // это необязательно
		
		adc1 = map(adc[1], 0, 4095, 1, 255); // 1 канал высота
		adc2 = map(adc[0], 0, 4095, 1, 255); // 0 канал элероны
		adc3 = map(adc[3], 0, 4095, 1, 255); // 9 канал АЦП - мотор
		adc4 = map(adc[4], 0, 4095, 1, 255); // 2 канал АЦП - киль
		
		LCD_print("X1:", 0, 1);
		LCD_var_str(18, 1, adc[4], 4);
		LCD_print("X2:", 43, 1);
		LCD_var_str(60, 1, adc[0], 4);

		LCD_print("Y1:", 0, 2);
		LCD_var_str(18, 2, adc[3], 4);
		LCD_print("Y2:", 43, 2);
		LCD_var_str(60, 2, adc[1], 4);
		
		float vbat = (3.3/4095)*adc[2]*kd * 10; // 3,3 В на делителе при 11,6 В максимальном, дальше делитель не имее смысла
		
		LCD_print("VBat:", 0, 3);
		LCD_var_str(30, 3, vbat, 3);
		LCD_print("Bat:", 43, 3);
		
		LCD_print("Signal:", 0, 4);
						
		///////////////////////////////////// ПЕРЕДАЧА /////////////////////////////////////////////
		nrf_data[0] = 77;
		nrf_data[1] = 86;
		nrf_data[2] = 97;
		
		nrf_data[3] = adc1;
		nrf_data[4] = adc2;
		nrf_data[7] = adc4;
		
		if (button){
		nrf_data[5] = adc3;
		}
		else{ 
			nrf_data[5] = 1;
		}	
		nrf_data[6] = 50; // установки не шлём
		
		uint8_t remsg = 0; // переменная для приёма байта пришедшего вместе с ответом
		float vbatreceiver; // для вычисления напряжения батареи еа приемнике
		
		if(write(&nrf_data, strlen((const char*)nrf_data))) // отправляем данные
		{
			if(isAckPayloadAvailable()) // проверяем пришло ли что-то вместе с ответом
			{
			read(&remsg, sizeof(remsg)); // получаем напряжение с батареи с ответом подтверждения
				vbatreceiver = remsg / 10; // вычитаем напругу батареи, делим на 10 чтобы получить реальное число
				LCD_var_str(65, 3, vbatreceiver, 3);
				LCD_print("Yes", 45, 4);
			}
		}
		else
		LCD_print("No", 45, 4);
		
		adc[0] = 0;
		adc[1] = 0;
		adc[2] = 0;
		adc[3] = 0;
		adc[4] = 0;
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc, 5);
	}
		
		// чтение текущего (нового) состояния кнопки
		unsigned char but_right_cur = HAL_GPIO_ReadPin(GPIOC, Push_R_Pin);
		unsigned char but_left_cur = HAL_GPIO_ReadPin(GPIOB, Push_L_Pin);
		
		if ((but_right_prev == GPIO_PIN_RESET) && (but_right_cur == GPIO_PIN_SET))
		{
			button = !button;
			// по нажатию R инвертируем светодиод PC13
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		} 
		but_right_prev = but_right_cur;
		
		if ((but_left_prev == GPIO_PIN_RESET) && (but_left_cur == GPIO_PIN_SET))
		{ 
			button = !button;
			// по нажатию R инвертируем светодиод PC13
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		} 
		but_left_prev = but_left_cur;
}

// Подменю настроек
void settings(void)
{
	uint8_t nrf_data[32] = {0,}; // буфер указываем максимального размера
	
	bool pin1 = HAL_GPIO_ReadPin(GPIOC, Push_R_Pin);
	bool pin2 = HAL_GPIO_ReadPin(GPIOB, Push_L_Pin);
	
	bool pin3 = HAL_GPIO_ReadPin(R_But_GPIO_Port, R_But_Pin);
	bool pin4 = HAL_GPIO_ReadPin(L_But_GPIO_Port, L_But_Pin);
	
	if(flag)
	{
		flag = 0;
		
		adc1 = map(adc[1], 0, 4095, 1, 255); // 1 канал высота
		adc2 = map(adc[0], 0, 4095, 1, 255); // 0 канал элероны
		adc3 = map(adc[3], 0, 4095, 1, 255); // 9 канал АЦП - мотор
		adc4 = map(adc[4], 0, 4095, 1, 255); // 2 канал АЦП - киль
		
		float vbat = (3.3/4095)*adc[2]*kd * 10;
		
		HAL_ADC_Stop_DMA(&hadc1);
	
		LCD_print("X1:", 0, 0);
		LCD_var_str(18, 0, adc[4], 4);
		LCD_print("X2:", 43, 0);
		LCD_var_str(60, 0, adc[0], 4);

		LCD_print("Y1:", 0, 1);
		LCD_var_str(18, 1, adc[3], 4);
		LCD_print("Y2:", 43, 1);
		LCD_var_str(60, 1, adc[1], 4);
		
		LCD_print("A5:", 0, 2);
		LCD_var_str(18, 2, adc[2], 4);
		
		LCD_print("VBat:", 0, 3);
		LCD_var_str(30, 3, vbat, 3);
		LCD_print("Bat:", 43, 3);
			
		LCD_print("Signal:", 0, 4);
	
		nrf_data[0] = 77;
		nrf_data[1] = 86;
		nrf_data[2] = 97;
		nrf_data[3] = adc1; // 
		nrf_data[4] = adc2; // 
		nrf_data[5] = 1;
		nrf_data[6] = 100; // чтобы приемник знал, что шлем установки
		nrf_data[7] = subtrim;
		nrf_data[8] = sens1;
		nrf_data[9] = sens2;
		nrf_data[10] = adc4;
	
		uint8_t remsg = 0; // переменная для приёма байта пришедшего вместе с ответом
		float vbatreceiver; // для вычисления напряжения батареи на приемнике
		
		if(write(&nrf_data, strlen((const char*)nrf_data))) // отправляем данные
		{
			if(isAckPayloadAvailable()) // проверяем пришло ли что-то вместе с ответом
			{
				read(&remsg, sizeof(remsg)); // получаем напряжение с батареи с ответом подтверждения
				vbatreceiver = remsg / 10; // вычитаем напругу батареи, делим на 10 чтобы получить реальное число
				LCD_var_str(65, 3, vbatreceiver, 3);
				LCD_print("Yes", 45, 4);
				snprintf(str_tx, 64, "Signal: Yes \r\n");
				CDC_Transmit_FS((unsigned char*)str_tx, strlen(str_tx));
			}
		}
		else
		snprintf(str_tx, 64, "Signal: Noo \r\n");
		CDC_Transmit_FS((unsigned char*)str_tx, strlen(str_tx));
		LCD_print("No", 45, 4);
		
		snprintf(str_tx, 120, "ADC1 X1:%d Y1:%d  M:%d Bat:%d Vbat:%d J_R:%d J_L:%d But_R:%d But_L:%d subtr:%d sens1:%d sens2:%d \r\n", nrf_data[3], nrf_data[4], adc3, (int)vbat*10, (int)vbatreceiver*10, 
		!pin1, !pin2, !pin3, !pin4, nrf_data[7], nrf_data[8], nrf_data[9]);
		CDC_Transmit_FS((uint8_t*)str_tx, strlen(str_tx));
	
		adc[0] = 0;
		adc[1] = 0;
		adc[2] = 0;
		adc[3] = 0;
		adc[4] = 0;
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc, 5);
	}
}

// Подменю датчиков
void sensors(void)
{
	//чтение текущего (нового) состояния кнопки
	unsigned char but_right_cur = HAL_GPIO_ReadPin(GPIOC, Push_R_Pin);
	unsigned char but_left_cur = HAL_GPIO_ReadPin(GPIOB, Push_L_Pin);
	unsigned char but_r_cur = HAL_GPIO_ReadPin(R_But_GPIO_Port, R_But_Pin);
	unsigned char but_l_cur = HAL_GPIO_ReadPin(L_But_GPIO_Port, L_But_Pin);
	
	uint8_t nrf_data[32] = {0,}; // буфер указываем максимального размера
	float vbat;
	
	if(flag)
	{
		flag = 0;

		HAL_ADC_Stop_DMA(&hadc1); // это необязательно
		
		adc1 = map(adc[1], 0, 4095, 1, 256); // 1 канал высота
		adc2 = map(adc[0], 0, 4095, 1, 256); // 0 канал элероны
		adc3 = map(adc[3], 0, 4095, 1, 256); // 9 канал АЦП - мотор
		adc4 = map(adc[4], 0, 4095, 1, 255); // 2 канал АЦП - киль
		
		vbat = (3.3/4095)*adc[2]*kd * 10; // 3,3 В на делителе при 11,6 В максимальном, дальше делитель не имее смысла
		
		adc[0] = 0;
		adc[1] = 0;
		adc[2] = 0;
		adc[3] = 0;
		adc[4] = 0;
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc, 5);
	}
	
	LCD_print("Sens1:", 0, 0);
	LCD_var_str(40, 0, sens1, 4);

	LCD_print("Subtrim:", 0, 1);
	LCD_var_str(40, 1, subtrim, 4);
	
	LCD_print("Sens2", 0, 2);
	LCD_var_str(40, 2, sens2, 4);
	
	LCD_print("VBat:", 0, 3);
	LCD_var_str(30, 3, vbat, 3);
	LCD_print("Bat:", 43, 3);
	LCD_print("Signal:", 0, 4);
	
	nrf_data[0] = 77;
	nrf_data[1] = 86;
	nrf_data[2] = 97;
	nrf_data[3] = adc1; // 
	nrf_data[4] = adc2; // 
	nrf_data[5] = 1;
	nrf_data[6] = 100; // чтобы приемник знал, что шлем установки
	nrf_data[7] = subtrim;
	nrf_data[8] = sens1;
	nrf_data[9] = sens2;
	nrf_data[10] = adc4;
	
	uint8_t remsg = 0; // переменная для приёма байта пришедшего вместе с ответом
	float vbatreceiver; // для вычисления напряжения батареи на приемнике
	
	if(write(&nrf_data, strlen((const char*)nrf_data))) // отправляем данные
	{
		if(isAckPayloadAvailable()) // проверяем пришло ли что-то вместе с ответом
		{
		read(&remsg, sizeof(remsg)); // получаем напряжение с батареи с ответом подтверждения
			vbatreceiver = remsg / 10; // вычитаем напругу батареи, делим на 10 чтобы получить реальное число
			LCD_var_str(65, 3, vbatreceiver, 3);
			snprintf(str_tx, 64, "Bat: %d \r\n", (int)vbatreceiver*10);
			CDC_Transmit_FS((unsigned char*)str_tx, strlen(str_tx));
			LCD_print("Yes", 45, 4);
		}
	}
	else
	LCD_print("No", 45, 4);
	
	snprintf(str_tx, 63, "ADC1 X1:%d Y1:%d  M:%d Ch:%d subtr:%d\n", nrf_data[3], nrf_data[4], nrf_data[5], nrf_data[6], nrf_data[7]);
	CDC_Transmit_FS((uint8_t*)str_tx, strlen(str_tx));
	
	// кнопка была нажата, если
	// прежнее состояние - отпущена (бит = 1), 
	// а новое состояние - нажата (бит = 0)
	if ((but_right_prev == GPIO_PIN_RESET) && (but_right_cur == GPIO_PIN_SET))
	{
		subtrim += 5;
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		// Чтобы не вылезти за границу
		if (subtrim > 140)
		{
			subtrim = 0;
		}
	} 
	but_right_prev = but_right_cur;
	
	if ((but_left_prev == GPIO_PIN_RESET) && (but_left_cur == GPIO_PIN_SET))
	{ 
		subtrim -= 5;
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		// Чтобы не вылезти за 0
		if (subtrim > 140)
		{
			subtrim = 0;
		}
	}
	// запоминаем текущее состояние кнопки
	but_left_prev = but_left_cur;
	
	//Увеличиваем чувствительность
	if ((but_r_prev == GPIO_PIN_RESET) && (but_r_cur == GPIO_PIN_SET) )
	{
		if (HAL_GPIO_ReadPin(GPIOB, Push_L_Pin))
		{
			sens1 += 5;
			if (sens1 > 60)
			{
				sens1 = 0;
			}
		}
			else
			{
				sens2 += 5;
				if (sens2 > 60)
				{
					sens2 = 30;
				}
			}
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	} 
	but_r_prev = but_r_cur;
	
	//Уменьшаем чувствительность при зажатой правой кнопке
	if ((but_l_prev == GPIO_PIN_RESET) && (but_l_cur == GPIO_PIN_SET))
	{
		if (HAL_GPIO_ReadPin(GPIOB, Push_L_Pin))
		{
		sens1 -= 5;
			if (sens1 > 60)
			{
				sens1 = 0;
			}
		}
		else
			{
				sens2 -= 5;
				if (sens2 > 60)
				{
					sens2 = 0;
				}
			}
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

	}
	but_l_prev = but_l_cur;
}

//вкл/выкл подсветки
void led_lcd(void)
{
	uint32_t timme = HAL_GetTick();
	while ((HAL_GPIO_ReadPin(R_But_GPIO_Port, R_But_Pin) == RESET) && (HAL_GPIO_ReadPin(L_But_GPIO_Port, L_But_Pin) == RESET))
	{	
		// пока нажаты кнопки проверяем на таймер
		if((HAL_GetTick() - timme) > 500) // интервал 500мс = 0.5 сек 
		{
			HAL_GPIO_TogglePin(LCD_LED_GPIO_Port, LCD_LED_Pin);
		}
	}
}

// кнопка правого стика 
bool right_button(void)
{
	unsigned char but_right_cur = HAL_GPIO_ReadPin(GPIOC, Push_R_Pin);
	if ((but_right_prev == GPIO_PIN_RESET) && (but_right_cur == GPIO_PIN_SET))
	{ 
		return 1;
	}
	// запоминаем текущее состояние кнопки
	but_right_prev = but_right_cur;
	return 0;
}

// кнопка левого стика 
bool left_button(void)
{
	unsigned char but_left_cur = HAL_GPIO_ReadPin(GPIOB, Push_L_Pin);
	if ((but_left_prev == GPIO_PIN_RESET) && (but_left_cur == GPIO_PIN_SET))
	{ 
		return 1;
	} 
	// запоминаем текущее состояние кнопки
	but_left_prev = but_left_cur;
	return 0;
}

// Функция для перевода одного диапазаона в другой
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//Функция на одновременное нажатие кнопок
bool twobuttons(void)
{
	uint32_t timme = HAL_GetTick();
	// пока нажаты кнопки проверяем на таймер
	while((HAL_GPIO_ReadPin(GPIOB, Push_L_Pin) == GPIO_PIN_RESET) && (HAL_GPIO_ReadPin(GPIOC, Push_R_Pin) == GPIO_PIN_RESET))
	{ 
		if((HAL_GetTick() - timme) > 1000) // интервал 1000мс = 1сек 
		{ 
			// попадаем в цикл и ждём отпускания кнопок, чтобы не сработало ещё раз где нибудь, где ожидается нажатие двух кнопок
			while((HAL_GPIO_ReadPin(GPIOB, Push_L_Pin) == GPIO_PIN_SET) && (HAL_GPIO_ReadPin(GPIOC, Push_R_Pin) == GPIO_PIN_SET))
			{
	//			;
			}
			return 1;
		}
	}
	return 0;
}

//speaker
void speaker (void)
{
	uint32_t timme = HAL_GetTick();
	//запуск PWM
	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1);
	if((HAL_GetTick() - timme) > 500) // интервал 1000мс = 1сек 
	{
		HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_1);
	}
}

void test(void)
{
	if(flag)
	{
		flag = 0;
		
		HAL_ADC_Stop_DMA(&hadc1);
	
		LCD_print("X1:", 0, 0);
		LCD_var_str(18, 0, adc[4], 4);
		LCD_print("X2:", 43, 0);
		LCD_var_str(60, 0, adc[0], 4);

		LCD_print("Y1:", 0, 1);
		LCD_var_str(18, 1, adc[3], 4);
		LCD_print("Y2:", 43, 1);
		LCD_var_str(60, 1, adc[1], 4);
		
		LCD_print("A5:", 0, 2);
		LCD_var_str(18, 2, adc[2], 4);

		float vbat = (3.3/4095) * adc[2] * kd;
		
		LCD_print("VBat:", 0, 3);
		LCD_var_str(30, 3, vbat, 3);
		LCD_print("Bat:", 43, 3);
			
		LCD_print("Signal:", 0, 4);

		snprintf(str_tx, 120, "Ch0: %d Ch1: %d Ch2: %d Ch3: %d Ch4: %d Vbat: %.3f \r\n", adc[0], adc[1], adc[2], adc[3], adc[4], (float)vbat);
		CDC_Transmit_FS((uint8_t*)str_tx, strlen(str_tx));
		HAL_Delay(100);
	
		adc[0] = 0;
		adc[1] = 0;
		adc[2] = 0;
		adc[3] = 0;
		adc[4] = 0;
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc, 5);
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
