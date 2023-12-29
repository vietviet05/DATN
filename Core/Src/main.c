/* USER CODE BEGIN Header */
/**
  ******************************************************************************
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
#include "ssd1306.h"
#include "fonts.h"
#include "test.h" 
#include "math.h"
#include "Button.h"
#include "RTC.h" 
#include "DHT.h" 
#include "display.h"   

#define ADDRESS_DATA_STORAGE 0x800FC00
#define NUM_PPM_VALUES 10
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

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
float Ro = 40;
float Rs;
float RL = 2;
double ppm ; 
float Flashback; 
float Settime; 
float SP;
DHT_Name dht22; 
uint8_t gas_change; 
uint8_t display_se; 
uint8_t menu = 0; 
float ppmArray[NUM_PPM_VALUES]; 

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
Button_Typdef button1;
Button_Typdef button2;
Button_Typdef button3;
Button_Typdef button4; 
uint16_t adc1;  
uint32_t timecurrent; 

RTC_Typedef hrtc1;
//uint8_t * numberToString(uint16_t val){
//	if (val == 0) return (uint8_t *)"0";
//  
//	static uint8_t buf[32] = {0};
//	
//	int i = 30;
//	
//	for(; val && i ; --i, val /= 10)
//	
//		buf[i] = "0123456789abcdef"[val % 10];
//	
//	return &buf[i+1];
//} 


void updatePPM(float newPPMValue) {
    for (int i = NUM_PPM_VALUES - 1; i > 0; i--) {
        ppmArray[i] = ppmArray[i - 1];
    }
    ppmArray[0] = newPPMValue;
}
void savePPMArrayToFlash() { 
    flash_Erase(ADDRESS_DATA_STORAGE);
    flash_Write_Array(ADDRESS_DATA_STORAGE, (uint8_t*)ppmArray, NUM_PPM_VALUES * sizeof(float));
} 
void readAndDisplayPPMArrayFromFlash() { 
	
    flash_Read_Array(ADDRESS_DATA_STORAGE, (uint8_t*)ppmArray, NUM_PPM_VALUES * sizeof(float));
		SSD1306_Fill(SSD1306_COLOR_BLACK);
		SSD1306_GotoXY (1, 20);
		for (int i = 0; i < NUM_PPM_VALUES; i++) {    
    SSD1306_Puts((char *)numberToString(ppmArray[i]), &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_Puts (":", &Font_7x10, 1);  }
}
void getmeasured(){
HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1000); 
		adc1 = HAL_ADC_GetValue(&hadc1);  
		HAL_ADC_Stop(&hadc1);    
    Rs = (5*RL/(adc1*3.3/4095))-RL;
	
   const float slopeLPG = -0.745;
   const float scaleLPG = 17.66; 
	 const float slopeCH4 = -0.569;
   const float scaleCH4 = 8.23;
	 if (gas_change == 0) 
		  { 
				ppm = pow(Rs / (Ro * scaleLPG), 1 / slopeLPG); 
			}
   else {
		 ppm = pow(Rs / (Ro * scaleCH4), 1 / slopeCH4);
	      } 			
		DHT_ReadTempHum(&dht22); //doc nhiet do, do am
}
void	btn_pressing_callback(Button_Typdef *ButtonX) {
if(ButtonX == &button3) {
	display_se = 2;
	if (display_se == 2&&ButtonX == &button3 ){
      menu ++; 
 menu = menu % 3;	}
}
if(ButtonX == &button4) { 
	if (display_se == 2&&menu ==1){
      display_se = 3;  
	if (ButtonX == &button4){
	//SSD1306_ScrollRight(1, 3);
	}
 }
}

}
void btn_press_short_callback(Button_Typdef *ButtonX)
{ 
	if(ButtonX == &button1)
	{ 
		display_se = 0;
		getmeasured();
		updatePPM(ppm);
		savePPMArrayToFlash();
		HAL_Delay(10);  
	}  
	if(ButtonX == &button2)
	{ 
		gas_change = (gas_change == 0) ? 1 : 0;	 
	}  
	if(ButtonX == &button3)
	{ 
	
	} 
	
} 
void btn_press_timeout_callback(Button_Typdef *ButtonX)
{ 
 
	if(ButtonX == &button1)
	{ display_se = 1;
		// say somthing  
		
	} 
	if(ButtonX == &button2) 
		{
    display_se = 2;
//		menu = 0;	 	
		}
//	if(ButtonX == &button3) 
//		{
//    display_se = 2;
//		menu = 0;	 	
//		}
} 
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	RTC_Init(&hrtc1,&hi2c2); 
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	DHT_Init(&dht22, DHT22, &htim4, GPIOB, GPIO_PIN_13);
	button_init(&button1,GPIOB,GPIO_PIN_12);
	button_init(&button2,GPIOA,GPIO_PIN_11);
	button_init(&button3,GPIOB,GPIO_PIN_14);
	button_init(&button4,GPIOB,GPIO_PIN_15);
	HAL_ADCEx_Calibration_Start(&hadc1);   
	SSD1306_Init (); // initialize the display 
//  HAL_Delay(200); 

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  button_handle(&button1);
		button_handle(&button2); 
		button_handle(&button3);
		button_handle(&button4);
		RTC_ReadTime(&hrtc1,&hrtc1.date_time);
		if(ppm > 1000 && HAL_GetTick() - timecurrent >= 1000)
    { HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	    timecurrent = HAL_GetTick(); 
			getmeasured();
		} 
//		if (display_se == 0 && gas_change == 0){ 
//		SSD1306_GotoXY (1, 50);  
//		SSD1306_Fill(SSD1306_COLOR_BLACK);
//		SSD1306_Puts((char *)numberToString(hrtc1.date_time.hour), &Font_7x10, SSD1306_COLOR_WHITE);
//		SSD1306_Puts (":", &Font_7x10, 1); 
//		SSD1306_Puts((char *)numberToString(hrtc1.date_time.min), &Font_7x10, SSD1306_COLOR_WHITE); 
//		SSD1306_Puts (":", &Font_7x10, 1); 
//		SSD1306_Puts((char *)numberToString(hrtc1.date_time.second), &Font_7x10, SSD1306_COLOR_WHITE); 
//		SSD1306_GotoXY (1,1);  
//		SSD1306_Puts ("LPG:", &Font_7x10, 1);  
//		SSD1306_Puts((char *)numberToString(ppm), &Font_11x18, SSD1306_COLOR_WHITE); 
//    SSD1306_Puts ("ppm", &Font_7x10, 1);  
//	//	readAndDisplayPPMArrayFromFlash(); 
//		  
//  	SSD1306_UpdateScreen(); // update screen 
//		} 
//		else if (display_se == 0&&gas_change == 1) 
//    {
//		SSD1306_GotoXY (1, 50);  
//		SSD1306_Fill(SSD1306_COLOR_BLACK);
//		SSD1306_Puts((char *)numberToString(hrtc1.date_time.hour), &Font_7x10, SSD1306_COLOR_WHITE);
//		SSD1306_Puts (":", &Font_7x10, 1); // print Hello 
//		SSD1306_Puts((char *)numberToString(hrtc1.date_time.min), &Font_7x10, SSD1306_COLOR_WHITE); 
//		SSD1306_Puts (":", &Font_7x10, 1); // print Hello 
//		SSD1306_Puts((char *)numberToString(hrtc1.date_time.second), &Font_7x10, SSD1306_COLOR_WHITE); 
//		SSD1306_GotoXY (1,1); // goto 10, 10 
//		SSD1306_Puts ("concent_LPG:", &Font_7x10, 1);  
//		SSD1306_Puts((char *)numberToString(ppm), &Font_7x10, SSD1306_COLOR_WHITE); 
//    SSD1306_Puts ("ppm", &Font_7x10, 1); 
//		SSD1306_UpdateScreen();	// update screen
//		}			
//		else if (display_se == 1) 
//    {
//		SSD1306_Fill(SSD1306_COLOR_BLACK);
//		Ro = 5*RL/(adc1*3.3/4095)-RL;
//		SSD1306_GotoXY (1,1); // goto 10, 10 
//    SSD1306_Puts ("da hieu chinh thiet bi", &Font_7x10, 1); // print Hello  
//		SSD1306_Puts ("Ro = ", &Font_7x10, 1); // print Hello 	
//    SSD1306_Puts((char *)numberToString(Ro), &Font_7x10, SSD1306_COLOR_WHITE);
////		HAL_Delay(100);
////		//SSD1306_Puts ("WORLD !!", &Font_7x10, 1); 
//    SSD1306_UpdateScreen(); // update screen
//		}	
   display();
		
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
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
  /* USER CODE BEGIN ADC1_Init 2 */

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
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
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
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xffff -1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|Menu_Pin|Select_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DHT22_GPIO_Port, DHT22_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(button_2_GPIO_Port, button_2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : Led_Pin */
  GPIO_InitStruct.Pin = Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 DHT22_Pin Menu_Pin Select_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_12|DHT22_Pin|Menu_Pin|Select_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : button_2_Pin */
  GPIO_InitStruct.Pin = button_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(button_2_GPIO_Port, &GPIO_InitStruct);

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
