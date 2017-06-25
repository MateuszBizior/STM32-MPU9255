/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t Received;
uint8_t Settings = 0x00;
uint8_t SetK= 0x02;
uint8_t SetKL= 0x12;

uint8_t DAHx;
uint8_t DALx;
uint8_t DAHy;
uint8_t DALy;
uint8_t DAHz;
uint8_t DALz;
uint8_t DGHx;
uint8_t DGLx;
uint8_t DGHy;
uint8_t DGLy;
uint8_t DGHz;
uint8_t DGLz;
uint8_t DHT;
uint8_t DLT;
uint8_t HLx;
uint8_t HLy;
uint8_t HLz;
uint8_t HHx;
uint8_t HHy;
uint8_t HHz;
uint8_t H;
uint8_t Htab[7];
uint8_t Mtab[14];
uint8_t ST1;
uint8_t ST2;

int16_t Ax;
int16_t Ay;
int16_t Az;
int16_t Gx;
int16_t Gy;
int16_t Gz;
int16_t Hx;
int16_t Hy;
int16_t Hz;
int16_t T;

float Ax_g;
float Ay_g;
float Az_g;
float Gx_g;
float Gy_g;
float Gz_g;
float Hx_g;
float Hy_g;
float Hz_g;
float Hx_n;
float Hy_n;
float Hz_n;
float H_n;
int work=0;
float arx;
float ary;
float arz;
float rx;
float ry;
float rz;
uint8_t flaga=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(atoi(&Received)==0)
	{
		flaga=0;
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		HAL_UART_Receive_IT(&huart1, &Received, 1);
	}
	if(atoi(&Received)==1)
	{
		flaga=1;
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		HAL_UART_Receive_IT(&huart1, &Received, 1);
	}
	if(atoi(&Received)==2)
	{
		flaga=2;
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		HAL_UART_Receive_IT(&huart1, &Received, 1);
	}

}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{


	if(htim->Instance == TIM2)
	{

	}
	if(htim->Instance == TIM3)
	{
		if (work>=100)
		{	//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			uint8_t Data[40]; // Tablica przechowujaca wysylana wiadomosc.
			uint16_t size = 0; // Rozmiar wysylanej wiadomosci
			work=0;
			if(flaga==2)
			{
				size = sprintf(Data, "P%dzR%dzJ%dzX%dzY%dzZ%dzA%dzB%dzC%dzn",(int)rx,(int)ry,(int)rz,(int)Hx_g,(int)Hy_g,(int)Hz_g,(int)ary,(int)arx,(int)arz);
				HAL_UART_Transmit(&huart1, Data, size,1000);
			}
			if(flaga==1)
			{
				size = sprintf(Data, "D%dzE%dzF%dzG%dzH%dzI%dzn",(int)(Ax_g*100),(int)(Ay_g*100),(int)(Az_g*100),(int)(Gx_g*100),(int)(Gy_g*100),(int)(Gz_g*100));
				HAL_UART_Transmit(&huart1, Data, size,1000);
			}
		}
		else work=work+1;

		// HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		 rx = (0.02 * ary) + (0.001 * Gx_g+rx)*0.98;
		 ry = (0.02 * arx) + (0.001 * (-Gy_g)+ry)*0.98;
		 rz = (0.02 * ary) + (0.001 * Gz_g+rz)*0.98;

	}

}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  //w³¹czenie przerwan od uart
  HAL_UART_Receive_IT(&huart1, &Received, 1);
  //ustawienia akcelerometru i zyroskopu
  HAL_I2C_Mem_Write(&hi2c1, 0xD0, 27, I2C_MEMADD_SIZE_8BIT, &Settings,1,1000);
  HAL_I2C_Mem_Write(&hi2c1, 0xD0, 28, I2C_MEMADD_SIZE_8BIT, &Settings,1,1000);
  HAL_I2C_Mem_Write(&hi2c1, 0xD0, 55, I2C_MEMADD_SIZE_8BIT, &SetK,1,1000);
  HAL_I2C_Mem_Write(&hi2c1, 0x18, 0x0A, I2C_MEMADD_SIZE_8BIT, &SetKL,1,1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	  //akcelerometr oœ X
	 /*	  HAL_I2C_Mem_Read(&hi2c1, 0xD0, 59, I2C_MEMADD_SIZE_8BIT, &DAHx,1,100);
	 	  HAL_I2C_Mem_Read(&hi2c1, 0xD0, 60, I2C_MEMADD_SIZE_8BIT, &DALx,1,100);
	 	  //akcelerometr oœ Y
	 	  HAL_I2C_Mem_Read(&hi2c1, 0xD0, 61, I2C_MEMADD_SIZE_8BIT, &DAHy,1,100);
	 	  HAL_I2C_Mem_Read(&hi2c1, 0xD0, 62, I2C_MEMADD_SIZE_8BIT, &DALy,1,100);
	 	  //akcelerometr oœ Z
	 	  HAL_I2C_Mem_Read(&hi2c1, 0xD0, 63, I2C_MEMADD_SIZE_8BIT, &DAHz,1,100);
	 	  HAL_I2C_Mem_Read(&hi2c1, 0xD0, 64, I2C_MEMADD_SIZE_8BIT, &DALz,1,100);
	 	  //Temperatura
	 	  HAL_I2C_Mem_Read(&hi2c1, 0xD0, 65, I2C_MEMADD_SIZE_8BIT, &DHT,1,100);
	 	  HAL_I2C_Mem_Read(&hi2c1, 0xD0, 66, I2C_MEMADD_SIZE_8BIT, &DLT,1,100);
	 	  //zyroskop oœ X
	 	  HAL_I2C_Mem_Read(&hi2c1, 0xD0, 67, I2C_MEMADD_SIZE_8BIT, &DGHx,1,100);
	 	  HAL_I2C_Mem_Read(&hi2c1, 0xD0, 68, I2C_MEMADD_SIZE_8BIT, &DGLx,1,100);
	 	  //zyroskop oœ Y
	 	  HAL_I2C_Mem_Read(&hi2c1, 0xD0, 69, I2C_MEMADD_SIZE_8BIT, &DGHy,1,100);
	 	  HAL_I2C_Mem_Read(&hi2c1, 0xD0, 70, I2C_MEMADD_SIZE_8BIT, &DGLy,1,100);
	 	  //zyroskop oœ Z
	 	  HAL_I2C_Mem_Read(&hi2c1, 0xD0, 71, I2C_MEMADD_SIZE_8BIT, &DGHz,1,100);
	 	  HAL_I2C_Mem_Read(&hi2c1, 0xD0, 72, I2C_MEMADD_SIZE_8BIT, &DGLz,1,100);*/

	 	  HAL_I2C_Mem_Read(&hi2c1, 0xD0, 59, I2C_MEMADD_SIZE_8BIT, &Mtab[0],14,1000);

	 	  HAL_I2C_Mem_Read(&hi2c1, 0x18, 0x02, I2C_MEMADD_SIZE_8BIT, &ST1,1,100);

	 		  if((ST1&0x01))
	 		  {
	 			HAL_I2C_Mem_Read(&hi2c1, 0x18, 0x03, I2C_MEMADD_SIZE_8BIT, &Htab[0],6,1000);
	 	 		HAL_I2C_Mem_Read(&hi2c1, 0x18, 0x09, I2C_MEMADD_SIZE_8BIT, &ST2,1,100);
	 	 		Hx= (int16_t)(Htab[1] << 8 | Htab[0]);
	 	 		Hy= (int16_t)(Htab[3] << 8 | Htab[2]);
	 	 		Hz= (int16_t)(Htab[5] << 8 | Htab[4]);
	 	 		Hx_g=(float)Hx*0.15;
	 	 		Hy_g=(float)Hy*0.15;
	 	 		Hz_g=(float)Hz*0.15;
	 		  }
	 	 				  //akcelerometr
	 	 				  Ax = (int16_t)(Mtab[0] << 8 | Mtab[1]);
	 	 				  Ay = (int16_t)(Mtab[2] << 8 | Mtab[3]);
	 	 				  Az = (int16_t)(Mtab[4] << 8 | Mtab[5]);
	 	 				  //temperatura
	 	 				  T = (int16_t)(Mtab[6] << 8 | Mtab[7]);
	 	 				  //zyroskop
	 	 				  Gx = (int16_t)(Mtab[8] << 8 | Mtab[9]);
	 	 				  Gy = (int16_t)(Mtab[10] << 8 | Mtab[11]);
	 	 				  Gz = (int16_t)(Mtab[12] << 8 | Mtab[13]);

	 	 				  Ax_g = (float)Ax/16384.0;
	 	 				  Ay_g = (float)Ay/16384.0;
	 	 				  Az_g = (float)Az/16384.0;
	 	 				  Gx_g = (float)Gx/131.0;
	 	 				  Gy_g = (float)Gy/131.0;
	 	 				  Gz_g = (float)Gz/131.0;


	 	 				 arx = (180.0/3.141592) * atan2(Ax_g, sqrtf((Ay_g*Ay_g + Az_g*Az_g)));
	 	 				 ary = (180.0/3.141592) * atan2(Ay_g, sqrtf((Ax_g*Ax_g + Az_g*Az_g)));
	 	 				 arz = (180.0/3.141592) * atan2(Az_g, sqrtf((Ax_g*Ax_g + Ay_g*Ay_g)));



  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 9999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 36;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
