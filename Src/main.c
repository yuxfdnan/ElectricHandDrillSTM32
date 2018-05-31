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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
//#include "stdio.h"
//#include  <math.h>

#include "user_conf.h"
//#include "Low_level.h"
#include "BLDC_TIM.h"
#include "BLDC_Motor.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define N     6
uint16_t ADCConvertedValue_2[N];
uint32_t ADC_Value1[30];

//uint8_t i;
//		AD1		AD2			AD3		AD4		AD5			AD6
uint32_t MidAD,CurrentAD,SwtichAD,MotorAD_A,MotorAD_B,MotorAD_C;


unsigned long BEMF_Cnt;
unsigned long BEMF_ADC_Cnt;

//unsigned short ADC_Value[6000][3];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM1_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
unsigned long BEMF(void);//Back Electromotive Force 反电动
void MyInitTIM1PWM(void);


//void BLDC_Init(void)
//{
//    MotorA.PWMTicks = 0;
//
//    MotorA.Step = 0;
//    MotorA.State = 0;
//
//    MotorA.FlagSwitchStep = 0;
//    MotorA.FlagBEMF = 0;
//}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t i;
	uint32_t tmpUint32[6];
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
  MX_DMA_Init();
  MX_ADC_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
  MyInitTIM1PWM();
  BLDC_Init();

  BEMF_Cnt = 0;
  BEMF_ADC_Cnt = 0;

  MotorA.Step = 0;

//  BLDC_SwitchStep();//Del for test

  /*1 Start the conversion process and enable interrupt */
  HAL_ADC_Start_DMA(&hadc, (uint32_t*)&ADC_Value1, 30);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //Only For test

//  HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0 上桥关
//  __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_1,TIM_OCNPOLARITY_HIGH);//有效打开
//  HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0 下桥关
//
//
//  /*  PhaseB configuration */
//  HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_2);// 1 上桥开
//  HAL_TIMEx_OCN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//下桥开,同步整流
//
  /*  PhaseC configuration */
  //CC3NP/CC3P没作用
//  HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 0 上桥关
//  __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_3,TIM_OCNPOLARITY_LOW);// 1  常开,一直打开
//  HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 1 下桥常开,一直打开
//  HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 0 上桥关
//  __HAL_TIM_SET_COMPARE(&BLDC_TIMER_NUM,TIM_CHANNEL_3,48000000/BLDC_PWM_Freq - 1);TIM_CCMR1_OC1M_0
//  BLDC_TIMER_NUM.Instance->CCMR1 |= TIM_CCMR1_OC1M_0;

  //CHANNEL_1
  //置位101:强制为有效电平.OC_Start OCN_Start -> OC高 OCN低
  //置位101:强制为有效电平.OC_Stop OCN_Start -> OC低 OCN高
  //置位101:强制为有效电平.OC_Start OCN_Stop -> OC高 OCN低
  //置位101:强制为有效电平.OC_Stop OCN_Stop -> OC低 OCN低

  //置位100:强制为无效电平.OC_Start OCN_Start -> OC低 OCN高
  //置位100:强制为无效电平.OC_Stop OCN_Start -> OC低 OCN低
  //置位100:强制为无效电平.OC_Start OCN_Stop -> OC低 OCN低
  //置位100:强制为无效电平.OC_Stop OCN_Stop -> OC低 OCN低
  BLDC_TIMER_NUM.Instance->CCMR1 &= ~(TIM_CCMR1_OC1M_0|TIM_CCMR1_OC1M_1|TIM_CCMR1_OC1M_2);//清零
  BLDC_TIMER_NUM.Instance->CCMR1 |= (TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_2);//置位101:强制为有效电平.强制OC1REF 为高。
//  BLDC_TIMER_NUM.Instance->CCMR1 |= (TIM_CCMR1_OC1M_2);//置位100:强制为无效电平.强制OC1REF 为低。
//  HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_1);
  HAL_TIMEx_OCN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_1);// 1 下桥常开,一直打开
  HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);
//  HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);// 1 下桥常开,一直打开
  HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_2);
  HAL_TIMEx_OCN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_2);
//  HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);
//  HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_3);



  MotorA.State = 0;
//  HAL_TIM_PWM_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_2);
//  HAL_TIMEx_PWMN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_2);
//  BLDC_SwitchStep();
  while (1)
  {
      //等待ADC转换，DMA传输完成
      //清除DMA传输完成标志位

	  //计算AD平均值
	  tmpUint32[0] = 0;
	  tmpUint32[1] = 0;
	  tmpUint32[2] = 0;
	  tmpUint32[3] = 0;
	  tmpUint32[4] = 0;
	  tmpUint32[5] = 0;
	  MidAD = 0;
	  CurrentAD = 0;
	  SwtichAD = 0;
	  MotorAD_A = 0;
	  MotorAD_B = 0;
	  MotorAD_C = 0;
	  for(i = 0; i < 5; i ++)
	  {
		  tmpUint32[0] += ADC_Value1[i*6+0];
		  tmpUint32[1] += ADC_Value1[i*6+1];
		  tmpUint32[2] += ADC_Value1[i*6+2];
		  tmpUint32[3] += ADC_Value1[i*6+3];
		  tmpUint32[4] += ADC_Value1[i*6+4];
		  tmpUint32[5] += ADC_Value1[i*6+5];
	  }
	  MidAD = tmpUint32[0]/5;
	  CurrentAD = tmpUint32[1]/5;
	  SwtichAD = tmpUint32[2]/5;
	  MotorAD_A = tmpUint32[3]/5;
	  MotorAD_B = tmpUint32[4]/5;
	  MotorAD_C = tmpUint32[5]/5;
	  ADCConvertedValue_2[0] = MotorAD_A;
	  ADCConvertedValue_2[1] = MotorAD_A;
	  ADCConvertedValue_2[2] = MotorAD_A;



	  //TODO
//	  MotorA.Step = 1;
//	  BLDC_SwitchStep();
//       switch(MotorA.State)
//       {
//		   case 0:                             //定位
//			   if(MotorA.PWMTicks >= 1000)
//			   {
//	//               BLDC_Ctrl_Board_LEDOn(LED4);
//
//				   //   MotorA.FlagSwitchStep = 1;   //换向
//				   BLDC_SwitchStep();
//				   MotorA.State++;
//
//	//               BLDC_Ctrl_Board_LEDOff(LED4);
//				   BEMF_Cnt = 0;
//			   }
//
//			   break;
//		   case 1:                             //启动
//
//			   if (MotorA.PWMTicks >= 8)
//			   {
//				   if ( BEMF())
//				   {
//	//                   BLDC_Ctrl_Board_LEDOn(LED1);
//					   //  MotorA.FlagSwitchStep = 1;   //换向
//					   BLDC_SwitchStep();
//					   BEMF_Cnt++;
//	//                   BLDC_Ctrl_Board_LEDOff(LED1);
//				   }
//			   }
//
//			   if (BEMF_ADC_Cnt < 6000)                   //记录启动过程bemf数据
//			   {
//	//               ADC_Value[BEMF_ADC_Cnt][0] = ADCConvertedValue_2[0];
//	//               ADC_Value[BEMF_ADC_Cnt][1] = ADCConvertedValue_2[1];
//	//               ADC_Value[BEMF_ADC_Cnt][2] = ADCConvertedValue_2[2];
//
//				   BEMF_ADC_Cnt++;
//			   }
//
//			   if (BEMF_Cnt >= 50)       //50次换向之后，认为达到平稳状态
//			   {
//				   MotorA.State++;
//				   BEMF_Cnt = 0;
//			   }
//			   break;
//
//		   case 2:
//			   if (MotorA.FlagBEMF == 0)      //未检测到过零事件
//			   {
//				   if (MotorA.PWMTicks >= 8)
//				   {
//					   if ( BEMF())
//					   {
//	//                       BLDC_Ctrl_Board_LEDOn(LED3);
//						   MotorA.FlagSwitchStep = MotorA.PWMTicksPre >> 1;   //延迟30电角度
//						   MotorA.FlagBEMF = 1;             //检测到过零事件，不再检测
//	//                       BLDC_Ctrl_Board_LEDOff(LED3);
//					   }
//				   }
//			   }
//			   else
//			   {
//				   if (MotorA.FlagSwitchStep == 0)          //延迟时刻到
//				   {
//	//                   BLDC_Ctrl_Board_LEDOn(LED1);
//					   BLDC_SwitchStep();               //换向
//					   BEMF_Cnt++;
//	//                   BLDC_Ctrl_Board_LEDOff(LED1);
//				   }
//				   else
//				   {
//					   MotorA.FlagSwitchStep--;
//				   }
//
//			   }
//
//			   if (BEMF_Cnt >= 80000)       //50000次换向之后，停止
//			   {
//				   MotorA.State++;
//				   BEMF_Cnt = 0;
//			   }
//
//			   break;
//
//		   case 3:
//			   BLDC_Stop();
//			   break;
//		   case 4:
//
//			   break;
//		   default:
//			   break;
//       }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

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
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

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

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_6;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim1.Init.Period = 48000000/BLDC_PWM_Freq - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 48000000/BLDC_PWM_Freq/3 - 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.Pulse = 48000000/BLDC_PWM_Freq - 100;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 64;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Ctrl_Pin|SwtichIn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Led1_Pin|Led2_Pin|Led3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Ctrl_Pin SwtichIn_Pin */
  GPIO_InitStruct.Pin = LED_Ctrl_Pin|SwtichIn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Swtich_Pin */
  GPIO_InitStruct.Pin = Swtich_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Swtich_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Led1_Pin Led2_Pin Led3_Pin */
  GPIO_InitStruct.Pin = Led1_Pin|Led2_Pin|Led3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Key1_Pin Key2_Pin */
  GPIO_InitStruct.Pin = Key1_Pin|Key2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
unsigned long BEMF(void)
{

    unsigned short VoltBEMF = 0;
    unsigned long dir = 0;

    switch (MotorA.Step)
    {
    case 0:
        VoltBEMF = ADCConvertedValue_2[0];
        dir = 1;
        break;
    case 1:
        VoltBEMF = ADCConvertedValue_2[1];
        break;
    case 2:
        VoltBEMF = ADCConvertedValue_2[2];
        dir = 1;
        break;
    case 3:
        VoltBEMF = ADCConvertedValue_2[0];
        break;
    case 4:
        VoltBEMF = ADCConvertedValue_2[1];
        dir = 1;
        break;
    case 5:
        VoltBEMF = ADCConvertedValue_2[2];
        break;
    default:
        break;
    }

    if (dir == 1)             //PWM-OFF检测BEMF，过零点标志是BEMF电压为0
    {
        if (VoltBEMF > 0)
            return 1;
    }
    else
    {
        if (VoltBEMF <= 26)
            return 1;
    }

    return 0;
}

/**
  * @brief  初始化TIM1的六路输出的初始化状态
  * @param  None
  * @retval None
  */
void MyInitTIM1PWM(void)
{

	//开启预装载寄存器
	__HAL_TIM_ENABLE_OCxPRELOAD(&BLDC_TIMER_NUM,TIM_CHANNEL_1);
	__HAL_TIM_ENABLE_OCxPRELOAD(&BLDC_TIMER_NUM,TIM_CHANNEL_2);
	__HAL_TIM_ENABLE_OCxPRELOAD(&BLDC_TIMER_NUM,TIM_CHANNEL_3);
	__HAL_TIM_ENABLE_OCxPRELOAD(&BLDC_TIMER_NUM,TIM_CHANNEL_4);

//    TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCx_Disable);
//    TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCxN_Disable);
    HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);
    HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);

//    TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Disable);
//    TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Disable);
    HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);
    HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);

//    TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Disable);
//    TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Disable);
    HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);
    HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);

//    TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_4, TIM_CCx_Enable);
    HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_4);

//#if (ADVANCE_TIMER > 0)&(USE_COM_EVENT > 0)
//    TIM_CCPreloadControl(BLDC_TIMER_NUM, ENABLE);     //高级定时器，使用COM事件同步更新寄存器，需要CCxE,CCxNE,OCxM预装载
//#endif

//    TIM_ITConfig(BLDC_TIMER_NUM, TIM_IT_CC4, ENABLE);
//    TIM_ITConfig(BLDC_TIMER_NUM, TIM_IT_Update, ENABLE);
    __HAL_TIM_ENABLE_IT(&BLDC_TIMER_NUM,TIM_IT_CC4);
    __HAL_TIM_ENABLE_IT(&BLDC_TIMER_NUM,TIM_IT_UPDATE);

    /* BLDC_TIMER_NUM counter enable */
//    TIM_Cmd(BLDC_TIMER_NUM, ENABLE);
    __HAL_TIM_ENABLE(&BLDC_TIMER_NUM);

    /* Main Output Enable */
//    TIM_CtrlPWMOutputs(BLDC_TIMER_NUM, ENABLE);
    __HAL_TIM_MOE_ENABLE(&BLDC_TIMER_NUM);
}
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
