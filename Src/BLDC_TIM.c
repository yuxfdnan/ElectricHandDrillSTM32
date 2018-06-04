

#include "BLDC_TIM.h"

#include "BLDC_Motor.h"

#include "stm32f0xx_hal.h"

//#include "bsp.h"


//static void TIM_RCC_Configuration(void)
//{
//
//    //使能定时器时钟
//    if ((BLDC_TIMER_CLK == RCC_APB2Periph_TIM1) || (BLDC_TIMER_CLK == RCC_APB2Periph_TIM8))
//        RCC_APB2PeriphClockCmd(BLDC_TIMER_CLK, ENABLE);
//    else
//        RCC_APB1PeriphClockCmd(BLDC_TIMER_CLK, ENABLE);
//
//    //使能定时器GPIO时钟
//    RCC_APB2PeriphClockCmd(TIMER_GPIO_AH_CLK | TIMER_GPIO_BH_CLK | TIMER_GPIO_CH_CLK | TIMER_GPIO_AL_CLK | \
//                           TIMER_GPIO_BL_CLK | TIMER_GPIO_CL_CLK | RCC_APB2Periph_AFIO, ENABLE);
//
//}



//static void TIM_NVIC_Configuration(void)
//{
//
//    NVIC_InitTypeDef NVIC_InitStructure;
//
//    /* Enable the BLDC_TIMER_NUM Interrupt */
//
//#if USE_COM_EVENT > 0ul
//
//    NVIC_InitStructure.NVIC_IRQChannel = BLDC_TIM_TRG_COM_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//
//    NVIC_Init(&NVIC_InitStructure);
//#endif
//
//    //定时器溢出中断
//    NVIC_InitStructure.NVIC_IRQChannel = BLDC_TIM_UP_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//
//    NVIC_Init(&NVIC_InitStructure);
//
//    //定时器捕获比较中断
//    NVIC_InitStructure.NVIC_IRQChannel = BLDC_TIM_CC_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//
//    NVIC_Init(&NVIC_InitStructure);
//
//}


//static void TIM_GPIO_Configuration(void)
//{
//
//    GPIO_InitTypeDef GPIO_InitStructure;
//
//    /*GPIO PhaseA_H 初始化*/
//    GPIO_InitStructure.GPIO_Pin = TIMER_GPIO_AH_PIN;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
//    GPIO_Init(TIMER_GPIO_AH_PORT, &GPIO_InitStructure);
//
//    /*GPIO PhaseB_H 初始化*/
//    GPIO_InitStructure.GPIO_Pin = TIMER_GPIO_BH_PIN;
//    GPIO_Init(TIMER_GPIO_BH_PORT, &GPIO_InitStructure);
//
//    /*GPIO PhaseC_H 初始化*/
//    GPIO_InitStructure.GPIO_Pin = TIMER_GPIO_CH_PIN;
//    GPIO_Init(TIMER_GPIO_CH_PORT, &GPIO_InitStructure);
//
//    /*GPIO PhaseA_L 初始化*/
//    GPIO_InitStructure.GPIO_Pin = TIMER_GPIO_AL_PIN;
//    GPIO_Init(TIMER_GPIO_AL_PORT, &GPIO_InitStructure);
//
//    /*GPIO PhaseB_L 初始化*/
//    GPIO_InitStructure.GPIO_Pin = TIMER_GPIO_BL_PIN;
//    GPIO_Init(TIMER_GPIO_BL_PORT, &GPIO_InitStructure);
//
//    /*GPIO PhaseC_L 初始化*/
//    GPIO_InitStructure.GPIO_Pin = TIMER_GPIO_CL_PIN;
//    GPIO_Init(TIMER_GPIO_CL_PORT, &GPIO_InitStructure);
//
//    //定时器比较通道IO口重映射
//#if TIMER_GPIO_REMAP > 0ul
//    GPIO_PinRemapConfig(TIMER_GPIO_AFIO, ENABLE);
//#endif
//}

//
//void BLDC_TIM_Configuration(void)
//{
//
//    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//    TIM_OCInitTypeDef  TIM_OCInitStructure;
//    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
//
//    unsigned long temp;
//    unsigned short Period;
//    unsigned short Pluse;
//
//    /* 初始化时钟 */
//    TIM_RCC_Configuration();
//
//    /* 初始化中断 */
//    TIM_NVIC_Configuration();
//
//    /* 初始化IO口 */
//    TIM_GPIO_Configuration();
//
//
//#if TIMER_MODE_UP_DOWN > 0
//    temp = SYS_CLK / BLDC_PWM_Freq / 2;                //中央对齐模式定时器初值减半
//    temp -= 1;
//    Period = (unsigned short)temp;
//
//    MotorA.PWM_Freq = BLDC_PWM_Freq;
//    MotorA.TimerPeriod = Period;
//
//    Pluse = (Period + 1) / 10 - 1; //默认占空比10%
//#else
//    temp = SYS_CLK / BLDC_PWM_Freq;
//    temp -= 1;
//    Period = (unsigned short)temp;
//
//    MotorA.PWM_Freq = BLDC_PWM_Freq;
//    MotorA.TimerPeriod = Period;
//
//    Pluse = (Period + 1) / 10 - 1; //默认占空比20%
//#endif
//
//
//    /* Time Base configuration */
//    TIM_TimeBaseStructure.TIM_Prescaler = 0;
//#if TIMER_MODE_UP_DOWN > 0
//    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned3;
//#else
//    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
//#endif /*TIMER_MODE_UP_DOWN*/
//    TIM_TimeBaseStructure.TIM_Period = Period;
//    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
//
//    TIM_TimeBaseInit(BLDC_TIMER_NUM, &TIM_TimeBaseStructure);
//
//    /* Channel 1, 2,3 and 4 Configuration in PWM mode */
//#if PMOS_ON_HIGH_SIDE > 0
//    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//#else
//    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//#endif /*PMOS_ON_HIGH_SIDE*/
//    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
//    TIM_OCInitStructure.TIM_Pulse = Pluse;
//#if PMOS_ON_HIGH_SIDE > 0
//    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
//#else
//    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//#endif /*PMOS_ON_HIGH_SIDE*/
//    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
//    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
//    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
//
//    TIM_OC1Init(BLDC_TIMER_NUM, &TIM_OCInitStructure);
//
//    TIM_OCInitStructure.TIM_Pulse = Pluse;
//    TIM_OC2Init(BLDC_TIMER_NUM, &TIM_OCInitStructure);
//
//    TIM_OCInitStructure.TIM_Pulse = Pluse;
//    TIM_OC3Init(BLDC_TIMER_NUM, &TIM_OCInitStructure);
//
//    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
//#if PWM_ON > 0ul
//    TIM_OCInitStructure.TIM_Pulse = Pluse - 300;
//#else
//    TIM_OCInitStructure.TIM_Pulse = Period - 100;    //PWM-OFF ADC采样时刻
//#endif
//    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
//
//    TIM_OC4Init(BLDC_TIMER_NUM, &TIM_OCInitStructure);
//
//    /* Automatic Output enable, Break, dead time and lock configuration*/
//    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
//    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
//    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
//    TIM_BDTRInitStructure.TIM_DeadTime = 64;
//    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
//    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
//    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
//
//    TIM_BDTRConfig(BLDC_TIMER_NUM, &TIM_BDTRInitStructure);
//
//    TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCx_Disable);
//    TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCxN_Disable);
//
//    TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Disable);
//    TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Disable);
//
//    TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Disable);
//    TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Disable);
//
//    TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_4, TIM_CCx_Enable);
//
//#if (ADVANCE_TIMER > 0)&(USE_COM_EVENT > 0)
//    TIM_CCPreloadControl(BLDC_TIMER_NUM, ENABLE);     //高级定时器，使用COM事件同步更新寄存器，需要CCxE,CCxNE,OCxM预装载
//#endif
//
//    TIM_ITConfig(BLDC_TIMER_NUM, TIM_IT_CC4, ENABLE);
//    TIM_ITConfig(BLDC_TIMER_NUM, TIM_IT_Update, ENABLE);
//
//    /* BLDC_TIMER_NUM counter enable */
//    TIM_Cmd(BLDC_TIMER_NUM, ENABLE);
//
//    /* Main Output Enable */
//    TIM_CtrlPWMOutputs(BLDC_TIMER_NUM, ENABLE);
//
//}


//void TIM1_TRG_COM_IRQHandler(void)
//{
//    /* Clear TIM1 COM pending bit */
//    TIM_ClearITPendingBit(TIM1, TIM_IT_COM);
//}


///*定时器更新事件中断*/
//void BLDC_TIM_UP_IRQHandler(void)
//{
//
//    TIM_ClearITPendingBit(BLDC_TIMER_NUM, TIM_IT_Update);
//
////     if(MotorA.FlagSwitchStep > 0)
////     {
////         BLDC_Ctrl_Board_LEDOn(LED1);
////
////         BLDC_Ctrl_Board_LEDOff(LED1);
////     }
//}

uint16_t Tim1_4CallbackCnt = 0;
/*定时器捕获比较事件中断*/
/**
  * @brief  Output Compare callback in non blocking mode
  * @param  htim TIM OC handle
  * @retval None
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{

  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the __HAL_TIM_OC_DelayElapsedCallback could be implemented in the user file
   */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
  {
//  TIM_ClearITPendingBit(BLDC_TIMER_NUM, TIM_IT_CC4);

	  //only for test
	  HAL_GPIO_TogglePin(Led2_GPIO_Port,Led2_Pin);

	  MotorA.PWMTicks++;
	  //TODO test
	  Tim1_4CallbackCnt ++;
	  if(Tim1_4CallbackCnt > 1)
	  {
		  Tim1_4CallbackCnt = 0;
		  BLDC_SwitchStep();
	  }

//  BLDC_Ctrl_Board_LEDOn(LED2);

//  ADC_SoftwareStartConvCmd(ADC1, ENABLE);

//  BLDC_Ctrl_Board_LEDOff(LED2);
  }
}

///*定时器捕获比较事件中断*/
//void BLDC_TIM_CC_IRQHandler(void)
//{
//
//    TIM_ClearITPendingBit(BLDC_TIMER_NUM, TIM_IT_CC4);
//    //    TIM_ClearITPendingBit(BLDC_TIMER_NUM, TIM_IT_CC2);
//    //    TIM_ClearITPendingBit(BLDC_TIMER_NUM, TIM_IT_CC3);
//
//    MotorA.PWMTicks++;
//
//    BLDC_Ctrl_Board_LEDOn(LED2);
//
//    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
//
//    BLDC_Ctrl_Board_LEDOff(LED2);
//
//}

