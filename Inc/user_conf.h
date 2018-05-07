#ifndef __USER_CONF_H
#define __USER_CONF_H

//#define SYS_CLK  72000000
//
//#define  BLDC_PWM_Freq   25000
//
//
//#define LEDn                4
//
//#define LED1_GPIO_PORT      GPIOA
//#define LED1_GPIO_PIN       GPIO_Pin_15
//#define LED1_GPIO_CLK       RCC_APB2Periph_GPIOA
//
//#define LED2_GPIO_PORT      GPIOC
//#define LED2_GPIO_PIN       GPIO_Pin_10
//#define LED2_GPIO_CLK       RCC_APB2Periph_GPIOC
//
//#define LED3_GPIO_PORT      GPIOC
//#define LED3_GPIO_PIN       GPIO_Pin_11
//#define LED3_GPIO_CLK       RCC_APB2Periph_GPIOC
//
//#define LED4_GPIO_PORT      GPIOC
//#define LED4_GPIO_PIN       GPIO_Pin_12
//#define LED4_GPIO_CLK       RCC_APB2Periph_GPIOC
//
//
//typedef enum
//{
//    LED1 = 0,
//    LED2 = 1,
//    LED3 = 2,
//    LED4 = 3,
//} Led_TypeDef;
//
//
#define BASIC_DAC           1

#define EXTEND_DAC          1


#if EXTEND_DAC > 0ul

#define    EXT_DAC_TIM           TIM2
#define    EXT_DAC_TIM_CLK       RCC_APB1Periph_TIM2

#define    EXT_DAC_TIM_Freq      72000ul

#define    DAC3_GPIO_PORT        GPIOB
#define    DAC3_GPIO_PIN         GPIO_Pin_10
#define    DAC3_GPIO_CLK         RCC_APB2Periph_GPIOB

#define    DAC4_GPIO_PORT        GPIOB
#define    DAC4_GPIO_PIN         GPIO_Pin_11
#define    DAC4_GPIO_CLK         RCC_APB2Periph_GPIOB

#endif


/*ADC部分*/
//使用ADC通道
#define  ADC_Channel_BEMF_A      ADC_Channel_2
#define  ADC_Channel_BEMF_B      ADC_Channel_1
#define  ADC_Channel_BEMF_C      ADC_Channel_0

#define  ADC_Channel_BusVolt     ADC_Channel_8
#define  ADC_Channel_ISense      ADC_Channel_9
#define  ADC_Channel_SpeedSet    ADC_Channel_10

//备用ADC通道，使用时需飞线
#define  ADC_Channel_ET_A        ADC_Channel_6
#define  ADC_Channel_ET_B        ADC_Channel_7
#define  ADC_Channel_ET_C        ADC_Channel_14
#define  ADC_Channel_ET_D        ADC_Channel_15

//使用ADC管脚
#define  ADC_Bemf_A_GPIO_PORT     GPIOA
#define  ADC_Bemf_A_GPIO_PIN      GPIO_Pin_2
#define  ADC_Bemf_A_GPIO_CLK      RCC_APB2Periph_GPIOA 

#define  ADC_Bemf_B_GPIO_PORT     GPIOA
#define  ADC_Bemf_B_GPIO_PIN      GPIO_Pin_1
#define  ADC_Bemf_B_GPIO_CLK      RCC_APB2Periph_GPIOA 

#define  ADC_Bemf_C_GPIO_PORT     GPIOA
#define  ADC_Bemf_C_GPIO_PIN      GPIO_Pin_0
#define  ADC_Bemf_C_GPIO_CLK      RCC_APB2Periph_GPIOA 

#define  ADC_BusVolt_GPIO_PORT    GPIOB
#define  ADC_BusVolt_GPIO_PIN     GPIO_Pin_0
#define  ADC_BusVolt_GPIO_CLK     RCC_APB2Periph_GPIOB 

#define  ADC_ISense_GPIO_PORT     GPIOB
#define  ADC_ISense_GPIO_PIN      GPIO_Pin_1
#define  ADC_ISense_GPIO_CLK      RCC_APB2Periph_GPIOB 

#define  ADC_SpeedSet_GPIO_PORT   GPIOC
#define  ADC_SpeedSet_GPIO_PIN    GPIO_Pin_0
#define  ADC_SpeedSet_GPIO_CLK    RCC_APB2Periph_GPIOC 

//备用ADC管脚，使用时需飞线
#define  ADC_ET_A_GPIO_PORT       GPIOA
#define  ADC_ET_A_GPIO_PIN        GPIO_Pin_6
#define  ADC_ET_A_GPIO_CLK        RCC_APB2Periph_GPIOA 

#define  ADC_ET_B_GPIO_PORT       GPIOA
#define  ADC_ET_B_GPIO_PIN        GPIO_Pin_7
#define  ADC_ET_B_GPIO_CLK        RCC_APB2Periph_GPIOA 

#define  ADC_ET_C_GPIO_PORT       GPIOC
#define  ADC_ET_C_GPIO_PIN        GPIO_Pin_4
#define  ADC_ET_C_GPIO_CLK        RCC_APB2Periph_GPIOC 

#define  ADC_ET_D_GPIO_PORT       GPIOC
#define  ADC_ET_D_GPIO_PIN        GPIO_Pin_5
#define  ADC_ET_D_GPIO_CLK        RCC_APB2Periph_GPIOC 


//定时器
#define  BLDC_TIMER_NUM         htim1
#define  BLDC_TIMER_CLK         RCC_APB2Periph_TIM1
//定时器输出管脚
#define  TIMER_GPIO_AH_PIN      GPIO_Pin_8  
#define  TIMER_GPIO_AH_PORT     GPIOA
#define  TIMER_GPIO_AH_CLK      RCC_APB2Periph_GPIOA

#define  TIMER_GPIO_BH_PIN      GPIO_Pin_9
#define  TIMER_GPIO_BH_PORT     GPIOA
#define  TIMER_GPIO_BH_CLK      RCC_APB2Periph_GPIOA

#define  TIMER_GPIO_CH_PIN      GPIO_Pin_10
#define  TIMER_GPIO_CH_PORT     GPIOA
#define  TIMER_GPIO_CH_CLK      RCC_APB2Periph_GPIOA

#define  TIMER_GPIO_AL_PIN      GPIO_Pin_13
#define  TIMER_GPIO_AL_PORT     GPIOB
#define  TIMER_GPIO_AL_CLK      RCC_APB2Periph_GPIOB

#define  TIMER_GPIO_BL_PIN      GPIO_Pin_14
#define  TIMER_GPIO_BL_PORT     GPIOB
#define  TIMER_GPIO_BL_CLK      RCC_APB2Periph_GPIOB

#define  TIMER_GPIO_CL_PIN      GPIO_Pin_15
#define  TIMER_GPIO_CL_PORT     GPIOB
#define  TIMER_GPIO_CL_CLK      RCC_APB2Periph_GPIOB

#define  TIMER_GPIO_REMAP       0ul    //定时器输出管脚是否需要重映射，0表示不需要，非零表示需要重映射
#define  TIMER_GPIO_AFIO        GPIO_PartialRemap_TIM1 

#define  PWM_ON                 0ul   //ADC同步采样反电动势策略，0表示PWM-OFF采样，非零表示PWM-ON采样

/*全NMOS或上桥臂使用PMOS*/
#define  PMOS_ON_HIGH_SIDE      1ul   //使用上桥臂MOSFET类型，0表示上桥臂使用N管，非零表示上桥臂使用P管

/*是否使用高级定时器*/
#define  ADVANCE_TIMER          1ul   //是否使用高级定时器

/*中央对齐模式*/
#define  TIMER_MODE_UP_DOWN     0ul   //PWM模式，是否使用中央对齐PWM 


#if ADVANCE_TIMER > 0u
/*是否使用COM事件*/
#define  USE_COM_EVENT          0ul   //COM事件换相
#endif



/*如果使用高级定时器，一下项需要定义*/

#if ADVANCE_TIMER > 0u 
#define  BLDC_TIM_BRK_IRQn         TIM1_BRK_IRQn
#define  BLDC_TIM_UP_IRQn          TIM1_UP_IRQn
#define  BLDC_TIM_TRG_COM_IRQn     TIM1_TRG_COM_IRQn
#define  BLDC_TIM_CC_IRQn          TIM1_CC_IRQn
#else
#define  BLDC_TIM_IRQn             TIM2_IRQn 
#endif


#if ADVANCE_TIMER > 0u 
#define  BLDC_TIM_BRK_IRQHandler       TIM1_BRK_IRQHandler
#define  BLDC_TIM_UP_IRQHandler        TIM1_UP_IRQHandler
#define  BLDC_TIM_TRG_COM_IRQHandler   TIM1_TRG_COM_IRQHandler
#define  BLDC_TIM_CC_IRQHandler        TIM1_CC_IRQHandler
#else
#define  BLDC_TIM_IRQHandler           TIM2_IRQHandler 
#endif


#define  SYS_TIMER_NUM             TIM6
#define  SYS_TIMER_CLK             RCC_APB1Periph_TIM6

#define  SYS_TIMER_IRQn            TIM6_IRQn
#define  SYS_TIMER_IRQHandler      TIM6_IRQHandler



#endif /*__USER_CONF_H*/

