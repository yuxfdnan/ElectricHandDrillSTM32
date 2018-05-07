

#include "BLDC_Motor.h"

//#include "main.h"
#include "stm32f0xx_hal.h"

#include "user_conf.h"


MotorParam_TypeDef MotorA;
extern TIM_HandleTypeDef htim1;


void BLDC_Init(void)
{
    MotorA.PWMTicks = 0;

    MotorA.Step = 0;
    MotorA.State = 0;

    MotorA.FlagSwitchStep = 0;
    MotorA.FlagBEMF = 0;
}


/*停止PWM输出*/
void BLDC_Stop(void)
{
//    TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCx_Disable);
//    TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Disable);
//    TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Disable);
    HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0 上桥关
    HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//0 上桥关
    HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);//0 上桥关


//    TIM_OC1NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_High);
//    TIM_OC2NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_High);
//    TIM_OC3NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_High);
    __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_1,TIM_OCNPOLARITY_HIGH);//有效打开
    __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_2,TIM_OCNPOLARITY_HIGH);//有效打开
    __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_3,TIM_OCNPOLARITY_HIGH);//有效打开

//    TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCxN_Disable);
//    TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Disable);
//    TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Disable);
    HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0 下桥关
    HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//0 下桥关
    HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);//0 下桥关

    MotorA.PWMTicks = 0;
}


void BLDC_SetStep(unsigned char step)
{
    MotorA.Step = step;
}


/*换向*/
void BLDC_SwitchStep(void)
{

    MotorA.Step = (MotorA.Step + 1) % 6;

    MotorA.PWMTicksPre = MotorA.PWMTicks;

    MotorA.FlagBEMF = 0;
    MotorA.PWMTicks = 0;

    switch (MotorA.Step)
    {
    case 0 ://BC导通
        /*  PhaseA configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCx_Disable);   // 0
//        TIM_OC1NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_High);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCxN_Disable); // 0
        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0 上桥关
        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_1,TIM_OCNPOLARITY_HIGH);//有效打开
        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0 下桥关


        /*  PhaseB configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Enable);    // 1 上桥开
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Enable);  //下桥开,同步整流
        HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_2);// 1 上桥开
        HAL_TIMEx_OCN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//下桥开,同步整流

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Disable);   // 0 上桥关
//        TIM_OC3NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_Low);// 1  常开,一直打开
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Disable); // 1 下桥常开,一直打开
        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 0 上桥关
        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_3,TIM_OCNPOLARITY_LOW);// 1  常开,一直打开
        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 1 下桥常开,一直打开
        break;
    case 1://AC
        /*  PhaseB configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Disable);   // 0
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Disable); // 0
        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);// 0 上桥关
        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//0 下桥关

        /*  PhaseA configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCx_Enable);    // 1
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCxN_Enable);  //同步整流
        HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_1);// 1 上桥开
        HAL_TIMEx_OCN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//下桥开,同步整流

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Disable);   // 0
//        TIM_OC3NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_Low);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Disable); // 1
        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 0
        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_3,TIM_OCNPOLARITY_LOW);//一直打开
        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);//0 一直打开
        break;
    case 2://AB
        /*  PhaseA configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCx_Enable);    // 1
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCxN_Enable);  //同步整流
        HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_1);// 1 上桥开
        HAL_TIMEx_OCN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//下桥开,同步整流

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Disable);   // 0
//        TIM_OC3NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_High);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Disable); // 0
        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 0
        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_3,TIM_OCNPOLARITY_HIGH);//有效打开
        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);//0

        /*  PhaseB configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Disable);   // 0
//        TIM_OC2NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_Low);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Disable); // 1
        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);// 1 上桥开
        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_2,TIM_OCNPOLARITY_LOW);//0 一直打开
        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//0 一直打开
        break;
    case 3://CB
        /*  PhaseA configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCx_Disable);   // 0
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCxN_Disable); // 0
        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0
        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0

        /*  PhaseB configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Disable);   // 0
//        TIM_OC2NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_Low);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Disable); // 1
        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);// 0
        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_2,TIM_OCNPOLARITY_LOW);//0 一直打开
        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//0 一直打开

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Enable);    // 1
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Enable);  //同步整流
        HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 1 上桥开
        HAL_TIMEx_OCN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_3);//下桥开,同步整流
        break;
    case 4://CA
        /*  PhaseB configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Disable);   // 0
//        TIM_OC2NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_High);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Disable); // 0
        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);// 0
        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_2,TIM_OCNPOLARITY_HIGH);//有效打开
        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//0

        /*  PhaseA configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCx_Disable);   // 0
//        TIM_OC1NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_Low);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCxN_Disable); // 1
        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);// 1 上桥开
        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_1,TIM_OCNPOLARITY_LOW);//0 一直打开
        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0 一直打开

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Enable);    // 1
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Enable);  //同步整流
        HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 1 上桥开
        HAL_TIMEx_OCN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_3);//下桥开,同步整流
        break;
    case 5://BA
        /*  PhaseA configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCx_Disable);   // 0
//        TIM_OC1NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_Low);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCxN_Disable); // 1
        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);// 0
        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_1,TIM_OCNPOLARITY_LOW);//0 一直打开
        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0 一直打开

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Disable);   // 0
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Disable); // 0
        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 0
        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);//0

        /*  PhaseB configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Enable);    // 1
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Enable);  //同步整流
        HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_2);// 1 上桥开
        HAL_TIMEx_OCN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//下桥开,同步整流
        break;
    default:
        MotorA.Step = 0;
    }

}













