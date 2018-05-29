

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

//    MotorA.Step = (MotorA.Step + 1) % 6;

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
//    	HAL_TIM_PWM_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0 上桥关
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_1,TIM_OCNPOLARITY_HIGH);//有效打开
//        HAL_TIMEx_PWMN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0 下桥关
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_1,CHANNEL_STOP,CHANNEL_STOP,TIM_MOD_PWM);


        /*  PhaseB configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Enable);    // 1 上桥开
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Enable);  //下桥开,同步整流
//        HAL_TIM_PWM_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_2);// 1 上桥开
//        HAL_TIMEx_PWMN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//下桥开,同步整流
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_2,CHANNEL_START,CHANNEL_START,TIM_MOD_PWM);

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Disable);   // 0 上桥关
//        TIM_OC3NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_Low);// 1  常开,一直打开
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Disable); // 1 下桥常开,一直打开
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 0 上桥关
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_3,TIM_OCNPOLARITY_LOW);// 1  常开,一直打开
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 1 下桥常开,一直打开
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_START,TIM_MOD_HIGH);
        break;
    case 1://AC
        /*  PhaseB configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Disable);   // 0
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Disable); // 0
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);// 0 上桥关
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//0 下桥关
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_STOP,TIM_MOD_PWM);

        /*  PhaseA configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCx_Enable);    // 1
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCxN_Enable);  //同步整流
//        HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_1);// 1 上桥开
//        HAL_TIMEx_OCN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//下桥开,同步整流
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_START,CHANNEL_START,TIM_MOD_PWM);

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Disable);   // 0
//        TIM_OC3NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_Low);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Disable); // 1
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 0
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_3,TIM_OCNPOLARITY_LOW);//一直打开
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);//0 一直打开
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_START,TIM_MOD_HIGH);
        break;
    case 2://AB
        /*  PhaseA configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCx_Enable);    // 1
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCxN_Enable);  //同步整流
        HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_1);// 1 上桥开
        HAL_TIMEx_OCN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//下桥开,同步整流
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_START,TIM_MOD_PWM);

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Disable);   // 0
//        TIM_OC3NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_High);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Disable); // 0
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 0
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_3,TIM_OCNPOLARITY_HIGH);//有效打开
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);//0
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_STOP,TIM_MOD_PWM);

        /*  PhaseB configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Disable);   // 0
//        TIM_OC2NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_Low);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Disable); // 1
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);// 1 上桥关
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_2,TIM_OCNPOLARITY_LOW);//0 一直打开
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//0 一直打开
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_START,TIM_MOD_HIGH);

        break;
    case 3://CB
        /*  PhaseA configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCx_Disable);   // 0
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCxN_Disable); // 0
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_STOP,TIM_MOD_PWM);

        /*  PhaseB configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Disable);   // 0
//        TIM_OC2NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_Low);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Disable); // 1
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);// 0
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_2,TIM_OCNPOLARITY_LOW);//0 一直打开
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//0 一直打开
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_START,TIM_MOD_HIGH);

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Enable);    // 1
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Enable);  //同步整流
//        HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 1 上桥开
//        HAL_TIMEx_OCN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_3);//下桥开,同步整流
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_START,CHANNEL_START,TIM_MOD_PWM);
        break;
    case 4://CA
        /*  PhaseB configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Disable);   // 0
//        TIM_OC2NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_High);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Disable); // 0
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);// 0
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_2,TIM_OCNPOLARITY_HIGH);//有效打开
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//0
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_STOP,TIM_MOD_PWM);

        /*  PhaseA configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCx_Disable);   // 0
//        TIM_OC1NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_Low);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCxN_Disable); // 1
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);// 1 上桥关
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_1,TIM_OCNPOLARITY_LOW);//0 一直打开
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0 一直打开
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_START,TIM_MOD_HIGH);

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Enable);    // 1
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Enable);  //同步整流
//        HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 1 上桥开
//        HAL_TIMEx_OCN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_3);//下桥开,同步整流
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_START,CHANNEL_START,TIM_MOD_PWM);
        break;
    case 5://BA
        /*  PhaseA configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCx_Disable);   // 0
//        TIM_OC1NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_Low);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCxN_Disable); // 1
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);// 0
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_1,TIM_OCNPOLARITY_LOW);//0 一直打开
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0 一直打开
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_START,TIM_MOD_HIGH);

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Disable);   // 0
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Disable); // 0
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 0
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);//0
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_STOP,TIM_MOD_PWM);

        /*  PhaseB configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Enable);    // 1
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Enable);  //同步整流
//        HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_2);// 1 上桥开
//        HAL_TIMEx_OCN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//下桥开,同步整流
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_START,CHANNEL_START,TIM_MOD_PWM);
        break;
    default:
        MotorA.Step = 0;
    }

}

/*换向*/
void BLDC_SwitchStepTest(void)
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
//    	HAL_TIM_PWM_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0 上桥关
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_1,TIM_OCNPOLARITY_HIGH);//有效打开
//        HAL_TIMEx_PWMN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0 下桥关
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_1,CHANNEL_STOP,CHANNEL_STOP,TIM_MOD_PWM);


        /*  PhaseB configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Enable);    // 1 上桥开
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Enable);  //下桥开,同步整流
//        HAL_TIM_PWM_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_2);// 1 上桥开
//        HAL_TIMEx_PWMN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//下桥开,同步整流
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_2,CHANNEL_START,CHANNEL_START,TIM_MOD_PWM);

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Disable);   // 0 上桥关
//        TIM_OC3NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_Low);// 1  常开,一直打开
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Disable); // 1 下桥常开,一直打开
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 0 上桥关
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_3,TIM_OCNPOLARITY_LOW);// 1  常开,一直打开
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 1 下桥常开,一直打开
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_START,TIM_MOD_HIGH);
        break;
    case 1://AC
        /*  PhaseB configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Disable);   // 0
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Disable); // 0
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);// 0 上桥关
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//0 下桥关
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_STOP,TIM_MOD_PWM);

        /*  PhaseA configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCx_Enable);    // 1
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCxN_Enable);  //同步整流
//        HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_1);// 1 上桥开
//        HAL_TIMEx_OCN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//下桥开,同步整流
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_START,CHANNEL_START,TIM_MOD_PWM);

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Disable);   // 0
//        TIM_OC3NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_Low);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Disable); // 1
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 0
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_3,TIM_OCNPOLARITY_LOW);//一直打开
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);//0 一直打开
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_START,TIM_MOD_HIGH);
        break;
    case 2://AB
        /*  PhaseA configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCx_Enable);    // 1
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCxN_Enable);  //同步整流
        HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_1);// 1 上桥开
        HAL_TIMEx_OCN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//下桥开,同步整流
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_START,TIM_MOD_PWM);

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Disable);   // 0
//        TIM_OC3NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_High);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Disable); // 0
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 0
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_3,TIM_OCNPOLARITY_HIGH);//有效打开
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);//0
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_STOP,TIM_MOD_PWM);

        /*  PhaseB configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Disable);   // 0
//        TIM_OC2NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_Low);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Disable); // 1
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);// 1 上桥关
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_2,TIM_OCNPOLARITY_LOW);//0 一直打开
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//0 一直打开
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_START,TIM_MOD_HIGH);

        break;
    case 3://CB
        /*  PhaseA configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCx_Disable);   // 0
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCxN_Disable); // 0
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_STOP,TIM_MOD_PWM);

        /*  PhaseB configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Disable);   // 0
//        TIM_OC2NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_Low);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Disable); // 1
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);// 0
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_2,TIM_OCNPOLARITY_LOW);//0 一直打开
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//0 一直打开
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_START,TIM_MOD_HIGH);

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Enable);    // 1
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Enable);  //同步整流
//        HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 1 上桥开
//        HAL_TIMEx_OCN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_3);//下桥开,同步整流
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_START,CHANNEL_START,TIM_MOD_PWM);
        break;
    case 4://CA
        /*  PhaseB configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Disable);   // 0
//        TIM_OC2NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_High);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Disable); // 0
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);// 0
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_2,TIM_OCNPOLARITY_HIGH);//有效打开
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//0
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_STOP,TIM_MOD_PWM);

        /*  PhaseA configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCx_Disable);   // 0
//        TIM_OC1NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_Low);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCxN_Disable); // 1
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);// 1 上桥关
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_1,TIM_OCNPOLARITY_LOW);//0 一直打开
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0 一直打开
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_START,TIM_MOD_HIGH);

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Enable);    // 1
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Enable);  //同步整流
//        HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 1 上桥开
//        HAL_TIMEx_OCN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_3);//下桥开,同步整流
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_START,CHANNEL_START,TIM_MOD_PWM);
        break;
    case 5://BA
        /*  PhaseA configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCx_Disable);   // 0
//        TIM_OC1NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_Low);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCxN_Disable); // 1
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);// 0
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_1,TIM_OCNPOLARITY_LOW);//0 一直打开
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0 一直打开
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_START,TIM_MOD_HIGH);

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Disable);   // 0
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Disable); // 0
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 0
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);//0
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_STOP,TIM_MOD_PWM);

        /*  PhaseB configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Enable);    // 1
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Enable);  //同步整流
//        HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_2);// 1 上桥开
//        HAL_TIMEx_OCN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//下桥开,同步整流
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_START,CHANNEL_START,TIM_MOD_PWM);
        break;
    default:
        MotorA.Step = 0;
    }

}

/**
  * @brief  设置每个通道的状态.
  * @param  htim TIM handle
  * @param  Channel TIM Channel to be enabled
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected 不用
  * @param  OC CHANNEL_START:开通道,CHANNEL_STOP:关通道
  * @param  OCN CHANNEL_START:开通道,CHANNEL_STOP:关通道
  * @param  OCxM TIM_MOD
  * 		This parameter can be one of the following values:
  * 		  @arg TIM_MOD_PWM: PWM模式
  * 		  @arg TIM_MOD_HIGH: 强制高
  * 		  @arg TIM_MOD_LOW: 强制低
  * @retval HAL status
  */
void SetChannelState(TIM_HandleTypeDef *htim, uint32_t Channel, uint8_t OC, uint8_t OCN, uint8_t OCxM)
{
	uint32_t tmpccmrx = 0U;
	uint32_t tmpCCxE = 0U;
	uint32_t tmpCCxNE = 0U;
	if(Channel == TIM_CHANNEL_1)
	{
		tmpCCxE = TIM_CCER_CC1E;
		tmpCCxNE = TIM_CCER_CC1NE;
	}else if(Channel == TIM_CHANNEL_2)
	{
		tmpCCxE = TIM_CCER_CC2E;
		tmpCCxNE = TIM_CCER_CC2NE;
	}else if(Channel == TIM_CHANNEL_3)
	{
		tmpCCxE = TIM_CCER_CC3E;
		tmpCCxNE = TIM_CCER_CC3NE;
	}
	if(OC == CHANNEL_START)
	{
//		HAL_TIM_OC_Start(htim,Channel);
		if(htim->Instance->CCER & tmpCCxE)//已开启
		{

		}
		else
		{//未开启
			HAL_TIM_PWM_Start(htim,Channel);
		}
	}
	else if(OC == CHANNEL_STOP)
	{
//		HAL_TIM_OC_Stop(htim,Channel);
		HAL_TIM_PWM_Stop(htim,Channel);
	}

	if(OCN == CHANNEL_START)
	{
		if(htim->Instance->CCER & tmpCCxNE)//已开启
		{

		}
		else
		{//未开启
//		HAL_TIMEx_OCN_Start(htim,Channel);
			HAL_TIMEx_PWMN_Start(htim,Channel);
		}
	}
	else if(OCN == CHANNEL_STOP)
	{
//		HAL_TIMEx_OCN_Stop(htim,Channel);
		HAL_TIMEx_PWMN_Stop(htim,Channel);
	}

	if(Channel == TIM_CHANNEL_1)
	{
		/* Get the TIMx CCMR1 register value */
		tmpccmrx = htim->Instance->CCMR1;
		tmpccmrx &= ~TIM_CCMR1_OC1M;//清零
		if(OCxM == TIM_MOD_PWM)
			tmpccmrx |= (TIM_OCMODE_PWM1);//置位110：PWM 模式1 － 在向上计数时
		else if(OCxM == TIM_MOD_HIGH)
			tmpccmrx |= (TIM_OCMODE_FORCED_ACTIVE);//置位101:强制为有效电平.强制OC1REF 为高。
		else if(OCxM == TIM_MOD_LOW)
			tmpccmrx |= (TIM_OCMODE_FORCED_INACTIVE);//置位100:强制为无效电平.强制OC1REF 为低。
		/* Write to TIMx CCMR1 */
		htim->Instance->CCMR1 = tmpccmrx;
	}
	else if(Channel == TIM_CHANNEL_2)
	{
		/* Get the TIMx CCMR1 register value */
		tmpccmrx = htim->Instance->CCMR1;
		tmpccmrx &= ~TIM_CCMR1_OC2M;//清零
		if(OCxM == TIM_MOD_PWM)
			tmpccmrx |= (TIM_OCMODE_PWM1 << 8U);//置位110：PWM 模式1 － 在向上计数时
		else if(OCxM == TIM_MOD_HIGH)
			tmpccmrx |= (TIM_OCMODE_FORCED_ACTIVE << 8U);//置位101:强制为有效电平.强制OC1REF 为高。
		else if(OCxM == TIM_MOD_LOW)
			tmpccmrx |= (TIM_OCMODE_FORCED_INACTIVE << 8U);//置位100:强制为无效电平.强制OC1REF 为低。
		/* Write to TIMx CCMR1 */
		htim->Instance->CCMR1 = tmpccmrx;
	}
	else if(Channel == TIM_CHANNEL_3)
	{
		/* Get the TIMx CCMR1 register value */
		tmpccmrx = htim->Instance->CCMR2;
		tmpccmrx &= ~TIM_CCMR2_OC3M;//清零
		if(OCxM == TIM_MOD_PWM)
			tmpccmrx |= (TIM_OCMODE_PWM1);//置位110：PWM 模式1 － 在向上计数时
		else if(OCxM == TIM_MOD_HIGH)
			tmpccmrx |= (TIM_OCMODE_FORCED_ACTIVE);//置位101:强制为有效电平.强制OC1REF 为高。
		else if(OCxM == TIM_MOD_LOW)
			tmpccmrx |= (TIM_OCMODE_FORCED_INACTIVE);//置位100:强制为无效电平.强制OC1REF 为低。
		/* Write to TIMx CCMR2 */
		htim->Instance->CCMR2 = tmpccmrx;
	}

}










