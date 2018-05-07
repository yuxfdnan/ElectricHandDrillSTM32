

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


/*ֹͣPWM���*/
void BLDC_Stop(void)
{
//    TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCx_Disable);
//    TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Disable);
//    TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Disable);
    HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0 ���Ź�
    HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//0 ���Ź�
    HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);//0 ���Ź�


//    TIM_OC1NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_High);
//    TIM_OC2NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_High);
//    TIM_OC3NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_High);
    __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_1,TIM_OCNPOLARITY_HIGH);//��Ч��
    __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_2,TIM_OCNPOLARITY_HIGH);//��Ч��
    __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_3,TIM_OCNPOLARITY_HIGH);//��Ч��

//    TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCxN_Disable);
//    TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Disable);
//    TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Disable);
    HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0 ���Ź�
    HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//0 ���Ź�
    HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);//0 ���Ź�

    MotorA.PWMTicks = 0;
}


void BLDC_SetStep(unsigned char step)
{
    MotorA.Step = step;
}


/*����*/
void BLDC_SwitchStep(void)
{

    MotorA.Step = (MotorA.Step + 1) % 6;

    MotorA.PWMTicksPre = MotorA.PWMTicks;

    MotorA.FlagBEMF = 0;
    MotorA.PWMTicks = 0;

    switch (MotorA.Step)
    {
    case 0 ://BC��ͨ
        /*  PhaseA configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCx_Disable);   // 0
//        TIM_OC1NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_High);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCxN_Disable); // 0
        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0 ���Ź�
        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_1,TIM_OCNPOLARITY_HIGH);//��Ч��
        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0 ���Ź�


        /*  PhaseB configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Enable);    // 1 ���ſ�
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Enable);  //���ſ�,ͬ������
        HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_2);// 1 ���ſ�
        HAL_TIMEx_OCN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//���ſ�,ͬ������

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Disable);   // 0 ���Ź�
//        TIM_OC3NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_Low);// 1  ����,һֱ��
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Disable); // 1 ���ų���,һֱ��
        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 0 ���Ź�
        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_3,TIM_OCNPOLARITY_LOW);// 1  ����,һֱ��
        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 1 ���ų���,һֱ��
        break;
    case 1://AC
        /*  PhaseB configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Disable);   // 0
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Disable); // 0
        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);// 0 ���Ź�
        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//0 ���Ź�

        /*  PhaseA configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCx_Enable);    // 1
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCxN_Enable);  //ͬ������
        HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_1);// 1 ���ſ�
        HAL_TIMEx_OCN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//���ſ�,ͬ������

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Disable);   // 0
//        TIM_OC3NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_Low);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Disable); // 1
        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 0
        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_3,TIM_OCNPOLARITY_LOW);//һֱ��
        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);//0 һֱ��
        break;
    case 2://AB
        /*  PhaseA configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCx_Enable);    // 1
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCxN_Enable);  //ͬ������
        HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_1);// 1 ���ſ�
        HAL_TIMEx_OCN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//���ſ�,ͬ������

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Disable);   // 0
//        TIM_OC3NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_High);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Disable); // 0
        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 0
        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_3,TIM_OCNPOLARITY_HIGH);//��Ч��
        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);//0

        /*  PhaseB configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Disable);   // 0
//        TIM_OC2NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_Low);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Disable); // 1
        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);// 1 ���ſ�
        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_2,TIM_OCNPOLARITY_LOW);//0 һֱ��
        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//0 һֱ��
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
        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_2,TIM_OCNPOLARITY_LOW);//0 һֱ��
        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//0 һֱ��

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Enable);    // 1
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Enable);  //ͬ������
        HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 1 ���ſ�
        HAL_TIMEx_OCN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_3);//���ſ�,ͬ������
        break;
    case 4://CA
        /*  PhaseB configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Disable);   // 0
//        TIM_OC2NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_High);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Disable); // 0
        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);// 0
        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_2,TIM_OCNPOLARITY_HIGH);//��Ч��
        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//0

        /*  PhaseA configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCx_Disable);   // 0
//        TIM_OC1NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_Low);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCxN_Disable); // 1
        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);// 1 ���ſ�
        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_1,TIM_OCNPOLARITY_LOW);//0 һֱ��
        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0 һֱ��

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Enable);    // 1
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Enable);  //ͬ������
        HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 1 ���ſ�
        HAL_TIMEx_OCN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_3);//���ſ�,ͬ������
        break;
    case 5://BA
        /*  PhaseA configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCx_Disable);   // 0
//        TIM_OC1NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_Low);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCxN_Disable); // 1
        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);// 0
        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_1,TIM_OCNPOLARITY_LOW);//0 һֱ��
        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0 һֱ��

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Disable);   // 0
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Disable); // 0
        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 0
        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);//0

        /*  PhaseB configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Enable);    // 1
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Enable);  //ͬ������
        HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_2);// 1 ���ſ�
        HAL_TIMEx_OCN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//���ſ�,ͬ������
        break;
    default:
        MotorA.Step = 0;
    }

}













