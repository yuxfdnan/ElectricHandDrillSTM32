

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

//    MotorA.Step = (MotorA.Step + 1) % 6;

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
//    	HAL_TIM_PWM_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0 ���Ź�
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_1,TIM_OCNPOLARITY_HIGH);//��Ч��
//        HAL_TIMEx_PWMN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0 ���Ź�
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_1,CHANNEL_STOP,CHANNEL_STOP,TIM_MOD_PWM);


        /*  PhaseB configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Enable);    // 1 ���ſ�
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Enable);  //���ſ�,ͬ������
//        HAL_TIM_PWM_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_2);// 1 ���ſ�
//        HAL_TIMEx_PWMN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//���ſ�,ͬ������
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_2,CHANNEL_START,CHANNEL_START,TIM_MOD_PWM);

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Disable);   // 0 ���Ź�
//        TIM_OC3NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_Low);// 1  ����,һֱ��
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Disable); // 1 ���ų���,һֱ��
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 0 ���Ź�
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_3,TIM_OCNPOLARITY_LOW);// 1  ����,һֱ��
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 1 ���ų���,һֱ��
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_START,TIM_MOD_HIGH);
        break;
    case 1://AC
        /*  PhaseB configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Disable);   // 0
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Disable); // 0
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);// 0 ���Ź�
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//0 ���Ź�
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_STOP,TIM_MOD_PWM);

        /*  PhaseA configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCx_Enable);    // 1
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCxN_Enable);  //ͬ������
//        HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_1);// 1 ���ſ�
//        HAL_TIMEx_OCN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//���ſ�,ͬ������
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_START,CHANNEL_START,TIM_MOD_PWM);

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Disable);   // 0
//        TIM_OC3NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_Low);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Disable); // 1
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 0
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_3,TIM_OCNPOLARITY_LOW);//һֱ��
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);//0 һֱ��
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_START,TIM_MOD_HIGH);
        break;
    case 2://AB
        /*  PhaseA configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCx_Enable);    // 1
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCxN_Enable);  //ͬ������
        HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_1);// 1 ���ſ�
        HAL_TIMEx_OCN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//���ſ�,ͬ������
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_START,TIM_MOD_PWM);

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Disable);   // 0
//        TIM_OC3NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_High);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Disable); // 0
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 0
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_3,TIM_OCNPOLARITY_HIGH);//��Ч��
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);//0
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_STOP,TIM_MOD_PWM);

        /*  PhaseB configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Disable);   // 0
//        TIM_OC2NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_Low);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Disable); // 1
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);// 1 ���Ź�
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_2,TIM_OCNPOLARITY_LOW);//0 һֱ��
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//0 һֱ��
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
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_2,TIM_OCNPOLARITY_LOW);//0 һֱ��
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//0 һֱ��
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_START,TIM_MOD_HIGH);

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Enable);    // 1
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Enable);  //ͬ������
//        HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 1 ���ſ�
//        HAL_TIMEx_OCN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_3);//���ſ�,ͬ������
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_START,CHANNEL_START,TIM_MOD_PWM);
        break;
    case 4://CA
        /*  PhaseB configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Disable);   // 0
//        TIM_OC2NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_High);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Disable); // 0
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);// 0
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_2,TIM_OCNPOLARITY_HIGH);//��Ч��
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//0
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_STOP,TIM_MOD_PWM);

        /*  PhaseA configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCx_Disable);   // 0
//        TIM_OC1NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_Low);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCxN_Disable); // 1
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);// 1 ���Ź�
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_1,TIM_OCNPOLARITY_LOW);//0 һֱ��
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0 һֱ��
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_START,TIM_MOD_HIGH);

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Enable);    // 1
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Enable);  //ͬ������
//        HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 1 ���ſ�
//        HAL_TIMEx_OCN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_3);//���ſ�,ͬ������
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_START,CHANNEL_START,TIM_MOD_PWM);
        break;
    case 5://BA
        /*  PhaseA configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCx_Disable);   // 0
//        TIM_OC1NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_Low);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCxN_Disable); // 1
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);// 0
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_1,TIM_OCNPOLARITY_LOW);//0 һֱ��
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0 һֱ��
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_START,TIM_MOD_HIGH);

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Disable);   // 0
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Disable); // 0
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 0
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);//0
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_STOP,TIM_MOD_PWM);

        /*  PhaseB configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Enable);    // 1
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Enable);  //ͬ������
//        HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_2);// 1 ���ſ�
//        HAL_TIMEx_OCN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//���ſ�,ͬ������
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_START,CHANNEL_START,TIM_MOD_PWM);
        break;
    default:
        MotorA.Step = 0;
    }

}

/*����*/
void BLDC_SwitchStepTest(void)
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
//    	HAL_TIM_PWM_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0 ���Ź�
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_1,TIM_OCNPOLARITY_HIGH);//��Ч��
//        HAL_TIMEx_PWMN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0 ���Ź�
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_1,CHANNEL_STOP,CHANNEL_STOP,TIM_MOD_PWM);


        /*  PhaseB configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Enable);    // 1 ���ſ�
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Enable);  //���ſ�,ͬ������
//        HAL_TIM_PWM_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_2);// 1 ���ſ�
//        HAL_TIMEx_PWMN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//���ſ�,ͬ������
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_2,CHANNEL_START,CHANNEL_START,TIM_MOD_PWM);

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Disable);   // 0 ���Ź�
//        TIM_OC3NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_Low);// 1  ����,һֱ��
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Disable); // 1 ���ų���,һֱ��
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 0 ���Ź�
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_3,TIM_OCNPOLARITY_LOW);// 1  ����,һֱ��
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 1 ���ų���,һֱ��
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_START,TIM_MOD_HIGH);
        break;
    case 1://AC
        /*  PhaseB configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Disable);   // 0
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Disable); // 0
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);// 0 ���Ź�
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//0 ���Ź�
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_STOP,TIM_MOD_PWM);

        /*  PhaseA configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCx_Enable);    // 1
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCxN_Enable);  //ͬ������
//        HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_1);// 1 ���ſ�
//        HAL_TIMEx_OCN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//���ſ�,ͬ������
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_START,CHANNEL_START,TIM_MOD_PWM);

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Disable);   // 0
//        TIM_OC3NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_Low);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Disable); // 1
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 0
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_3,TIM_OCNPOLARITY_LOW);//һֱ��
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);//0 һֱ��
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_START,TIM_MOD_HIGH);
        break;
    case 2://AB
        /*  PhaseA configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCx_Enable);    // 1
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCxN_Enable);  //ͬ������
        HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_1);// 1 ���ſ�
        HAL_TIMEx_OCN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//���ſ�,ͬ������
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_START,TIM_MOD_PWM);

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Disable);   // 0
//        TIM_OC3NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_High);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Disable); // 0
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 0
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_3,TIM_OCNPOLARITY_HIGH);//��Ч��
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);//0
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_STOP,TIM_MOD_PWM);

        /*  PhaseB configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Disable);   // 0
//        TIM_OC2NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_Low);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Disable); // 1
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);// 1 ���Ź�
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_2,TIM_OCNPOLARITY_LOW);//0 һֱ��
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//0 һֱ��
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
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_2,TIM_OCNPOLARITY_LOW);//0 һֱ��
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//0 һֱ��
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_START,TIM_MOD_HIGH);

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Enable);    // 1
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Enable);  //ͬ������
//        HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 1 ���ſ�
//        HAL_TIMEx_OCN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_3);//���ſ�,ͬ������
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_START,CHANNEL_START,TIM_MOD_PWM);
        break;
    case 4://CA
        /*  PhaseB configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Disable);   // 0
//        TIM_OC2NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_High);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Disable); // 0
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);// 0
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_2,TIM_OCNPOLARITY_HIGH);//��Ч��
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//0
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_STOP,TIM_MOD_PWM);

        /*  PhaseA configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCx_Disable);   // 0
//        TIM_OC1NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_Low);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCxN_Disable); // 1
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);// 1 ���Ź�
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_1,TIM_OCNPOLARITY_LOW);//0 һֱ��
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0 һֱ��
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_START,TIM_MOD_HIGH);

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Enable);    // 1
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Enable);  //ͬ������
//        HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 1 ���ſ�
//        HAL_TIMEx_OCN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_3);//���ſ�,ͬ������
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_START,CHANNEL_START,TIM_MOD_PWM);
        break;
    case 5://BA
        /*  PhaseA configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCx_Disable);   // 0
//        TIM_OC1NPolarityConfig(BLDC_TIMER_NUM, TIM_OCNPolarity_Low);
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_1, TIM_CCxN_Disable); // 1
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);// 0
//        __HAL_TIM_SET_OC_POLARITY(&BLDC_TIMER_NUM,TIM_CHANNEL_1,TIM_OCNPOLARITY_LOW);//0 һֱ��
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_1);//0 һֱ��
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_START,TIM_MOD_HIGH);

        /*  PhaseC configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCx_Disable);   // 0
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_3, TIM_CCxN_Disable); // 0
//        HAL_TIM_OC_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);// 0
//        HAL_TIMEx_OCN_Stop(&BLDC_TIMER_NUM,TIM_CHANNEL_3);//0
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_STOP,CHANNEL_STOP,TIM_MOD_PWM);

        /*  PhaseB configuration */
//        TIM_CCxCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCx_Enable);    // 1
//        TIM_CCxNCmd(BLDC_TIMER_NUM, TIM_Channel_2, TIM_CCxN_Enable);  //ͬ������
//        HAL_TIM_OC_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_2);// 1 ���ſ�
//        HAL_TIMEx_OCN_Start(&BLDC_TIMER_NUM,TIM_CHANNEL_2);//���ſ�,ͬ������
    	SetChannelState(&BLDC_TIMER_NUM,TIM_CHANNEL_3,CHANNEL_START,CHANNEL_START,TIM_MOD_PWM);
        break;
    default:
        MotorA.Step = 0;
    }

}

/**
  * @brief  ����ÿ��ͨ����״̬.
  * @param  htim TIM handle
  * @param  Channel TIM Channel to be enabled
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected ����
  * @param  OC CHANNEL_START:��ͨ��,CHANNEL_STOP:��ͨ��
  * @param  OCN CHANNEL_START:��ͨ��,CHANNEL_STOP:��ͨ��
  * @param  OCxM TIM_MOD
  * 		This parameter can be one of the following values:
  * 		  @arg TIM_MOD_PWM: PWMģʽ
  * 		  @arg TIM_MOD_HIGH: ǿ�Ƹ�
  * 		  @arg TIM_MOD_LOW: ǿ�Ƶ�
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
		if(htim->Instance->CCER & tmpCCxE)//�ѿ���
		{

		}
		else
		{//δ����
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
		if(htim->Instance->CCER & tmpCCxNE)//�ѿ���
		{

		}
		else
		{//δ����
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
		tmpccmrx &= ~TIM_CCMR1_OC1M;//����
		if(OCxM == TIM_MOD_PWM)
			tmpccmrx |= (TIM_OCMODE_PWM1);//��λ110��PWM ģʽ1 �� �����ϼ���ʱ
		else if(OCxM == TIM_MOD_HIGH)
			tmpccmrx |= (TIM_OCMODE_FORCED_ACTIVE);//��λ101:ǿ��Ϊ��Ч��ƽ.ǿ��OC1REF Ϊ�ߡ�
		else if(OCxM == TIM_MOD_LOW)
			tmpccmrx |= (TIM_OCMODE_FORCED_INACTIVE);//��λ100:ǿ��Ϊ��Ч��ƽ.ǿ��OC1REF Ϊ�͡�
		/* Write to TIMx CCMR1 */
		htim->Instance->CCMR1 = tmpccmrx;
	}
	else if(Channel == TIM_CHANNEL_2)
	{
		/* Get the TIMx CCMR1 register value */
		tmpccmrx = htim->Instance->CCMR1;
		tmpccmrx &= ~TIM_CCMR1_OC2M;//����
		if(OCxM == TIM_MOD_PWM)
			tmpccmrx |= (TIM_OCMODE_PWM1 << 8U);//��λ110��PWM ģʽ1 �� �����ϼ���ʱ
		else if(OCxM == TIM_MOD_HIGH)
			tmpccmrx |= (TIM_OCMODE_FORCED_ACTIVE << 8U);//��λ101:ǿ��Ϊ��Ч��ƽ.ǿ��OC1REF Ϊ�ߡ�
		else if(OCxM == TIM_MOD_LOW)
			tmpccmrx |= (TIM_OCMODE_FORCED_INACTIVE << 8U);//��λ100:ǿ��Ϊ��Ч��ƽ.ǿ��OC1REF Ϊ�͡�
		/* Write to TIMx CCMR1 */
		htim->Instance->CCMR1 = tmpccmrx;
	}
	else if(Channel == TIM_CHANNEL_3)
	{
		/* Get the TIMx CCMR1 register value */
		tmpccmrx = htim->Instance->CCMR2;
		tmpccmrx &= ~TIM_CCMR2_OC3M;//����
		if(OCxM == TIM_MOD_PWM)
			tmpccmrx |= (TIM_OCMODE_PWM1);//��λ110��PWM ģʽ1 �� �����ϼ���ʱ
		else if(OCxM == TIM_MOD_HIGH)
			tmpccmrx |= (TIM_OCMODE_FORCED_ACTIVE);//��λ101:ǿ��Ϊ��Ч��ƽ.ǿ��OC1REF Ϊ�ߡ�
		else if(OCxM == TIM_MOD_LOW)
			tmpccmrx |= (TIM_OCMODE_FORCED_INACTIVE);//��λ100:ǿ��Ϊ��Ч��ƽ.ǿ��OC1REF Ϊ�͡�
		/* Write to TIMx CCMR2 */
		htim->Instance->CCMR2 = tmpccmrx;
	}

}










