#ifndef __BLDC_MOTOR_H_
#define __BLDC_MOTOR_H_

#include "user_conf.h"
//#include "stdint.h"


typedef struct
{
    volatile unsigned char State;
    volatile unsigned char Step;
    volatile unsigned char FlagSwitchStep;      //�����־
    volatile unsigned char FlagBEMF;        //�����־

    volatile unsigned short PWMTicks;

    volatile unsigned short PWMTicksPre;

    unsigned short ObjectDutyCycle;
    unsigned short NonceDutyCycle;

    unsigned short PWM_Freq;
    unsigned short TimerPeriod;

    unsigned short NonceBEMF;
    unsigned short MaxBEMF;
    unsigned short MinBEMF;

    unsigned short CommutationTimeMax;    //�����ʱ��
    unsigned short CommutationTimeMin;       //��С����ʱ��
    unsigned short CommutationTimePre;       //�ϴλ���ʱ��


    unsigned short Count1;
    unsigned short Count2;
    unsigned char ADC_BEMF_Filter;

} MotorParam_TypeDef;



/*File BLDC_Motor.c */

extern MotorParam_TypeDef MotorA;

void BLDC_Init(void);

void BLDC_Stop(void);

void BLDC_SetStep(unsigned char step);

void BLDC_SwitchStep(void);


#endif /*__BLDC_MOTOR_H_*/


