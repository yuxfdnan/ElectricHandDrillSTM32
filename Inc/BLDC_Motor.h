#ifndef __BLDC_MOTOR_H_
#define __BLDC_MOTOR_H_

#include "user_conf.h"
//#include "stdint.h"


typedef struct
{
    volatile unsigned char State;
    volatile unsigned char Step;
    volatile unsigned char FlagSwitchStep;      //换向标志
    volatile unsigned char FlagBEMF;        //过零标志

    volatile unsigned short PWMTicks;

    volatile unsigned short PWMTicksPre;

    unsigned short ObjectDutyCycle;
    unsigned short NonceDutyCycle;

    unsigned short PWM_Freq;
    unsigned short TimerPeriod;

    unsigned short NonceBEMF;
    unsigned short MaxBEMF;
    unsigned short MinBEMF;

    unsigned short CommutationTimeMax;    //最大换向时间
    unsigned short CommutationTimeMin;       //最小换向时间
    unsigned short CommutationTimePre;       //上次换向时刻


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


