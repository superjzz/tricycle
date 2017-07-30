#ifndef _TIMER_H_
#define _TIMER_H_

#include "stm32f4xx.h"
#include  "parameter.h"
#include "control.h"
#define ENCODER_TIM_ChuShi (u16)(0)   //码盘定时器初始值，不可大于65535 因为F103的定时器是16位的。
void Pwm_Init(u32 arr,u32 psc);	//Pwm初始化
void Encoder_Init(void);	//编码器初始化
void MotorEncoder_Init_TIM5(void);//电机码盘1初始化
void MotorEncoder_Init_TIM3(void);//电机码盘2初始化
void Timer4_Init(u16 arr,u16 psc);//定时器4 系统心跳
#endif
