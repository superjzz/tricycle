#ifndef _TIMER_H_
#define _TIMER_H_

#include "stm32f4xx.h"
#include  "parameter.h"
#include "control.h"
#define ENCODER_TIM_ChuShi (u16)(0)   //���̶�ʱ����ʼֵ�����ɴ���65535 ��ΪF103�Ķ�ʱ����16λ�ġ�
void Pwm_Init(u32 arr,u32 psc);	//Pwm��ʼ��
void Encoder_Init(void);	//��������ʼ��
void MotorEncoder_Init_TIM5(void);//�������1��ʼ��
void MotorEncoder_Init_TIM3(void);//�������2��ʼ��
void Timer4_Init(u16 arr,u16 psc);//��ʱ��4 ϵͳ����
#endif
