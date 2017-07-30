#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "stm32f4xx.h"
#include "sys.h"
//6612端口定义
#define AIN1 PBout(13)	
#define AIN2 PBout(11)	
#define BIN1 PDout(9)	
#define BIN2 PDout(11)
#define STBY PBout(15)
#define PWMA   TIM1->CCR1  //PA8
#define PWMB   TIM1->CCR4  //PA11
void Motor6612_Init(void);//6612端口初始化
void Motor_StartOrStop(u8 state);//电机停止或者启动
void Motor_Dir(u8 Right_motordir,u32 Right_motorpwm,u8 Left_motodir,u32 Left_motorpwm);
void Set_Pwm(int moto1,int moto2);
void Motor_Init(void);//电机初始化
int myabs(int a);
void Xianfu_Pwm(void);
#endif
