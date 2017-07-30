#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "stm32f4xx.h"
#include "parameter.h"
#include "motor.h"
extern float Position_KP;
void Get_RightMotorSpeed(float time);//电机转速计算
void Get_LeftMotorSpeed(float time);//电机转速计算
s16 MotorSpeed_Ctrl(float Kp,float Ki,float Kd,float speed_set,float time);//电机转速环
s16 Pendulum_Ctrl(float Kp,float Ki,float Kd,float angle_set,float time);	//角度环
s16 MotorPosition_Ctrl(float Kp,float Ki,float Kd,float position_set,float time);//位置控制
int velocity(int encoder_left,int encoder_right);
int Position_PID (int Encoder,int Target);//函数功能：位置式PID控制器
int Incremental_PI (int Encoder,int Target);//函数功能：增量式PID控制器
int Read_Encoder(u8 TIMX);//函数功能：单位时间读取编码器计数
#endif
