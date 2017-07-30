#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "stm32f4xx.h"
#include "parameter.h"
#include "motor.h"
extern float Position_KP;
void Get_RightMotorSpeed(float time);//���ת�ټ���
void Get_LeftMotorSpeed(float time);//���ת�ټ���
s16 MotorSpeed_Ctrl(float Kp,float Ki,float Kd,float speed_set,float time);//���ת�ٻ�
s16 Pendulum_Ctrl(float Kp,float Ki,float Kd,float angle_set,float time);	//�ǶȻ�
s16 MotorPosition_Ctrl(float Kp,float Ki,float Kd,float position_set,float time);//λ�ÿ���
int velocity(int encoder_left,int encoder_right);
int Position_PID (int Encoder,int Target);//�������ܣ�λ��ʽPID������
int Incremental_PI (int Encoder,int Target);//�������ܣ�����ʽPID������
int Read_Encoder(u8 TIMX);//�������ܣ���λʱ���ȡ����������
#endif
