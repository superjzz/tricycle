#ifndef _PARAMETER_H_
#define _PARAMETER_H_

#include "stm32f4xx.h"
#include "sys.h"
/*********************ϵͳ��ؽṹ��*********************/


//PID�ṹ��
typedef struct
{
	float kp;
	float kd;
	float ki;
}pid_f;


//ϵͳʱ������
typedef struct
{
	u8 Duty_1ms;	//1ms����
	u8 Duty_5ms;	//5ms����
	u8 Duty_4ms;	//4ms����
	u8 Duty_10ms;	//10ms����
	u16 sys_delay;	//ϵͳ��ʱ����
}TASK_TIME;

//ϵͳ��ز���
typedef struct
{
	s16 syspwm;
}SYSTEM_VALUE;

//�����ز���
typedef struct
{
	s16 motor_speed;	//�����ǰת�� r/min���˲���
	int motorspeed_set;	//����ٶȸ���
	int motor_CNT;          //��ǰ�������λ��
  int motorCNT_set;          //�趨�������λ��
   u16 motor_speed_nolb;  //�����ǰת�� r/min��δ�˲���
   u16 motor_change;      //��λʱ���ڵ�����̸����ı仯ֵ
}MOTOR_VALUE;
/*********************��ر�������*********************/
extern TASK_TIME Task_Time;//ϵͳ����ʱ��
extern SYSTEM_VALUE System_Value;//ϵͳ��ز���
extern MOTOR_VALUE RightMotor_Value;
extern MOTOR_VALUE LeftMotor_Value;
#endif
