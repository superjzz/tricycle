#ifndef _PARAMETER_H_
#define _PARAMETER_H_

#include "stm32f4xx.h"
#include "sys.h"
/*********************系统相关结构体*********************/


//PID结构体
typedef struct
{
	float kp;
	float kd;
	float ki;
}pid_f;


//系统时间任务
typedef struct
{
	u8 Duty_1ms;	//1ms任务
	u8 Duty_5ms;	//5ms任务
	u8 Duty_4ms;	//4ms任务
	u8 Duty_10ms;	//10ms任务
	u16 sys_delay;	//系统延时任务
}TASK_TIME;

//系统相关参数
typedef struct
{
	s16 syspwm;
}SYSTEM_VALUE;

//电机相关参数
typedef struct
{
	s16 motor_speed;	//电机当前转速 r/min（滤波后）
	int motorspeed_set;	//电机速度给定
	int motor_CNT;          //当前电机码盘位置
  int motorCNT_set;          //设定电机码盘位置
   u16 motor_speed_nolb;  //电机当前转速 r/min（未滤波）
   u16 motor_change;      //单位时间内电机码盘个数的变化值
}MOTOR_VALUE;
/*********************相关变量声明*********************/
extern TASK_TIME Task_Time;//系统任务时间
extern SYSTEM_VALUE System_Value;//系统相关参数
extern MOTOR_VALUE RightMotor_Value;
extern MOTOR_VALUE LeftMotor_Value;
#endif
