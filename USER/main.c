#include "stm32f4xx.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "lcd.h"
#include "adc.h"
#include "timer.h"
#include "DataScope_DP.h"
#include "motor.h"
#include "control.h"
#include "led.h"
#include "key.h"

//add a different
int Moto1;    //电机PWM变量
int i;
//extern float Position_KP;
int main(void)
{ 
   u8 key;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);    //初始化延时函数
	uart_init(256000);	//初始化串口波特率为115200
  MotorEncoder_Init_TIM3();//电机码盘初始化
  MotorEncoder_Init_TIM5();//电机码盘初始化
  Motor6612_Init();  //6612端口初始化
  Pwm_Init(7200-1,0);//码盘电机pwm初始化  //84M/84=1Mhz的计数频率,重装载值7200，所以PWM频率为 1M/500=2Khz.
  Timer4_Init(100-1,83);		//定时器4，系统心跳 100us 1次中断
  KEY_Init();
  LED_Init();
  STBY=1;//开机启动电机
  RightMotor_Value.motorCNT_set=10000;
  while(1)
	{ 
    key=KEY_Scan(0);		//得到键值
    if(key)
		{						   
			switch(key)
			{				 
				case KEY1_PRES:	//
         STBY=0;
					break;
				case KEY2_PRES:	//	
          RightMotor_Value.motorCNT_set+=2000;
					break;
				case KEY3_PRES:	//
          RightMotor_Value.motorCNT_set-=2000;
					break;
			}
		}else    
    LED0=!LED0;
		delay_ms(500);  

	}
}



