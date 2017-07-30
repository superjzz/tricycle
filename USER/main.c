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
int Moto1;    //���PWM����
int i;
//extern float Position_KP;
int main(void)
{ 
   u8 key;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);    //��ʼ����ʱ����
	uart_init(256000);	//��ʼ�����ڲ�����Ϊ115200
  MotorEncoder_Init_TIM3();//������̳�ʼ��
  MotorEncoder_Init_TIM5();//������̳�ʼ��
  Motor6612_Init();  //6612�˿ڳ�ʼ��
  Pwm_Init(7200-1,0);//���̵��pwm��ʼ��  //84M/84=1Mhz�ļ���Ƶ��,��װ��ֵ7200������PWMƵ��Ϊ 1M/500=2Khz.
  Timer4_Init(100-1,83);		//��ʱ��4��ϵͳ���� 100us 1���ж�
  KEY_Init();
  LED_Init();
  STBY=1;//�����������
  RightMotor_Value.motorCNT_set=10000;
  while(1)
	{ 
    key=KEY_Scan(0);		//�õ���ֵ
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



