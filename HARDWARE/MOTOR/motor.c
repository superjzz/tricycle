#include "motor.h"
#include "sys.h"
#include "parameter.h"
//6612�˿ڳ�ʼ��
void Motor6612_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure_B;
  GPIO_InitTypeDef  GPIO_InitStructure_D;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  //GPIOB13 11 15��ʼ������
  GPIO_InitStructure_B.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_11|GPIO_Pin_15 ;
  GPIO_InitStructure_B.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure_B.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure_B.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure_B.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOB, &GPIO_InitStructure_B);//��ʼ��
  //GPIOD9 11��ʼ������
  GPIO_InitStructure_D.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_11;
  GPIO_InitStructure_D.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure_D.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure_D.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure_D.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOD, &GPIO_InitStructure_D);//��ʼ��	
       
	GPIO_ResetBits(GPIOB,GPIO_Pin_13);//ȫ������ ���ֹͣ���������Դ�Ͽ�����
  GPIO_ResetBits(GPIOB,GPIO_Pin_11);
  GPIO_ResetBits(GPIOB,GPIO_Pin_15);
  GPIO_ResetBits(GPIOD,GPIO_Pin_9);  
  GPIO_ResetBits(GPIOD,GPIO_Pin_11);  

}

//6612STBY-PB15 ���Ƶ����������ֹͣ
//������1��ʼ 0ֹͣ
void Motor_StartOrStop(u8 state)
{
	if(state)
	{
		STBY=1;	//6612STBY��������Ϊ����
	}
	else
	{
		STBY=0;	//����Ϊֹͣ
	}
}

//�������ҵ������ת
//������ Left_motodir Left_motorpwm   Right_motordir Right_motorpwm 
//       �� 0/1            pwm        ��    0/1           pwm
void Motor_Dir(u8 Left_motordir,u32 Left_motorpwm,u8 Right_motordir,u32 Right_motorpwm)
{
  STBY=1;//���ӵ�Դ

    switch(Left_motordir){
      case 1 :AIN1=1;AIN2=0;TIM1->CCR4 = Left_motorpwm;break;//���motor��ת
      case 0 :AIN1=0;AIN2=1;TIM1->CCR4 = Left_motorpwm;break;//���motor��ת
      default: break;
    }

		 switch(Right_motordir){
      case 1 :BIN1=0;BIN2=1;TIM1->CCR1 = Right_motorpwm;break;//�ұ�motor��ת
      case 0 :BIN1=1;BIN2=0;TIM1->CCR1 = Right_motorpwm;break;//�ұ�motor��ת
      default: break;
     }
}
/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ���������PWM������PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int moto1,int moto2)
{     
    	if(moto2<0)			AIN2=1,			AIN1=0;  //����
			else 	          AIN2=0,			AIN1=1;
			PWMA=myabs(moto2);
		  if(moto1>0)   	BIN1=1, 		BIN2=0;       //����
			else            BIN1=0;			BIN2=1;
			PWMB=myabs(moto1);	
}
/**************************************************************************
�������ܣ�����PWM��ֵ 
��ڲ�������
����  ֵ����
**************************************************************************/
void Xianfu_Pwm(void)
{	
	  int Amplitude=6900;    //===PWM������7200 ������6900
    if(LeftMotor_Value.motorspeed_set<-Amplitude) LeftMotor_Value.motorspeed_set=-Amplitude;	
		if(LeftMotor_Value.motorspeed_set>Amplitude)  LeftMotor_Value.motorspeed_set=Amplitude;	
	  if(RightMotor_Value.motorspeed_set<-Amplitude) RightMotor_Value.motorspeed_set=-Amplitude;	
		if(RightMotor_Value.motorspeed_set>Amplitude)  RightMotor_Value.motorspeed_set=Amplitude;		
	
}
//�����ʼ��
void Motor_Init(void)
{
	//�˿ڳ�ʼ�� E2λBREAK E0ΪDIR
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);	//GPIOEʱ��ʹ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_2;	//PE0 PE2 ��©��� ������
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;	
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOE, &GPIO_InitStructure); 
	//�����ʼ״̬����
	Motor_StartOrStop(0);//���ֹͣ
//	Motor_Dir(1);	//�������
}

/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

