#include "motor.h"
#include "sys.h"
#include "parameter.h"
//6612端口初始化
void Motor6612_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure_B;
  GPIO_InitTypeDef  GPIO_InitStructure_D;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  //GPIOB13 11 15初始化设置
  GPIO_InitStructure_B.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_11|GPIO_Pin_15 ;
  GPIO_InitStructure_B.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure_B.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure_B.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure_B.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure_B);//初始化
  //GPIOD9 11初始化设置
  GPIO_InitStructure_D.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_11;
  GPIO_InitStructure_D.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure_D.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure_D.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure_D.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOD, &GPIO_InitStructure_D);//初始化	
       
	GPIO_ResetBits(GPIOB,GPIO_Pin_13);//全部拉低 电机停止工作且与电源断开连接
  GPIO_ResetBits(GPIOB,GPIO_Pin_11);
  GPIO_ResetBits(GPIOB,GPIO_Pin_15);
  GPIO_ResetBits(GPIOD,GPIO_Pin_9);  
  GPIO_ResetBits(GPIOD,GPIO_Pin_11);  

}

//6612STBY-PB15 控制电机启动或者停止
//参数：1开始 0停止
void Motor_StartOrStop(u8 state)
{
	if(state)
	{
		STBY=1;	//6612STBY引脚拉高为启动
	}
	else
	{
		STBY=0;	//拉低为停止
	}
}

//控制左右电机正反转
//参数： Left_motodir Left_motorpwm   Right_motordir Right_motorpwm 
//       左 0/1            pwm        右    0/1           pwm
void Motor_Dir(u8 Left_motordir,u32 Left_motorpwm,u8 Right_motordir,u32 Right_motorpwm)
{
  STBY=1;//连接电源

    switch(Left_motordir){
      case 1 :AIN1=1;AIN2=0;TIM1->CCR4 = Left_motorpwm;break;//左边motor正转
      case 0 :AIN1=0;AIN2=1;TIM1->CCR4 = Left_motorpwm;break;//左边motor反转
      default: break;
    }

		 switch(Right_motordir){
      case 1 :BIN1=0;BIN2=1;TIM1->CCR1 = Right_motorpwm;break;//右边motor正转
      case 0 :BIN1=1;BIN2=0;TIM1->CCR1 = Right_motorpwm;break;//右边motor反转
      default: break;
     }
}
/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int moto1,int moto2)
{     
    	if(moto2<0)			AIN2=1,			AIN1=0;  //右轮
			else 	          AIN2=0,			AIN1=1;
			PWMA=myabs(moto2);
		  if(moto1>0)   	BIN1=1, 		BIN2=0;       //左轮
			else            BIN1=0;			BIN2=1;
			PWMB=myabs(moto1);	
}
/**************************************************************************
函数功能：限制PWM赋值 
入口参数：无
返回  值：无
**************************************************************************/
void Xianfu_Pwm(void)
{	
	  int Amplitude=6900;    //===PWM满幅是7200 限制在6900
    if(LeftMotor_Value.motorspeed_set<-Amplitude) LeftMotor_Value.motorspeed_set=-Amplitude;	
		if(LeftMotor_Value.motorspeed_set>Amplitude)  LeftMotor_Value.motorspeed_set=Amplitude;	
	  if(RightMotor_Value.motorspeed_set<-Amplitude) RightMotor_Value.motorspeed_set=-Amplitude;	
		if(RightMotor_Value.motorspeed_set>Amplitude)  RightMotor_Value.motorspeed_set=Amplitude;		
	
}
//电机初始化
void Motor_Init(void)
{
	//端口初始化 E2位BREAK E0为DIR
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);	//GPIOE时钟使能
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_2;	//PE0 PE2 开漏输出 不上拉
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;	
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOE, &GPIO_InitStructure); 
	//电机初始状态控制
	Motor_StartOrStop(0);//电机停止
//	Motor_Dir(1);	//电机正传
}

/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

