#include "timer.h"
#include "DataScope_DP.h"
#include "adc.h"
#include "control.h"
extern __IO uint16_t ADC1ConvertedValue[1];
extern __IO uint32_t ADC1ConvertedVoltage ;
u16 AD_Value;
int Velocity_Pwm;
//extern float Position_KP;
//使能PA8 PA9 PWM输出
void Pwm_Init(u32 arr,u32 psc) //arr 4000 psc 0
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	//enable clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

/////////////////////////////////////////////////////////////////////////////
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_11 ;	//PA8 11  开漏输出 不上拉
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);//PA8 复用为TIM1_CH1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_TIM1);//PA8 复用为TIM1_CH4

	
	/* Compute the prescaler value */
  //PrescalerValue = 83;         //(uint16_t) ( ( SystemCoreClock ) / 168000000 ) - 1;	//1分频
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = arr;		//PWM周期		自动重装载值	168MHZ 8000 ==> 23.8*2us ==> 21kHz	
  TIM_TimeBaseStructure.TIM_Prescaler = psc;		//预分频
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;//死区与采样时钟Tdts与定时器时钟的分频比
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;	//重复计数次数
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);	//init tim1
	
	TIM_ARRPreloadConfig(TIM1, ENABLE);	//使能定时器1预装载缓冲器
	//pwm init
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	//PWM模式1
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;		//比较输出高电平
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;	//空闲状态的输出电平，死区时才用到
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//PWM输出使能
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
  TIM_OCInitStructure.TIM_Pulse = 0;	//reload value
  
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);	//初始化定时器1 通道1
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);	//初始化定时器1 通道4
	
  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);	//输出比较1预装载使能
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);	//输出比较4预装载使能
  //enable tim1
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	TIM_Cmd(TIM1, ENABLE);	
}


//编码器初始化
void Encoder_Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitTypeDef TIM8_ICInitStructure;
	//配置时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);	//定时器8时钟使能
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	//GPIOC 时钟使能

	//配置引脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6| GPIO_Pin_7;	//PC8 PC9 推挽输出 不上拉
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 

  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);//PC8 PC9 复用为TIM8的CH3和CH4
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);
	//配置定时器

  TIM_DeInit(TIM8);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;//不分频
  TIM_TimeBaseStructure.TIM_Period = 3999; //周期，这里
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	//不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
	//配置正交计数
  TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//计数模式与输入极性
  TIM_ICStructInit(&TIM_ICInitStructure);
	TIM8_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM8_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  TIM8_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;         //配置输入分频,不分频 
  TIM8_ICInitStructure.TIM_ICFilter = 0x6;		//输入滤波
  TIM_ICInit(TIM8, &TIM8_ICInitStructure);
	TIM8_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM8_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM8_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;         //配置输入分频,不分频 
	TIM8_ICInitStructure.TIM_ICFilter = 0x6;
	TIM_ICInit(TIM8, &TIM8_ICInitStructure);     
	//TIM_ICInitStructure.TIM_ICFilter = 6;//滤波设置
  //TIM_ICInit(TIM8, &TIM_ICInitStructure);
	//清除所有中断标志
  TIM_ClearFlag(TIM8, TIM_FLAG_Update);	//清除更新中断标志
  //TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);	//更新中断
	TIM8->CNT = 0 ;   
	TIM_Cmd(TIM8, ENABLE); // 使能定时器
	
}

//电机码盘1初始化
void MotorEncoder_Init_TIM5(void)
{
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitTypeDef TIM5_ICInitStructure;
	//配置时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);	//定时器5时钟使能
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	//GPIOA 时钟使能

	//配置引脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1;	//PA0 PA1 推挽输出 不上拉
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);//PA0 PA1 复用为TIM5的CH3和CH4
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
	//配置定时器

  TIM_DeInit(TIM5);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;//不分频
  TIM_TimeBaseStructure.TIM_Period =65535-1; //电机码盘390个脉冲，4倍频1600个
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	//配置正交计数
  TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//计数模式与输入极性
  TIM_ICStructInit(&TIM_ICInitStructure);
	TIM5_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;         //配置输入分频,不分频 
  TIM5_ICInitStructure.TIM_ICFilter = 0x8;		//输入捕获滤波
  TIM_ICInit(TIM5, &TIM5_ICInitStructure);
	TIM5_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;         //配置输入分频,不分频 
	TIM5_ICInitStructure.TIM_ICFilter = 0x8;
	TIM_ICInit(TIM5, &TIM5_ICInitStructure);     
	//TIM_ICInitStructure.TIM_ICFilter = 6;//滤波设置
  //TIM_ICInit(TIM8, &TIM_ICInitStructure);
	//清除所有中断标志
  TIM_ClearFlag(TIM5, TIM_FLAG_Update);	//清除更新中断标志
  //TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);	//更新中断
	TIM5->CNT = ENCODER_TIM_ChuShi ;//计数值初始化在time.h中
	TIM_Cmd(TIM5, ENABLE); // 使能定时器
}



//电机码盘2初始化
void MotorEncoder_Init_TIM3(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitTypeDef TIM3_ICInitStructure;
	//配置时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	//定时器3时钟使能
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	//GPIOC 时钟使能

	//配置引脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6| GPIO_Pin_7;	//PC6 PC7 推挽输出 不上拉
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(GPIOC, &GPIO_InitStructure); 

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);//PC6 PC7 复用为TIM3的CH1和CH2
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);
	//配置定时器

    TIM_DeInit(TIM3);
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0;//不分频
    TIM_TimeBaseStructure.TIM_Period =65535-1; //电机码盘390个脉冲，4倍频1600个
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//不分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	//配置正交计数
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//计数模式与输入极性
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM3_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
    TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;         //配置输入分频,不分频 
    TIM3_ICInitStructure.TIM_ICFilter = 0x8;		//输入捕获滤波
    TIM_ICInit(TIM3, &TIM3_ICInitStructure);
	TIM3_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;         //配置输入分频,不分频 
	TIM3_ICInitStructure.TIM_ICFilter = 0x8;
	TIM_ICInit(TIM3, &TIM3_ICInitStructure);     
	//TIM_ICInitStructure.TIM_ICFilter = 6;//滤波设置
  //TIM_ICInit(TIM8, &TIM_ICInitStructure);
	//清除所有中断标志
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);	//清除更新中断标志
  //TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);	//更新中断
	TIM3->CNT = ENCODER_TIM_ChuShi ;////计数值初始化在time.h中
	TIM_Cmd(TIM3, ENABLE); // 使能定时器
}
//通用定时器4
void Timer4_Init(u16 arr,u16 psc)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC->APB1ENR|=1<<2;	//TIM4时钟使能    
 	TIM4->ARR=arr;  	//设定计数器自动重装值 
	TIM4->PSC=psc;  	//预分频器	  
	TIM4->DIER|=1<<0;   //允许更新中断	  
	TIM4->CR1|=0x01;    //使能定时器4
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn; //定时器4中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01; //子优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

//定时器4中断服务程序	 
void TIM4_IRQHandler(void)
{ 	
	int i;
	unsigned char Send_Count;
	if(TIM4->SR&0X0001)//溢出中断
	{	
		Task_Time.Duty_1ms++;
		Task_Time.sys_delay++;
		if(Task_Time.Duty_1ms >= 30 )    //2毫秒定时中断
		{

  //      RightMotor_Value.motorspeed_set=Position_PID(TIM5->CNT,RightMotor_Value.motorCNT_set);
      RightMotor_Value.motor_speed=Read_Encoder(5);
    //RightMotor_Value.motorspeed_set=MotorSpeed_Ctrl(100,20,0,10,3);//转速环 
      RightMotor_Value.motorspeed_set=velocity(0,RightMotor_Value.motor_speed);
      Xianfu_Pwm();
      Set_Pwm(0, RightMotor_Value.motorspeed_set);
            DataScope_Get_Channel_Data(TIM5->CNT, 1 );
            DataScope_Get_Channel_Data(RightMotor_Value.motor_speed, 2 );
            DataScope_Get_Channel_Data(RightMotor_Value.motorspeed_set, 3 );
 //           DataScope_Get_Channel_Data(RightMotor_Value.motorspeed_set, 4);
				Send_Count = DataScope_Data_Generate(3);
         
					for( i = 0 ; i < Send_Count; i++)
					{
					while((USART1->SR&0X40)==0);  
					USART1->DR = DataScope_OutPut_Buffer[i]; 
					}
				Task_Time.Duty_1ms = 0;
		 }
	}
	TIM4->SR&=~(1<<0);//清除中断标志位 	    
}
