#include "timer.h"
#include "DataScope_DP.h"
#include "adc.h"
#include "control.h"
extern __IO uint16_t ADC1ConvertedValue[1];
extern __IO uint32_t ADC1ConvertedVoltage ;
u16 AD_Value;
int Velocity_Pwm;
//extern float Position_KP;
//ʹ��PA8 PA9 PWM���
void Pwm_Init(u32 arr,u32 psc) //arr 4000 psc 0
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	//enable clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

/////////////////////////////////////////////////////////////////////////////
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_11 ;	//PA8 11  ��©��� ������
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);//PA8 ����ΪTIM1_CH1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_TIM1);//PA8 ����ΪTIM1_CH4

	
	/* Compute the prescaler value */
  //PrescalerValue = 83;         //(uint16_t) ( ( SystemCoreClock ) / 168000000 ) - 1;	//1��Ƶ
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = arr;		//PWM����		�Զ���װ��ֵ	168MHZ 8000 ==> 23.8*2us ==> 21kHz	
  TIM_TimeBaseStructure.TIM_Prescaler = psc;		//Ԥ��Ƶ
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;//���������ʱ��Tdts�붨ʱ��ʱ�ӵķ�Ƶ��
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//���ϼ���
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;	//�ظ���������
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);	//init tim1
	
	TIM_ARRPreloadConfig(TIM1, ENABLE);	//ʹ�ܶ�ʱ��1Ԥװ�ػ�����
	//pwm init
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	//PWMģʽ1
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;		//�Ƚ�����ߵ�ƽ
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;	//����״̬�������ƽ������ʱ���õ�
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//PWM���ʹ��
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
  TIM_OCInitStructure.TIM_Pulse = 0;	//reload value
  
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);	//��ʼ����ʱ��1 ͨ��1
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);	//��ʼ����ʱ��1 ͨ��4
	
  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);	//����Ƚ�1Ԥװ��ʹ��
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);	//����Ƚ�4Ԥװ��ʹ��
  //enable tim1
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	TIM_Cmd(TIM1, ENABLE);	
}


//��������ʼ��
void Encoder_Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitTypeDef TIM8_ICInitStructure;
	//����ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);	//��ʱ��8ʱ��ʹ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	//GPIOC ʱ��ʹ��

	//��������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6| GPIO_Pin_7;	//PC8 PC9 ������� ������
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 

  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);//PC8 PC9 ����ΪTIM8��CH3��CH4
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);
	//���ö�ʱ��

  TIM_DeInit(TIM8);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;//����Ƶ
  TIM_TimeBaseStructure.TIM_Period = 3999; //���ڣ�����
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	//����Ƶ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
	//������������
  TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//����ģʽ�����뼫��
  TIM_ICStructInit(&TIM_ICInitStructure);
	TIM8_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM8_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  TIM8_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;         //���������Ƶ,����Ƶ 
  TIM8_ICInitStructure.TIM_ICFilter = 0x6;		//�����˲�
  TIM_ICInit(TIM8, &TIM8_ICInitStructure);
	TIM8_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM8_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
	TIM8_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;         //���������Ƶ,����Ƶ 
	TIM8_ICInitStructure.TIM_ICFilter = 0x6;
	TIM_ICInit(TIM8, &TIM8_ICInitStructure);     
	//TIM_ICInitStructure.TIM_ICFilter = 6;//�˲�����
  //TIM_ICInit(TIM8, &TIM_ICInitStructure);
	//��������жϱ�־
  TIM_ClearFlag(TIM8, TIM_FLAG_Update);	//��������жϱ�־
  //TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);	//�����ж�
	TIM8->CNT = 0 ;   
	TIM_Cmd(TIM8, ENABLE); // ʹ�ܶ�ʱ��
	
}

//�������1��ʼ��
void MotorEncoder_Init_TIM5(void)
{
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitTypeDef TIM5_ICInitStructure;
	//����ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);	//��ʱ��5ʱ��ʹ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	//GPIOA ʱ��ʹ��

	//��������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1;	//PA0 PA1 ������� ������
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);//PA0 PA1 ����ΪTIM5��CH3��CH4
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
	//���ö�ʱ��

  TIM_DeInit(TIM5);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;//����Ƶ
  TIM_TimeBaseStructure.TIM_Period =65535-1; //�������390�����壬4��Ƶ1600��
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//����Ƶ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	//������������
  TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//����ģʽ�����뼫��
  TIM_ICStructInit(&TIM_ICInitStructure);
	TIM5_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;         //���������Ƶ,����Ƶ 
  TIM5_ICInitStructure.TIM_ICFilter = 0x8;		//���벶���˲�
  TIM_ICInit(TIM5, &TIM5_ICInitStructure);
	TIM5_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
	TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;         //���������Ƶ,����Ƶ 
	TIM5_ICInitStructure.TIM_ICFilter = 0x8;
	TIM_ICInit(TIM5, &TIM5_ICInitStructure);     
	//TIM_ICInitStructure.TIM_ICFilter = 6;//�˲�����
  //TIM_ICInit(TIM8, &TIM_ICInitStructure);
	//��������жϱ�־
  TIM_ClearFlag(TIM5, TIM_FLAG_Update);	//��������жϱ�־
  //TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);	//�����ж�
	TIM5->CNT = ENCODER_TIM_ChuShi ;//����ֵ��ʼ����time.h��
	TIM_Cmd(TIM5, ENABLE); // ʹ�ܶ�ʱ��
}



//�������2��ʼ��
void MotorEncoder_Init_TIM3(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitTypeDef TIM3_ICInitStructure;
	//����ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	//��ʱ��3ʱ��ʹ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	//GPIOC ʱ��ʹ��

	//��������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6| GPIO_Pin_7;	//PC6 PC7 ������� ������
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(GPIOC, &GPIO_InitStructure); 

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);//PC6 PC7 ����ΪTIM3��CH1��CH2
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);
	//���ö�ʱ��

    TIM_DeInit(TIM3);
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0;//����Ƶ
    TIM_TimeBaseStructure.TIM_Period =65535-1; //�������390�����壬4��Ƶ1600��
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//����Ƶ
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	//������������
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//����ģʽ�����뼫��
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM3_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
    TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;         //���������Ƶ,����Ƶ 
    TIM3_ICInitStructure.TIM_ICFilter = 0x8;		//���벶���˲�
    TIM_ICInit(TIM3, &TIM3_ICInitStructure);
	TIM3_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
	TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;         //���������Ƶ,����Ƶ 
	TIM3_ICInitStructure.TIM_ICFilter = 0x8;
	TIM_ICInit(TIM3, &TIM3_ICInitStructure);     
	//TIM_ICInitStructure.TIM_ICFilter = 6;//�˲�����
  //TIM_ICInit(TIM8, &TIM_ICInitStructure);
	//��������жϱ�־
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);	//��������жϱ�־
  //TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);	//�����ж�
	TIM3->CNT = ENCODER_TIM_ChuShi ;////����ֵ��ʼ����time.h��
	TIM_Cmd(TIM3, ENABLE); // ʹ�ܶ�ʱ��
}
//ͨ�ö�ʱ��4
void Timer4_Init(u16 arr,u16 psc)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC->APB1ENR|=1<<2;	//TIM4ʱ��ʹ��    
 	TIM4->ARR=arr;  	//�趨�������Զ���װֵ 
	TIM4->PSC=psc;  	//Ԥ��Ƶ��	  
	TIM4->DIER|=1<<0;   //��������ж�	  
	TIM4->CR1|=0x01;    //ʹ�ܶ�ʱ��4
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn; //��ʱ��4�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01; //�����ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

//��ʱ��4�жϷ������	 
void TIM4_IRQHandler(void)
{ 	
	int i;
	unsigned char Send_Count;
	if(TIM4->SR&0X0001)//����ж�
	{	
		Task_Time.Duty_1ms++;
		Task_Time.sys_delay++;
		if(Task_Time.Duty_1ms >= 30 )    //2���붨ʱ�ж�
		{

  //      RightMotor_Value.motorspeed_set=Position_PID(TIM5->CNT,RightMotor_Value.motorCNT_set);
      RightMotor_Value.motor_speed=Read_Encoder(5);
    //RightMotor_Value.motorspeed_set=MotorSpeed_Ctrl(100,20,0,10,3);//ת�ٻ� 
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
	TIM4->SR&=~(1<<0);//����жϱ�־λ 	    
}
