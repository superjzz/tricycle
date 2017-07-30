#include "control.h"
//Right���ת�ټ���
//������ time ������ִ������ ��λms 
void Get_RightMotorSpeed(float time)
{
	u16 n_cnt;
	u16 motor_cnt;
   u16 motor_maichong=65534;//�������390�����壬4��Ƶ1560��
	u16 n_motor_spd;
	static u16 last_cnt;
	n_cnt =TIM5->CNT;
	if((TIM5->CR1 &(1<<4)) == 0)	//��ת
	{
		if(n_cnt>=last_cnt)	//�������������
		{
			motor_cnt = n_cnt-last_cnt;	//������������ӵĸ���
			last_cnt = n_cnt;
		}
		else	//����
		{
			motor_cnt = motor_maichong - last_cnt + n_cnt;
			last_cnt = n_cnt; 
		}
	}
	else	//��ת
	{
		if(n_cnt<=last_cnt)
		{
			motor_cnt = last_cnt - n_cnt;	//��������ӵ�����
			last_cnt = n_cnt; 	
		}
		else		//��ת����
      {
			motor_cnt = motor_maichong - n_cnt + last_cnt;
			last_cnt = n_cnt; 
      }
	}
   RightMotor_Value.motor_change=motor_cnt;
	n_motor_spd =  (motor_cnt/time)*38;// ((������/1560)/time)*60000ms = r/min
   RightMotor_Value.motor_speed_nolb=n_motor_spd;
	RightMotor_Value.motor_speed = RightMotor_Value.motor_speed + 6.24f*(time/1000.0f)*5*(n_motor_spd - RightMotor_Value.motor_speed)	;//��ͨ�˲�
	//System_Value.motor_speed =n_motor_spd;
	//Task_Time.Duty_1ms = 0;//����ʱ���־λ��0
}

//Left���ת�ټ���
//������ time ������ִ������ ��λms 
void Get_LeftMotorSpeed(float time)
{
	u16 n_cnt;
	u16 motor_cnt;
   u16 motor_maichong=1560;//�������390�����壬4��Ƶ1560��
	u16 n_motor_spd;
	static u16 last_cnt;
	n_cnt =TIM3->CNT;
	if((TIM3->CR1 &(1<<4)) == 0)	//��ת
	{
		if(n_cnt>=last_cnt)	//�������������
		{
			motor_cnt = n_cnt-last_cnt;	//������������ӵĸ���
			last_cnt = n_cnt;
		}
		else	//����
		{
			motor_cnt = motor_maichong - last_cnt + n_cnt;
			last_cnt = n_cnt; 
		}
	}
	else	//��ת
	{
		if(n_cnt<=last_cnt)
		{
			motor_cnt = last_cnt - n_cnt;	//��������ӵ�����
			last_cnt = n_cnt; 	
		}
		else		//��ת����
      {
			motor_cnt = motor_maichong - n_cnt + last_cnt;
			last_cnt = n_cnt; 
      }
	}
   LeftMotor_Value.motor_change=motor_cnt;
	n_motor_spd =  (motor_cnt/time)*36;// ((������/1660)/time)*60000ms = r/min
   LeftMotor_Value.motor_speed_nolb=n_motor_spd;
	LeftMotor_Value.motor_speed = LeftMotor_Value.motor_speed + 6.24f*(time/1000.0f)*5*(n_motor_spd - LeftMotor_Value.motor_speed)	;//��ͨ�˲�
	//System_Value.motor_speed =n_motor_spd;
	//Task_Time.Duty_1ms = 0;//����ʱ���־λ��0
}

/**************************************************************************
�������ܣ���λʱ���ȡ����������
��ڲ�������ʱ��
����  ֵ���ٶ�ֵ
**************************************************************************/
int Read_Encoder(u8 TIMX)
{
    int Encoder_TIM;    
   switch(TIMX)
	 {
//   case 2:  Encoder_TIM= (short)TIM2 -> CNT;  TIM2 -> CNT=0;break;
		 case 3:  Encoder_TIM= (short)TIM3 -> CNT;  TIM3 -> CNT=0;
     break;	
//   case 4:  Encoder_TIM= (short)TIM4 -> CNT;  TIM4 -> CNT=0;break;
     case 5:  Encoder_TIM= (short)TIM5 -> CNT;  TIM5 -> CNT=0;
     break;	     
		 default:  Encoder_TIM=0;
	 }
		return Encoder_TIM;
}

/**************************************************************************
�������ܣ�λ��ʽPID������
��ڲ���������������λ����Ϣ��Ŀ��λ��
����  ֵ�����PWM
����λ��ʽ��ɢPID��ʽ 
pwm=Kp*e(k)+Ki*��e(k)+Kd[e��k��-e(k-1)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  
��e(k)����e(k)�Լ�֮ǰ��ƫ����ۻ���;����kΪ1,2,,k;
pwm�������
**************************************************************************/
int Position_PID (int Encoder,int Target)
{ 	
	 
   float Position_KP=150,Position_KI=0.01,Position_KD=1000;
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //����ƫ��
	 Integral_bias+=Bias;	                                 //���ƫ��Ļ���
	 Pwm=Position_KP*Bias+Position_KI*Integral_bias+Position_KD*(Bias-Last_Bias);       //λ��ʽPID������
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}

/**************************************************************************
�������ܣ��ٶ�PI���� �޸�ǰ�������ٶȣ�����Target_Velocity�����磬�ĳ�60�ͱȽ�����
��ڲ��������ֱ������仯�����ֱ������仯
����  ֵ���ٶȿ���PWM
��    �ߣ�ƽ��С��֮��
**************************************************************************/
int velocity(int encoder_left,int encoder_right)
{  
    static float Velocity,Encoder_Least,Encoder,Movement;
	  static float Encoder_Integral;
	  float kp=70,ki=6;
	  //=============�ٶ�PI������=======================//	
		Encoder_Least =(encoder_left+encoder_right)+20;                    //===��ȡ�����ٶ�ƫ��==�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶȣ��˴�Ϊ�㣩 
		Encoder *= 0.8;		                                                //===һ�׵�ͨ�˲���       
		Encoder += Encoder_Least*0.2;	                                    //===һ�׵�ͨ�˲���    
		Encoder_Integral +=Encoder;                                       //===���ֳ�λ�� ����ʱ�䣺10ms
		Encoder_Integral=Encoder_Integral-Movement;                       //===����ң�������ݣ�����ǰ������
		if(Encoder_Integral>10000)  Encoder_Integral=10000;             //===�����޷�
		if(Encoder_Integral<-10000)	Encoder_Integral=-10000;              //===�����޷�	
		Velocity=Encoder*kp+Encoder_Integral*ki;                          //===�ٶȿ���	
		return Velocity;
}

/**************************************************************************
�������ܣ�����PI������
��ڲ���������������ֵ��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI (int Encoder,int Target)
{ 	
   float Kp=100,Ki=20;	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //����ƫ��
	 Pwm+=Kp*(Bias-Last_bias)+Ki*Bias;   //����ʽPI������
	 Last_bias=Bias;	                   //������һ��ƫ�� 
	 return Pwm;                         //�������
}




//���ת�ٻ�
//������ִ������
//			PID��Ӧ����
//			ת��Ŀ��ֵ
//���أ�PWM��ֵ
s16 MotorSpeed_Ctrl(float Kp,float Ki,float Kd,float speed_set,float time)
{
	s16 now_err;	//��ǰ���
	s16 pwm_out;	//PWM�����ֵ
	static s16 last_err;//��һʱ�̵����
	static s16 err_d;	//���΢�� 
	static s16 err_i;//������
	now_err = speed_set - RightMotor_Value.motor_speed;//��ǰ���
	if((now_err<20)&&(now_err>(-20)))	//��������
	{
		now_err = 0;
	}
	err_i += Ki*((now_err)*(time));	//��������
	err_d = Kd*((now_err-last_err)/(time));	//΢������
	pwm_out = Kp*(now_err+err_d+err_i);	//PID��ʽ
	last_err = now_err;	//�������
	return pwm_out;
}

//�ڸ˿���
//������ִ������
//			PID��Ӧ����
//			�ڸ�λ��Ŀ��Ƕ�angle_set
//���أ�PWM��ֵ
s16 Pendulum_Ctrl(float Kp,float Ki,float Kd,float angle_set,float time)
{
	s16 angle_err;	//��ǰ���
	s16 pwm_out;	//PWM�����ֵ
	static s16 last_err;//��һʱ�̵����
	static s16 err_d=0;	//���΢�� 
	static s16 err_i=0;//������
	float  now_angle;	//��ǰ�Ƕ�
	now_angle = TIM8->CNT  ;
	angle_err = now_angle - angle_set;//��ǰ���
	if((angle_err>-2)&&(angle_err<(2)))	//�������� ����5��
	{
		angle_err = 0;
	}
//	if((angle_err >250) || (angle_err<-250))	//ʧ��
//	{
//		angle_err = 0;
//		TIM1->CCR1 = 1000;
//	}
	err_i += Ki*((angle_err)*(time));	//��������
	err_d = Kd*((angle_err-last_err)/(time));	//΢������
	pwm_out = Kp*(angle_err+err_d+err_i);	//PID��ʽ
	last_err = angle_err;	//�������
	if(pwm_out > 3500)  //3500
	{
		pwm_out = 3500;
	}
	else
	{
		if(pwm_out < (-3500))
		{
			pwm_out = (-3500);
		}
	}
	return pwm_out;
}


//���λ�û�����
//������ִ������
//			PID��Ӧ����
//			Ŀ��λ��
//���أ�PWM��ֵ
s16 MotorPosition_Ctrl(float Kp,float Ki,float Kd,float position_set,float time)
{
	s16 position_err;	//��ǰ���
	s16 pwm_out;	//PWM�����ֵ
	static s16 last_err;//��һʱ�̵����
	static s16 err_d;	//���΢�� 
	static s16 err_i;//������
	s16  now_position;	//��ǰ�Ƕ�
	now_position = TIM5->CNT  ;
//	if(now_position > 4000)
//	{
//		now_position = (now_position%4000);
//	}
	position_err = position_set - now_position;//��ǰ���
	if(position_err <(-2200))
	{
		position_err = position_err + 3999;
	}
	if((position_err>-5)&&(position_err<(5)))	//�������� 
	{
		position_err = 0;
	}
//	if((position_err >400) || (position_err<-400))	//ʧ��
//	{
//		Motor_StartOrStop(0);
//	}
	err_i += Ki*((position_err)*(time));	//��������
	err_d = Kd*((position_err-last_err)/(time));	//΢������
	pwm_out = Kp*(position_err+err_d+err_i);	//PID��ʽ
	last_err = position_err;	//�������
	if(pwm_out > 3500)
	{
		pwm_out = 3500;
	}
	else
	{
		if(pwm_out < (-3500))
		{
			pwm_out = (-3500);
		}
	}
	return pwm_out;
}
