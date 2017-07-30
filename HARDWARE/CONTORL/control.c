#include "control.h"
//Right电机转速计算
//参数： time 函数的执行周期 单位ms 
void Get_RightMotorSpeed(float time)
{
	u16 n_cnt;
	u16 motor_cnt;
   u16 motor_maichong=65534;//电机码盘390个脉冲，4倍频1560个
	u16 n_motor_spd;
	static u16 last_cnt;
	n_cnt =TIM5->CNT;
	if((TIM5->CR1 &(1<<4)) == 0)	//正转
	{
		if(n_cnt>=last_cnt)	//正常递增情况下
		{
			motor_cnt = n_cnt-last_cnt;	//计算出脉冲增加的个数
			last_cnt = n_cnt;
		}
		else	//过峰
		{
			motor_cnt = motor_maichong - last_cnt + n_cnt;
			last_cnt = n_cnt; 
		}
	}
	else	//反转
	{
		if(n_cnt<=last_cnt)
		{
			motor_cnt = last_cnt - n_cnt;	//计算出增加的脉冲
			last_cnt = n_cnt; 	
		}
		else		//反转过峰
      {
			motor_cnt = motor_maichong - n_cnt + last_cnt;
			last_cnt = n_cnt; 
      }
	}
   RightMotor_Value.motor_change=motor_cnt;
	n_motor_spd =  (motor_cnt/time)*38;// ((脉冲数/1560)/time)*60000ms = r/min
   RightMotor_Value.motor_speed_nolb=n_motor_spd;
	RightMotor_Value.motor_speed = RightMotor_Value.motor_speed + 6.24f*(time/1000.0f)*5*(n_motor_spd - RightMotor_Value.motor_speed)	;//低通滤波
	//System_Value.motor_speed =n_motor_spd;
	//Task_Time.Duty_1ms = 0;//任务时间标志位清0
}

//Left电机转速计算
//参数： time 函数的执行周期 单位ms 
void Get_LeftMotorSpeed(float time)
{
	u16 n_cnt;
	u16 motor_cnt;
   u16 motor_maichong=1560;//电机码盘390个脉冲，4倍频1560个
	u16 n_motor_spd;
	static u16 last_cnt;
	n_cnt =TIM3->CNT;
	if((TIM3->CR1 &(1<<4)) == 0)	//正转
	{
		if(n_cnt>=last_cnt)	//正常递增情况下
		{
			motor_cnt = n_cnt-last_cnt;	//计算出脉冲增加的个数
			last_cnt = n_cnt;
		}
		else	//过峰
		{
			motor_cnt = motor_maichong - last_cnt + n_cnt;
			last_cnt = n_cnt; 
		}
	}
	else	//反转
	{
		if(n_cnt<=last_cnt)
		{
			motor_cnt = last_cnt - n_cnt;	//计算出增加的脉冲
			last_cnt = n_cnt; 	
		}
		else		//反转过峰
      {
			motor_cnt = motor_maichong - n_cnt + last_cnt;
			last_cnt = n_cnt; 
      }
	}
   LeftMotor_Value.motor_change=motor_cnt;
	n_motor_spd =  (motor_cnt/time)*36;// ((脉冲数/1660)/time)*60000ms = r/min
   LeftMotor_Value.motor_speed_nolb=n_motor_spd;
	LeftMotor_Value.motor_speed = LeftMotor_Value.motor_speed + 6.24f*(time/1000.0f)*5*(n_motor_spd - LeftMotor_Value.motor_speed)	;//低通滤波
	//System_Value.motor_speed =n_motor_spd;
	//Task_Time.Duty_1ms = 0;//任务时间标志位清0
}

/**************************************************************************
函数功能：单位时间读取编码器计数
入口参数：定时器
返回  值：速度值
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
函数功能：位置式PID控制器
入口参数：编码器测量位置信息，目标位置
返回  值：电机PWM
根据位置式离散PID公式 
pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  
∑e(k)代表e(k)以及之前的偏差的累积和;其中k为1,2,,k;
pwm代表输出
**************************************************************************/
int Position_PID (int Encoder,int Target)
{ 	
	 
   float Position_KP=150,Position_KI=0.01,Position_KD=1000;
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
	 Pwm=Position_KP*Bias+Position_KI*Integral_bias+Position_KD*(Bias-Last_Bias);       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;                                           //增量输出
}

/**************************************************************************
函数功能：速度PI控制 修改前进后退速度，请修Target_Velocity，比如，改成60就比较慢了
入口参数：左轮编码器变化、右轮编码器变化
返回  值：速度控制PWM
作    者：平衡小车之家
**************************************************************************/
int velocity(int encoder_left,int encoder_right)
{  
    static float Velocity,Encoder_Least,Encoder,Movement;
	  static float Encoder_Integral;
	  float kp=70,ki=6;
	  //=============速度PI控制器=======================//	
		Encoder_Least =(encoder_left+encoder_right)+20;                    //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零） 
		Encoder *= 0.8;		                                                //===一阶低通滤波器       
		Encoder += Encoder_Least*0.2;	                                    //===一阶低通滤波器    
		Encoder_Integral +=Encoder;                                       //===积分出位移 积分时间：10ms
		Encoder_Integral=Encoder_Integral-Movement;                       //===接收遥控器数据，控制前进后退
		if(Encoder_Integral>10000)  Encoder_Integral=10000;             //===积分限幅
		if(Encoder_Integral<-10000)	Encoder_Integral=-10000;              //===积分限幅	
		Velocity=Encoder*kp+Encoder_Integral*ki;                          //===速度控制	
		return Velocity;
}

/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI (int Encoder,int Target)
{ 	
   float Kp=100,Ki=20;	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //计算偏差
	 Pwm+=Kp*(Bias-Last_bias)+Ki*Bias;   //增量式PI控制器
	 Last_bias=Bias;	                   //保存上一次偏差 
	 return Pwm;                         //增量输出
}




//电机转速环
//参数：执行周期
//			PID对应参数
//			转速目标值
//返回：PWM的值
s16 MotorSpeed_Ctrl(float Kp,float Ki,float Kd,float speed_set,float time)
{
	s16 now_err;	//当前误差
	s16 pwm_out;	//PWM的输出值
	static s16 last_err;//上一时刻的误差
	static s16 err_d;	//误差微分 
	static s16 err_i;//误差积分
	now_err = speed_set - RightMotor_Value.motor_speed;//当前误差
	if((now_err<20)&&(now_err>(-20)))	//死区控制
	{
		now_err = 0;
	}
	err_i += Ki*((now_err)*(time));	//积分运算
	err_d = Kd*((now_err-last_err)/(time));	//微分运算
	pwm_out = Kp*(now_err+err_d+err_i);	//PID公式
	last_err = now_err;	//保存误差
	return pwm_out;
}

//摆杆控制
//参数：执行周期
//			PID对应参数
//			摆杆位置目标角度angle_set
//返回：PWM的值
s16 Pendulum_Ctrl(float Kp,float Ki,float Kd,float angle_set,float time)
{
	s16 angle_err;	//当前误差
	s16 pwm_out;	//PWM的输出值
	static s16 last_err;//上一时刻的误差
	static s16 err_d=0;	//误差微分 
	static s16 err_i=0;//误差积分
	float  now_angle;	//当前角度
	now_angle = TIM8->CNT  ;
	angle_err = now_angle - angle_set;//当前误差
	if((angle_err>-2)&&(angle_err<(2)))	//死区控制 正负5度
	{
		angle_err = 0;
	}
//	if((angle_err >250) || (angle_err<-250))	//失控
//	{
//		angle_err = 0;
//		TIM1->CCR1 = 1000;
//	}
	err_i += Ki*((angle_err)*(time));	//积分运算
	err_d = Kd*((angle_err-last_err)/(time));	//微分运算
	pwm_out = Kp*(angle_err+err_d+err_i);	//PID公式
	last_err = angle_err;	//保存误差
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


//电机位置环控制
//参数：执行周期
//			PID对应参数
//			目标位置
//返回：PWM的值
s16 MotorPosition_Ctrl(float Kp,float Ki,float Kd,float position_set,float time)
{
	s16 position_err;	//当前误差
	s16 pwm_out;	//PWM的输出值
	static s16 last_err;//上一时刻的误差
	static s16 err_d;	//误差微分 
	static s16 err_i;//误差积分
	s16  now_position;	//当前角度
	now_position = TIM5->CNT  ;
//	if(now_position > 4000)
//	{
//		now_position = (now_position%4000);
//	}
	position_err = position_set - now_position;//当前误差
	if(position_err <(-2200))
	{
		position_err = position_err + 3999;
	}
	if((position_err>-5)&&(position_err<(5)))	//死区控制 
	{
		position_err = 0;
	}
//	if((position_err >400) || (position_err<-400))	//失控
//	{
//		Motor_StartOrStop(0);
//	}
	err_i += Ki*((position_err)*(time));	//积分运算
	err_d = Kd*((position_err-last_err)/(time));	//微分运算
	pwm_out = Kp*(position_err+err_d+err_i);	//PID公式
	last_err = position_err;	//保存误差
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
