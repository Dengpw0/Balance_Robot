#include "timer.h"
#include "led.h"
#include "usart.h"
#include "main.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "kalman.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//定时器PWM 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/4
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 
int Turn_PID(PidTypeDef *pid,int gyro_Z,int groz_med);
//外部变量 extern说明改变量已在其它文件定义
extern int   Encoder, CurrentPosition; //当前速度、当前位置
extern int   TargetVelocity, CurrentPosition, EncoderLeft,EncoderRight,PWM_left,PWM_right,Turn_out;//目标速度、目标圈数、编码器读数、PWM控制变量
extern int speedleft,speedright;
extern float targetspeed;	//速度环期望（直立时为0.行走时给值）
extern int groz_med;	//转向环期望（为0时走直线）
extern float pitch,roll,yaw,pitch_med,groy_med,pitch_kalman; 		//欧拉角							//定义四个实际参数	欧拉角	一个pitch中值，负责平衡
extern int pitchsee;
extern short gyrox,gyroy,gyroz;	//陀螺仪原始数据			//定义三个实际参数	陀螺仪角度
extern short aacx,aacy,aacz;		//加速度传感器原始数据	//定义三个实际参数	加速度
extern int Turn_Kp,gyroz_med,gyroz_set;
extern u16 flag;
extern PidTypeDef position;
extern PidTypeDef speed;
//TIM14 PWM部分初始化 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM2_PWM_Init(u32 arr,u32 psc)
{		 					 
	//此部分需手动修改IO口设置
//	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  	//TIM14时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_TIM2); //GPIOF9复用为定时器14
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM2); //GPIOF9复用为定时器14
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_4|GPIO_Pin_5;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_Init(GPIOA,&GPIO_InitStructure);              //初始化PF9
	
	GPIO_ResetBits(GPIOA, GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_4|GPIO_Pin_5); //初始化设置引脚低电平
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_0;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;        //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure);              //初始化PF9
	
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=0; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);//初始化定时器14
	
	//初始化TIM14 Channel1 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性低
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC4
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC4

	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);  //使能TIM14在CCR4上的预装载寄存器
  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);  //使能TIM14在CCR4上的预装载寄存器
  TIM_ARRPreloadConfig(TIM2,ENABLE);//ARPE使能 	//外设
	
	TIM_Cmd(TIM2, ENABLE);  //使能TIM14

}  
void TIM3_Encode_Init(void)
{		 					 
	GPIO_InitTypeDef GPIO_InitStructure; //定义一个引脚初始化的结构体  
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;//定义一个定时器初始化的结构体
  TIM_ICInitTypeDef TIM_ICInitStructure; //定义一个定时器编码器模式初始化的结构体
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //使能TIM5时钟
	RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //使能CPIOA时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	//TIM5_CH2、TIM5_CH3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);	//根据GPIO_InitStructure的参数初始化GPIO
	
	 GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3); //GPIOF9复用位定时器14
	 GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3); //GPIOF9复用位定时器14

	TIM_TimeBaseStructure.TIM_Period = 0xffff; //设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_Prescaler = 0; // 预分频器 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //选择时钟分频：不分频
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct的参数初始化定时器TIM4
	
  TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //使用编码器模式3：CH1、CH2同时计数，为四分频
  TIM_ICStructInit(&TIM_ICInitStructure); //把TIM_ICInitStruct 中的每一个参数按缺省值填入
	TIM_ICInitStructure.TIM_ICFilter = 10;  //设置滤波器长度
  TIM_ICInit(TIM3, &TIM_ICInitStructure); //根TIM_ICInitStructure参数初始化定时器TIM4编码器模式
	
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);//清除TIM的更新标志位
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); //更新中断使能
  TIM_SetCounter(TIM3,0); //初始化清空编码器数值
	
	TIM_Cmd(TIM3, ENABLE); //使能定时器4

}  
void TIM4_Encode_Init(void)
{		 					 
	GPIO_InitTypeDef GPIO_InitStructure; //定义一个引脚初始化的结构体  
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;//定义一个定时器初始化的结构体
  TIM_ICInitTypeDef TIM_ICInitStructure; //定义一个定时器编码器模式初始化的结构体
	
	RCC_APB1PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //使能TIM5时钟
	RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //使能CPIOA时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;	//TIM5_CH2、TIM5_CH3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);	//根据GPIO_InitStructure的参数初始化GPIO
	
	 GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_TIM1); //GPIOF9复用位定时器14
	 GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_TIM1); //GPIOF9复用位定时器14

	TIM_TimeBaseStructure.TIM_Period = 0xffff; //设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_Prescaler = 0; // 预分频器 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //选择时钟分频：不分频
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct的参数初始化定时器TIM4
	
  TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //使用编码器模式3：CH1、CH2同时计数，为四分频
  TIM_ICStructInit(&TIM_ICInitStructure); //把TIM_ICInitStruct 中的每一个参数按缺省值填入
	TIM_ICInitStructure.TIM_ICFilter = 10;  //设置滤波器长度
  TIM_ICInit(TIM1, &TIM_ICInitStructure); //根TIM_ICInitStructure参数初始化定时器TIM4编码器模式
	
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);//清除TIM的更新标志位
  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); //更新中断使能
  TIM_SetCounter(TIM1,0); //初始化清空编码器数值
	
	TIM_Cmd(TIM1, ENABLE); //使能定时器4

}  
/**************************************************************************
函数功能：读取TIM4编码器数值
入口参数：无
返回  值：无
**************************************************************************/
int Read_Encoder(int left_or_right)
{
	int Encoder_TIM;
	switch(left_or_right)
	{
		case 0:
			Encoder_TIM=TIM3->CNT; //读取计数
			if(Encoder_TIM>0xefff)Encoder_TIM=Encoder_TIM-0xffff; //转化计数值为有方向的值，大于0正转，小于0反转。
																														//TIM4->CNT范围为0-0xffff，初值为0。
			TIM3->CNT=0; //读取完后计数清零
			return Encoder_TIM; //返回值
		case 1:
			Encoder_TIM=TIM1->CNT; //读取计数
			if(Encoder_TIM>0xefff)Encoder_TIM=Encoder_TIM-0xffff; //转化计数值为有方向的值，大于0正转，小于0反转。
																														//TIM4->CNT范围为0-0xffff，初值为0。
			TIM1->CNT=0; //读取完后计数清零
			return Encoder_TIM; //返回值
	}
	return 0;
}
/**************************************************************************
函数功能：TIM4中断服务函数
入口参数：无
返回  值：无
**************************************************************************/
//void TIM3_IRQHandler(void)
//{ 		    		  			    
//	if(TIM3->SR&0X0001)//溢出中断
//	{    				   				     	    	
//	}				   
//	TIM3->SR&=~(1<<0);//清除中断标志位 	    
//}
//void TIM4_IRQHandler(void)
//{ 		    		  			    
//	if(TIM4->SR&0X0001)//溢出中断
//	{    				   				     	    	
//	}				   
//	TIM4->SR&=~(1<<0);//清除中断标志位 	    
//}
/**************************************************************************
函数功能：通用定时器2初始化函数，
入口参数：自动重装载值 预分频系数 默认定时时钟为72MHZ时，两者共同决定定时中断时间
返回  值：无
**************************************************************************/
void EncoderRead_TIM1(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue; //定义一个定时中断的结构体
	NVIC_InitTypeDef NVIC_InitStrue; //定义一个中断优先级初始化的结构体
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //使能通用定时器2时钟
	
	TIM_TimeBaseInitStrue.TIM_Period=arr; //计数模式为向上计数时，定时器从0开始计数，计数超过到arr时触发定时中断服务函数
	TIM_TimeBaseInitStrue.TIM_Prescaler=psc; //预分频系数，决定每一个计数的时长
	TIM_TimeBaseInitStrue.TIM_CounterMode=TIM_CounterMode_Up; //计数模式：向上计数
	TIM_TimeBaseInitStrue.TIM_ClockDivision=TIM_CKD_DIV1; //一般不使用，默认TIM_CKD_DIV1
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStrue); //根据TIM_TimeBaseInitStrue的参数初始化定时器TIM2
	
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); //使能TIM2中断，中断模式为更新中断：TIM_IT_Update
	
	NVIC_InitStrue.NVIC_IRQChannel=TIM4_IRQn; //属于TIM2中断
	NVIC_InitStrue.NVIC_IRQChannelCmd=ENABLE; //中断使能
	NVIC_InitStrue.NVIC_IRQChannelPreemptionPriority=1; //抢占优先级为1级，值越小优先级越高，0级优先级最高
	NVIC_InitStrue.NVIC_IRQChannelSubPriority=1; //响应优先级为1级，值越小优先级越高，0级优先级最高
	NVIC_Init(&NVIC_InitStrue); //根据NVIC_InitStrue的参数初始化VIC寄存器，设置TIM2中断
	
	TIM_Cmd(TIM4, ENABLE); //使能定时器TIM2
}
/**
* @name PidCalc_IMU
**/
void PidCalc_IMU(PidTypeDef *pid, float med, float pitch, float gyroy,float type)
{
	float PWM_out;
  PWM_out = pid->Kp*(pitch-med)+pid->Kd*(gyroy-groy_med);
	pid->output = PWM_out;
}
/**
* @name PidCalc_Encode
**/
void PidCalc_Encode(PidTypeDef *pid, float left, float right, float targetspeed, float type)
{
	static float PWM_out;
	static float Encoder_S;
	static float Encoder_Least,Encoder_Least_last;
	static float Err_Lowout,Err_Lowout_last;	//低通滤波
  static float a=0.7;
	// 1.计算速度偏差
  // 舍去误差--我的理解：能够让速度为"0"的角度，就是机械中值。
	Encoder_Least = (left+right) - targetspeed;
	// 2.对速度偏差进行低通滤波
  // low_out = (1-a)*Ek+a*low_out_last
  Err_Lowout = (1-a)*Encoder_Least + a*Encoder_Least_last; // 使得波形更加平滑，滤除高频干扰，放置速度突变
  Err_Lowout_last = Err_Lowout;   // 防止速度过大影响直立环的正常工作
  // 3.对速度偏差积分出位移
  Encoder_S+=Err_Lowout;
  // 4.积分限幅
  Encoder_S=Encoder_S>10000?10000:(Encoder_S<(-10000)?(-10000):Encoder_S);
  
  // 5.速度环控制输出
  PWM_out = pid->Kp*Err_Lowout+pid->Ki*Encoder_S;
  
  pid->output = PWM_out;
}
int Turn_PID(PidTypeDef *pid,int gyro_Z,int groz_med)
{
	 int PWM_out;
  
  PWM_out = Turn_Kp*(gyro_Z-groz_med);
  return PWM_out;
}
#define SPEED 0
#define POSITION 1
void TIM4_IRQHandler()
{
  if(TIM_GetITStatus(TIM4, TIM_IT_Update)==1) //当发生中断时状态寄存器(TIMx_SR)的bit0会被硬件置1
	{
	   EncoderLeft=Read_Encoder(0);   //读取当前编码器读数
		 EncoderRight=Read_Encoder(1);   //读取当前编码器读数
		 mpu_dmp_get_data(&pitch,&roll,&yaw);	//得到数据时，return 0，跳出循环，如果没得到数据，一直循环，直到得到dmp计算出的数据
     MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);  // 读取角速度
     MPU_Get_Accelerometer(&aacx,&aacy,&aacz); // 读取加速度
		 pitch_kalman=KalmanFilter(&speedK,pitch);//请使用滤波后速度反馈值
	   pitchsee = pitch_kalman * 100;
		 //编码器位置环
//		 motor[LEFT].position.PositionSet = 1;
//		 motor[RIGHT].position.PositionSet = 2;
//		
//		 motor[LEFT].position.PositionNow += EncoderLeft;
//		 motor[RIGHT].position.PositionNow += EncoderRight;
//		 
//		 PID_Calc(&motor[LEFT].position.position_pid,motor[LEFT].position.PositionNow,motor[LEFT].position.PositionSet);
//		 PID_Calc(&motor[RIGHT].position.position_pid,motor[RIGHT].position.PositionNow,motor[RIGHT].position.PositionSet);
//		 
//		 motor[LEFT].position.PWM = motor[LEFT].position.position_pid.output;
//		 motor[RIGHT].position.PWM = motor[RIGHT].position.position_pid.output;
		
		//编码器速度环
//		 motor[LEFT].speed.SpeedSet = TargetVelocity;//motor[LEFT].position.PWM/76;	//
//		 motor[RIGHT].speed.SpeedSet = TargetVelocity;//motor[RIGHT].position.PWM/76;	//
		
//		 motor[LEFT].speed.SpeedNow = EncoderLeft*1000*200*0.000120830;
//		 motor[RIGHT].speed.SpeedNow = EncoderRight*1000*200*0.000120830;
		 
//		 motor[LEFT].speed.seeSpeedSet =  motor[LEFT].speed.SpeedSet;
//		 motor[RIGHT].speed.seeSpeedSet =  motor[RIGHT].speed.SpeedSet;
	 
//		 PID_Calc(&motor[LEFT].speed.speed_pid,motor[LEFT].speed.SpeedNow,motor[LEFT].speed.SpeedSet);
//		 PID_Calc(&motor[RIGHT].speed.speed_pid,motor[RIGHT].speed.SpeedNow,motor[RIGHT].speed.SpeedSet);
		
//		 motor[LEFT].speed.PWM = motor[DOUBLE].speed.speed_pid.output;
//		 motor[RIGHT].speed.PWM = motor[LEFT].speed.PWM;
		
		
		//速度环
		 PidCalc_Encode(&motor[DOUBLE].speed.speed_pid,EncoderLeft,EncoderRight,targetspeed,ENCODE_SPEED);
		//直立环
		 PidCalc_IMU(&motor[LEFT].position.imu_pid,pitch_med+motor[DOUBLE].speed.speed_pid.output,pitch_kalman,gyroy,IMU_POSITION);
		 PidCalc_IMU(&motor[RIGHT].position.imu_pid,pitch_med+motor[DOUBLE].speed.speed_pid.output,pitch_kalman,gyroy,IMU_POSITION);
		 motor[LEFT].position.PWM = motor[LEFT].position.imu_pid.output;
		 motor[RIGHT].position.PWM = motor[RIGHT].position.imu_pid.output;
		 Turn_out=Turn_PID(&speed_diff[DOUBLE].diff_pid,gyroz,groz_med);
		 
//		 //差速补偿，	。。不能加差速，直立环很南条
//		 PID_Calc(&speed_diff[LEFT].diff_pid,motor[LEFT].position.PWM,motor[RIGHT].position.PWM);
//		 PID_Calc(&speed_diff[RIGHT].diff_pid,motor[RIGHT].position.PWM,motor[LEFT].position.PWM);
//		 speedleft = motor[LEFT].position.PWM+speed_diff[LEFT].diff_pid.output;
//		 speedright = motor[RIGHT].position.PWM+speed_diff[RIGHT].diff_pid.output;
		 speedleft = motor[LEFT].position.PWM-Turn_out;//负反馈	左-右正，令gyroz趋近0
		 speedright = motor[RIGHT].position.PWM+Turn_out;//
			 
#if (SPEED)
SetPWM(motor[LEFT].speed.PWM,motor[RIGHT].speed.PWM); //设置PWM
#endif
		
#if (POSITION)
//		SetPWM(motor[LEFT].position.PWM, motor[RIGHT].position.PWM); //设置PWM
SetPWM(speedleft, speedright); //设置PWM
#endif
			
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update); //状态寄存器(TIMx_SR)的bit0置0
	}
}
/**************************************************************************
函数功能：设置TIM3通道4PWM值
入口参数：PWM值
返回  值：无
**************************************************************************/
void SetPWM(int pwmleft,int pwmright)
{
  if(pwmleft>=0)//pwm>=0 (BIN1, BIN2)=(0, 1) 正转
  {
		PAout(5)=0; //BIN1=0
		PAout(4)=1; //BIN2=1
		TIM2->CCR4=pwmleft;
		TIM_SetCompare4(TIM2, pwmleft);
  }
  else if(pwmleft<0)//pwm<0 (BIN1, BIN2)=(1, 0) 反转
  {
		PAout(5)=1; //BIN1=1
		PAout(4)=0; //BIN2=0
		TIM2->CCR4=-pwmleft;
		TIM_SetCompare4(TIM2, -pwmleft);
  }
	if(pwmright>=0)//pwm>=0 (BIN1, BIN2)=(0, 1) 正转
  {
		PAout(2)=1; //BIN1=0
		PAout(1)=0; //BIN2=1
		TIM2->CCR1=pwmright;
		TIM_SetCompare1(TIM2, pwmright);
  }
  else if(pwmright<0)//pwm<0 (BIN1, BIN2)=(1, 0) 反转
  {
		PAout(2)=0; //BIN1=1
		PAout(1)=1; //BIN2=0
		TIM2->CCR1=-pwmright;
		TIM_SetCompare1(TIM2, -pwmright);
  }
}
