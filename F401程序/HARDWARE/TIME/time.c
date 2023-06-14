#include "time.h"
#include "led.h"
#include "usart.h"
#include "main.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "kalman.h"



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
void TIM1_Encode_Init(void)
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
函数功能：通用定时器4初始化函数，
入口参数：自动重装载值 预分频系数 默认定时时钟为72MHZ时，两者共同决定定时中断时间
返回  值：无
**************************************************************************/
void Control_TIM4(u16 arr, u16 psc)
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
/**************************************************************************
函数功能：通用定时器5初始化函数，
入口参数：自动重装载值 预分频系数 默认定时时钟为72MHZ时，两者共同决定定时中断时间
返回  值：无
**************************************************************************/
void TICK_TIM5(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue; //定义一个定时中断的结构体
	NVIC_InitTypeDef NVIC_InitStrue; //定义一个中断优先级初始化的结构体
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); //使能通用定时器2时钟
	
	TIM_TimeBaseInitStrue.TIM_Period=arr; //计数模式为向上计数时，定时器从0开始计数，计数超过到arr时触发定时中断服务函数
	TIM_TimeBaseInitStrue.TIM_Prescaler=psc; //预分频系数，决定每一个计数的时长
	TIM_TimeBaseInitStrue.TIM_CounterMode=TIM_CounterMode_Up; //计数模式：向上计数
	TIM_TimeBaseInitStrue.TIM_ClockDivision=TIM_CKD_DIV1; //一般不使用，默认TIM_CKD_DIV1
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStrue); //根据TIM_TimeBaseInitStrue的参数初始化定时器TIM2
	
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE); //使能TIM2中断，中断模式为更新中断：TIM_IT_Update
	
	NVIC_InitStrue.NVIC_IRQChannel=TIM5_IRQn; //属于TIM2中断
	NVIC_InitStrue.NVIC_IRQChannelCmd=ENABLE; //中断使能
	NVIC_InitStrue.NVIC_IRQChannelPreemptionPriority=1; //抢占优先级为1级，值越小优先级越高，0级优先级最高
	NVIC_InitStrue.NVIC_IRQChannelSubPriority=1; //响应优先级为1级，值越小优先级越高，0级优先级最高
	NVIC_Init(&NVIC_InitStrue); //根据NVIC_InitStrue的参数初始化VIC寄存器，设置TIM2中断
	
	TIM_Cmd(TIM5, ENABLE); //使能定时器TIM2
}
void TIM5_IRQHandler()
{
	if(TIM_GetITStatus(TIM5, TIM_IT_Update)==1) 
	{//TIM5滴答计时
		times++;
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	}
}
#define SPEED 0
#define POSITION 1
void TIM4_IRQHandler()
{
  if(TIM_GetITStatus(TIM4, TIM_IT_Update)==1) //当发生中断时状态寄存器(TIMx_SR)的bit0会被硬件置1
	{
	  EncoderLeft=Read_Encoder(0);   //读取当前编码器读数
		EncoderRight=Read_Encoder(1);   //读取当前编码器读数
		EncoderLeft_kal = KalmanFilter(&MOTOR_Kalman,EncoderLeft);
		EncoderRight_kal = KalmanFilter(&MOTOR_Kalman,EncoderRight);
		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
		{
		  mpu_dmp_get_data(&pitch,&roll,&yaw);			// 获取三轴角度
      MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);  // 读取角速度
			gyroy_kal = KalmanFilter(&MPU6050_Kalman,gyroy);
			gyroz_kal = KalmanFilter(&MPU6050_Kalman,gyroz);
      MPU_Get_Accelerometer(&aacx,&aacy,&aacz); // 读取加速度
		}
		 //编码器位置环
//		 motor[LEFT].position.PositionSet = 1;
//		 motor[RIGHT].position.PositionSet = 2;
//		
//		 motor[LEFT].position.PositionNow += EncoderLeft;
//		 motor[RIGHT].position.PositionNow += EncoderRight;
//		 
//		 PID_Calc(&motor[LEFT].speed.speed_pid,motor[LEFT].speed.SpeedNow,motor[LEFT].speed.SpeedSet);
//		 PID_Calc(&motor[RIGHT].speed.speed_pid,motor[RIGHT].speed.SpeedNow,motor[RIGHT].speed.SpeedSet);
//		 PID_Calc(&motor[LEFT].position.position_pid,motor[LEFT].position.PositionNow,motor[LEFT].position.PositionSet);
//		 PID_Calc(&motor[RIGHT].position.position_pid,motor[RIGHT].position.PositionNow,motor[RIGHT].position.PositionSet);
//		 
//		 motor[LEFT].position.PWM = motor[LEFT].position.position_pid.output;
//		 motor[RIGHT].position.PWM = motor[RIGHT].position.position_pid.output;
		//编码器速度环
//		 motor[LEFT].speed.SpeedSet = TargetVelocity;//motor[LEFT].position.PWM/76;	//
//		 motor[RIGHT].speed.SpeedSet = TargetVelocity;//motor[RIGHT].position.PWM/76;	//
		
/***********Balance***********/
		//速度环
		PidCalc_Encode(&motor[DOUBLE].speed.speed_pid,EncoderLeft_kal,EncoderRight_kal,targetspeed,ENCODE_SPEED);
		motor[DOUBLE].speed.PWM = motor[DOUBLE].speed.speed_pid.output;
		//直立环
		PidCalc_IMU(&motor[LEFT].position.imu_pid,pitch_med,pitch,gyroy_kal,IMU_POSITION);
		PidCalc_IMU(&motor[RIGHT].position.imu_pid,pitch_med,pitch,gyroy_kal,IMU_POSITION);
		motor[LEFT].position.PWM = motor[LEFT].position.imu_pid.output;
		motor[RIGHT].position.PWM = motor[LEFT].position.PWM;
		//转向环
		Turn_out=Turn_PID(gyroz_kal,yaw);
		//三环融合
		speedleft = motor[LEFT].position.PWM+motor[DOUBLE].speed.PWM+Turn_out;//负反馈	左-右正，令gyroz趋近0
		speedright = motor[RIGHT].position.PWM+motor[DOUBLE].speed.PWM-Turn_out;
		//PWM输出限幅
		if(speedleft>7000)speedleft=7000;//最大7200
		if(speedright<-7000)speedright=-7000;
		if(speedright>7000)speedright=7000;//最大7200
		if(speedleft<-7000)speedleft=-7000;
			 
#if (SPEED)
//SetPWM(targetspeed,targetspeed); //设置PWM
//SetPWM(motor[DOUBLE].speed.speed_pid.output,motor[DOUBLE].speed.speed_pid.output); //设置PWM
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
