#include "motorencoder.h" 
#include "pid.h"
#include "led.h"
#include "main.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 

//外部变量 extern说明改变量已在其它文件定义
extern int   Encoder, CurrentPosition; //当前速度、当前位置
extern int   TargetVelocity, CurrentPosition, EncoderLeft,EncoderRight,PWM_left,PWM_right;//目标速度、目标圈数、编码器读数、PWM控制变量
extern float TargetCircle,AngleNow,AngleSet,SpeeSet,SpeedNow,AngleMax,k,AngleMin;
extern float Velcity_Kp,  Velcity_Ki,  Velcity_Kd; //相关速度PID参数
extern float Position_Kp, Position_Ki, Position_Kd; //相关位置PID参数
extern int   MortorRun;  //允许电机控制标志位

extern float pitch,roll,yaw,pitch_med; 		//欧拉角							//定义四个实际参数	欧拉角	一个pitch中值，负责平衡
extern short gyrox,gyroy,gyroz;	//陀螺仪原始数据			//定义三个实际参数	陀螺仪角度
extern short aacx,aacy,aacz;		//加速度传感器原始数据	//定义三个实际参数	加速度

extern u16 flag;
extern PidTypeDef position;
extern PidTypeDef speed;

extern u16 times;
/**************************************************************************
函数功能：编码器初始化函数
入口参数：无
返回  值：无
**************************************************************************/
void MotorEncoderLeft_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; //定义一个引脚初始化的结构体  
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;//定义一个定时器初始化的结构体
  TIM_ICInitTypeDef TIM_ICInitStructure; //定义一个定时器编码器模式初始化的结构体
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //使能TIM4时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能CPIOB时钟
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	//TIM4_CH1、TIM4_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);	//根据GPIO_InitStructure的参数初始化GPIO

	TIM_TimeBaseStructure.TIM_Period = 0xffff; //设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_Prescaler = 0; // 预分频器 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //选择时钟分频：不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct的参数初始化定时器TIM4
	
  TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //使用编码器模式3：CH1、CH2同时计数，为四分频
  TIM_ICStructInit(&TIM_ICInitStructure); //把TIM_ICInitStruct 中的每一个参数按缺省值填入
	TIM_ICInitStructure.TIM_ICFilter = 10;  //设置滤波器长度
  TIM_ICInit(TIM4, &TIM_ICInitStructure); //根TIM_ICInitStructure参数初始化定时器TIM4编码器模式
	
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);//清除TIM的更新标志位
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); //更新中断使能
  TIM_SetCounter(TIM4,0); //初始化清空编码器数值
	
	TIM_Cmd(TIM4, ENABLE); //使能定时器4
}
/**************************************************************************
函数功能：编码器初始化函数
入口参数：无
返回  值：无
**************************************************************************/
void MotorEncoderRight_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; //定义一个引脚初始化的结构体  
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;//定义一个定时器初始化的结构体
  TIM_ICInitTypeDef TIM_ICInitStructure; //定义一个定时器编码器模式初始化的结构体
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); //使能TIM5时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //使能CPIOA时钟
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_0;	//TIM5_CH2、TIM5_CH3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);	//根据GPIO_InitStructure的参数初始化GPIO

	TIM_TimeBaseStructure.TIM_Period = 0xffff; //设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_Prescaler = 0; // 预分频器 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //选择时钟分频：不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct的参数初始化定时器TIM4
	
  TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //使用编码器模式3：CH1、CH2同时计数，为四分频
  TIM_ICStructInit(&TIM_ICInitStructure); //把TIM_ICInitStruct 中的每一个参数按缺省值填入
	TIM_ICInitStructure.TIM_ICFilter = 10;  //设置滤波器长度
  TIM_ICInit(TIM5, &TIM_ICInitStructure); //根TIM_ICInitStructure参数初始化定时器TIM4编码器模式
	
	TIM_ClearFlag(TIM5, TIM_FLAG_Update);//清除TIM的更新标志位
  TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE); //更新中断使能
  TIM_SetCounter(TIM5,0); //初始化清空编码器数值
	
	TIM_Cmd(TIM5, ENABLE); //使能定时器4
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
		case 1:
			Encoder_TIM=TIM4->CNT; //读取计数
			if(Encoder_TIM>0xefff)Encoder_TIM=Encoder_TIM-0xffff; //转化计数值为有方向的值，大于0正转，小于0反转。
																														//TIM4->CNT范围为0-0xffff，初值为0。
			TIM4->CNT=0; //读取完后计数清零
			return Encoder_TIM; //返回值
		case 0:
			Encoder_TIM=TIM5->CNT; //读取计数
			if(Encoder_TIM>0xefff)Encoder_TIM=Encoder_TIM-0xffff; //转化计数值为有方向的值，大于0正转，小于0反转。
																														//TIM4->CNT范围为0-0xffff，初值为0。
			TIM5->CNT=0; //读取完后计数清零
			return Encoder_TIM; //返回值
	}
	return 0;
}

/**************************************************************************
函数功能：TIM4中断服务函数
入口参数：无
返回  值：无
**************************************************************************/
void TIM4_IRQHandler(void)
{ 		    		  			    
	if(TIM4->SR&0X0001)//溢出中断
	{    				   				     	    	
	}				   
	TIM4->SR&=~(1<<0);//清除中断标志位 	    
}

/**************************************************************************
函数功能：通用定时器2初始化函数，
入口参数：自动重装载值 预分频系数 默认定时时钟为72MHZ时，两者共同决定定时中断时间
返回  值：无
**************************************************************************/
void EncoderRead_TIM2(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue; //定义一个定时中断的结构体
	NVIC_InitTypeDef NVIC_InitStrue; //定义一个中断优先级初始化的结构体
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //使能通用定时器2时钟
	
	TIM_TimeBaseInitStrue.TIM_Period=arr; //计数模式为向上计数时，定时器从0开始计数，计数超过到arr时触发定时中断服务函数
	TIM_TimeBaseInitStrue.TIM_Prescaler=psc; //预分频系数，决定每一个计数的时长
	TIM_TimeBaseInitStrue.TIM_CounterMode=TIM_CounterMode_Up; //计数模式：向上计数
	TIM_TimeBaseInitStrue.TIM_ClockDivision=TIM_CKD_DIV1; //一般不使用，默认TIM_CKD_DIV1
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStrue); //根据TIM_TimeBaseInitStrue的参数初始化定时器TIM2
	
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); //使能TIM2中断，中断模式为更新中断：TIM_IT_Update
	
	NVIC_InitStrue.NVIC_IRQChannel=TIM2_IRQn; //属于TIM2中断
	NVIC_InitStrue.NVIC_IRQChannelCmd=ENABLE; //中断使能
	NVIC_InitStrue.NVIC_IRQChannelPreemptionPriority=1; //抢占优先级为1级，值越小优先级越高，0级优先级最高
	NVIC_InitStrue.NVIC_IRQChannelSubPriority=1; //响应优先级为1级，值越小优先级越高，0级优先级最高
	NVIC_Init(&NVIC_InitStrue); //根据NVIC_InitStrue的参数初始化VIC寄存器，设置TIM2中断
	
	TIM_Cmd(TIM2, ENABLE); //使能定时器TIM2
}

/**
* @name PidCalc
**/
void PidCalc(PidTypeDef *pid, float pitch, float gyroy)
{
	float PWM_out;
  
  PWM_out = pid->Kp*(pitch-pitch_med)+pid->Kd*(gyroy-0);
  
	pid->output = PWM_out;
}


/**************************************************************************
函数功能：TIM2中断服务函数 定时读取编码器数值并进行位置闭环控制 10ms进入一次
入口参数：无
返回  值：无
**************************************************************************/
#define SPEED 0
#define POSITION 1
void TIM2_IRQHandler()
{
  if(TIM_GetITStatus(TIM2, TIM_IT_Update)==1) //当发生中断时状态寄存器(TIMx_SR)的bit0会被硬件置1
	{
	   EncoderLeft=Read_Encoder(1);   //读取当前编码器读数
		 EncoderRight=Read_Encoder(0);   //读取当前编码器读数
		
		  mpu_dmp_get_data(&pitch,&roll,&yaw);	//得到数据时，return 0，跳出循环，如果没得到数据，一直循环，直到得到dmp计算出的数据
      MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);  // 读取角速度
      MPU_Get_Accelerometer(&aacx,&aacy,&aacz); // 读取加速度
		//正常运行闪灯
		times ++;
		if((times - 30)>0)
		{
			times -= 30;
			LED1=!LED1;
		}
//		 //位置环
//		 motor[LEFT].position.PositionSet = pitch*1560*1.04/2;
//		 motor[RIGHT].position.PositionSet = pitch*1560*1.04/2;
//		
//		 motor[LEFT].position.PositionNow += EncoderLeft;
//		 motor[RIGHT].position.PositionNow += EncoderRight;
//		 
//		 
//		 PID_Calc(&motor[LEFT].position.position_pid,motor[LEFT].position.PositionNow,motor[LEFT].position.PositionSet);
//		 PID_Calc(&motor[RIGHT].position.position_pid,motor[RIGHT].position.PositionNow,motor[RIGHT].position.PositionSet);
//		 
//		 motor[LEFT].position.PWM = motor[LEFT].position.position_pid.output;
//		 motor[RIGHT].position.PWM = motor[RIGHT].position.position_pid.output;
//		
//		
//		
//		//速度环
//		 motor[LEFT].speed.SpeedSet = motor[LEFT].position.PWM/76;	//TargetVelocity;//
//		 motor[RIGHT].speed.SpeedSet = motor[RIGHT].position.PWM/76;	//TargetVelocity;//
//		
//		 motor[LEFT].speed.SpeedNow = EncoderLeft*1000*200*0.000120830;
//		 motor[RIGHT].speed.SpeedNow = EncoderRight*1000*200*0.000120830;
		 
//		 motor[LEFT].speed.seeSpeedSet =  motor[LEFT].speed.SpeedSet;
//		 motor[RIGHT].speed.seeSpeedSet =  motor[RIGHT].speed.SpeedSet;
		 
//		 PID_Calc(&motor[LEFT].speed.speed_pid,motor[LEFT].speed.SpeedNow,motor[LEFT].speed.SpeedSet);
//		 PID_Calc(&motor[RIGHT].speed.speed_pid,motor[RIGHT].speed.SpeedNow,motor[RIGHT].speed.SpeedSet);
//		 
//		 motor[LEFT].speed.PWM = motor[LEFT].speed.speed_pid.output;
//		 motor[RIGHT].speed.PWM = motor[RIGHT].speed.speed_pid.output;
		 
		 PidCalc(&motor[LEFT].position.imu_pid,pitch,gyroy);
		 PidCalc(&motor[RIGHT].position.imu_pid,pitch,gyroy);
		 motor[LEFT].position.PWM = motor[LEFT].position.imu_pid.output;
		 motor[RIGHT].position.PWM = motor[RIGHT].position.imu_pid.output;
		 
#if (SPEED==1)
		SetPWM(motor[LEFT].speed.PWM, motor[RIGHT].speed.PWM); //设置PWM
#endif
		
#if (POSITION==1)
		SetPWM(motor[LEFT].position.PWM, motor[RIGHT].position.PWM); //设置PWM
#endif
			
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update); //状态寄存器(TIMx_SR)的bit0置0
	}
}

/**************************************************************************
函数功能：速度闭环PID控制(实际为PI控制)
入口参数：目标速度 当前速度
返回  值：速度控制值
根据增量式离散PID公式 
ControlVelocity+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
ControlVelocity代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
ControlVelocity+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
//int Velocity_FeedbackControl(int TargetVelocity, int CurrentVelocity)
//{
//		int Bias;  //定义相关变量
//		static int ControlVelocity, Last_bias; //静态变量，函数调用结束后其值依然存在
//		
//		Bias=TargetVelocity-CurrentVelocity; //求速度偏差
//		
//		ControlVelocity+=Velcity_Kp*(Bias-Last_bias)+Velcity_Ki*Bias;  //增量式PI控制器
//                                                                   //Velcity_Kp*(Bias-Last_bias) 作用为限制加速度
//	                                                                 //Velcity_Ki*Bias             速度控制值由Bias不断积分得到 偏差越大加速度越大
//		Last_bias=Bias;	
//		return ControlVelocity; //返回速度控制值
//}

/**************************************************************************
函数功能：位置式PID控制器
入口参数：目标圈数(位置) 当前位置
返回  值：速度控制值
根据位置式离散PID公式 
pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  
∑e(k)代表e(k)以及之前的偏差的累积和;其中k为1,2,,k;
pwm代表输出
**************************************************************************/
//int Position_FeedbackControl(float Circle, int CurrentPosition)
//{
//		float TargetPosition,Bias, ControlVelocity;     //定义相关变量
//		static float Last_bias, Integral_Bias;          //静态变量，函数调用结束后其值依然存在
//		
//	  TargetPosition=Circle*1560*1.04; //目标位置=目标圈数*1040
//	                                   //10ms读取一次编码器(即100HZ)，电机减速比为20，霍尔编码器精度13，AB双相组合得到4倍频，
//	                                   //则转1圈编码器读数为20*13*4=1040，电机转速=Encoder*100/1040r/s 使用定时器2
//	                                   //1.04是误差系数，电机本身存在误差，可根据实际情况调整该系数以提高控制精度
//		Bias=TargetPosition-CurrentPosition; //求位置偏差
//	  Integral_Bias+=Bias;
//    if(Integral_Bias> 970) Integral_Bias= 970;	//积分限幅 防止到达目标位置后过冲
//	  if(Integral_Bias<-970) Integral_Bias=-970;	//积分限幅 防止到达目标位置后过冲
//	
//		ControlVelocity=Position_Kp*Bias+Position_Ki*Integral_Bias+Position_Kd*(Bias-Last_bias);  //增量式PI控制器
//	                                                                                            //Position_Kp*Bias 偏差越大速度越大
//	                                                                                            //Position_Ki*Integral_Bias 减小稳态误差
//	                                                                                            //Position_Kd*(Bias-Last_bias) 限制速度

//		Last_bias=Bias;	
//		return ControlVelocity;    //返回速度控制值 
//}


