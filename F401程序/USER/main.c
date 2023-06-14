#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "kalman.h"
#include "main.h"
#include "time.h"
#include "pid.h"
#include "oled.h"
#include "usart_obser.h"

/************************************************************************************************MOTOR************************************************************************************************/
int TargetVelocity=100,EncoderLeft,EncoderRight,PWM_left,PWM_right,Turn_out;  //目标速度、目标圈数(位置)、编码器读数、PWM控制变量
motor_control motor[3];					//两个电机结构体
speed_difference speed_diff[3];	//差速结构体
int speedleft,speedright;				//左右电机速度输出

/************************************************************************************************MPU6050************************************************************************************************/
float pitch,roll,yaw,pitch_kalman,yaw_kalman; 	//欧拉角		
short aacx,aacy,aacz;								//加速度传感器原始数据
short gyrox,gyroy,gyroz;						//陀螺仪原始数据

/************************************************************************************************Banlance************************************************************************************************/
//四个平衡值
//由于每次上电，不同方向，机械pitch机械中值都不一样，所以上电时保持平衡，等待陀螺仪自己校准出机械中值0
float pitch_med = 0;			//pitch角机械中值
float groy_med = -6;			//pitch轴角速度中值
int groz_med = -39;				//z轴角速度中值，保证两轮直行
float targetspeed = 0;		//速度环期望（直立时为0.行走时给值）
float Yaw_target = 0;					//转向环速度期望
//pid调参系数
float Turn_Kp = 0;							//转向环Kp
float Turn_Kd = 0;
float kp,kd;
float kp_speed,ki_speed;
//陀螺仪与电机卡尔曼滤波结构体
extKalman_t MPU6050_Kalman;
extKalman_t MOTOR_Kalman;
float tq = 1,tr = 1.5;		//陀螺仪卡尔曼滤波系数
float tq_m = 1,tr_m = 50;	//电机编码器卡尔曼滤波系数
//卡尔曼滤波后的值
short gyroy_kal;
short gyroz_kal;
int EncoderLeft_kal;
int EncoderRight_kal;
//定时器计时
int times;						//定时器4，0.1ms进入一次，进入后+1

/**
* @name TASK_Move
* @brief	移动控制
**/
void TASK_Move(void)
{
	task_flag_last = task_flag;	 //记录当前任务状态
	switch(task_flag)
	{
		case 0:
			times = 0;							 //清零计时
			break;
		case 1:
			targetspeed = 80;				//前进
			if(times >= 500)				//5s后回到平衡
				targetspeed = 0;			//平衡
			if(times >= 600)				//1s调整后转向
				Yaw_target = 88;			//左转
			if(times >= 900)				//3s调整后进入取货阶段
			{
				task_flag = 2;				//进入取货阶段
				times = 0;						//再次清零计时
			}
			break;
		case 2:
			if(times >= 500)				//3s取货
				task_flag = 3;				//取货完成进入送货阶段
			break;
		case 3:
			Yaw_target = 0;		      //右转
			if(times >= 700)				//2s右转后前进
				targetspeed = 65;			//前进
			if(times >= 1000)				//3s前进后停下
				targetspeed = 0;			//平衡
			if(times >= 1100)				//调整1秒
				Yaw_target = 88;			//左转
			if(times >= 1200)				//调整1秒
				task_flag = 4;				//进入卸货状态，完成任务
			break;
		case 4:
			if(times >= 1400)				//卸货2秒，保证OLED显示2s
				task_flag = 0;				//清楚flag
			break;
		default:
			break;
	}
}	

/************************************************************************************************OLED************************************************************************************************/
//陀螺仪数据显示
unsigned char mpu_error[] = "mpu_error!";
unsigned char mpu_ok[] = "mpu_ok!";
unsigned char pitchangle[] = "pitch:";
unsigned char gyroyspeed[] = "gyroy:";
unsigned char gyrozspeed[] = "gyroz:";
unsigned char task_getgoods[] = "";
//任务显示
int i,x;				//for循环所用变量
/**
* @name OLED_Display
* @brief OLED显示切换
**/
void OLED_Display(void)
{
	if(task_flag != task_flag_last)	//当任务状态改变时
			OLED_Clear();								//清屏上次内容
	if(!task_flag)//无任务时显示当前陀螺仪数据
	{
		OLED_ShowString(0,0,pitchangle,16);
		OLED_ShowString(0,18,gyroyspeed,16);
		OLED_ShowString(0,36,gyrozspeed,16);
		OLED_Float(60,0,pitch,3);
		OLED_Float(60,18,gyroy_kal,3);
		OLED_Float(60,36,gyroz_kal,3);
	  OLED_Refresh();
	}
	else					//收到串口发来的指令显示任务信息
	{
		switch(task_flag)
		{
			case 1:		//串口得到，开始任务
				//正在前往取货
				for(i = 0,x=8;i < 6;i++)
				{
						OLED_ShowChinese(x,25,i,16);
						x += 20;
				}
				break;
			case 2:		//完成第一个步骤后得到
				//正在取货
				for(i = 6,x=28;i < 10;i++)
				{
						OLED_ShowChinese(x,25,i,16);
						x += 20;
				}
				break;
			case 3:		//完成第二步得到
				//正在前往送货
				for(i = 10,x=8;i < 16;i++)
				{
						OLED_ShowChinese(x,25,i,16);
						x += 20;
				}
				break;
			case 4:		//完成第三步得到
				//搬运完成
				for(i = 16,x=28;i < 20;i++)
				{
						OLED_ShowChinese(x,25,i,16);
						x += 20;
				}
				break;
			default:
				break;
		}
		OLED_Refresh();
	}
}

//USART Flag
int task_flag = 0;
int task_flag_last;
/**
* @name PID_Init
* @brief PID参数初始化
**/
void PID_Init(void)
{
		/*效果可以*/
//	Turn_Kp=1;0
//	kp_speed = 0.00015;
//	ki_speed = 0.00001;
//	kp = 1850;
//  kd = 3.2;
	/*最终效果参数*/
//	直立环
//	kp = 2400;
//  kd = 8.6;
//	转向环
//	Turn_Kp=5;
//	Turn_Kd=0.3;
//	速度环
//	kp_speed = 80;
//	ki_speed = 0.09;
  //直立环（陀螺仪）
	kp = 2400;
  kd = 8.6;
  PIDInit(&motor[LEFT].position.imu_pid,kp,0,kd);
  PIDInit(&motor[RIGHT].position.imu_pid,kp,0,kd);
  //pd转向环（陀螺仪）
	Turn_Kp=5;
	Turn_Kd=0.3;
  //pid速度环（编码器）
	kp_speed = 80;
	ki_speed = 0.09;
	//pid位置环（编码器）
  PIDInit(&motor[DOUBLE].speed.speed_pid,kp_speed,ki_speed,0);
	//初始化卡尔曼滤波
	KalmanCreate(&MPU6050_Kalman,tq,tr);
	KalmanCreate(&MOTOR_Kalman,tq_m,tr_m);
}

/**
* @name TASK_Init
* @brief 任务初始化
**/
void TASK_Init(void)
{
	PID_Init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);		  //初始化延时函数
	uart_init(9600);      //初始化串口波特率为9600，用于蓝牙通信
	LED_Init();		        //初始化LED端口，用于测试
	OLED_Init();					//初始化OLED显示屏
	OLED_ColorTurn(0);    //0正常显示，1 反色显示
  OLED_DisplayTurn(0);  //0正常显示 1 屏幕翻转显示
  OLED_Refresh();				//刷新屏幕以显示
	MPU_IIC_Init();       //初始化IIC总线
	MPU_Init();					  //初始化MPU6050
  while(mpu_dmp_init()!=0){//陀螺仪初始化
	  OLED_ShowString(30,18,mpu_error,16);
		OLED_Refresh();     //初始化失败，OLED显示 mpu_error
	}
	OLED_Clear();
	//初始化成功OLED显示mpu_ok
	OLED_ShowString(40,0,mpu_ok,16);
	OLED_Refresh();
	TIM1_Encode_Init();		//初始化定时器1编码器捕获
	TIM3_Encode_Init();		//初始化定时器3编码器捕获
	TIM2_PWM_Init(8399,0);//初始化定时器2PWM输出
	TICK_TIM5(8399,99);		//10ms定时器，用于滴答时间
	Control_TIM4(839, 9); //初始化定时器4用于数据处理，0.1ms进入一次中断
}
int main(void)
{ 
	TASK_Init();
//	delay_ms(3000);
	OLED_Clear();//清屏上次内容
	LED = 0;
	while(1)
	{	oscillographyData[0] = EncoderLeft;
		oscillographyData[1] = EncoderLeft_kal;

		oscillographyData[2] = pitch;
		oscillographyData[3] = roll;
		oscillographyData[4] = yaw;
		oscillographyData[5] = gyroy_kal;
		oscillographyData[6] = gyroz_kal;
		oscillographyData[7] = 0;
		oscillographyData[8] = 0;
		oscillographyData[9] = 0;
		oscillography_sendData(oscillographyData, 10);
//		printf("task_flag:%d",task_flag);
		OLED_Display();	//OLED根据标志位显示不同内容
		TASK_Move();		//任务步骤执行
	}
}
