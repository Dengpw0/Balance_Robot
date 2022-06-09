/***********************************************
公司：东莞市微宏智能科技有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com
版本V1.0
修改时间：2020-06-30
Allrightsreserved
***********************************************/



#include "delay.h"
#include "usart.h"			
#include "TB6612.h"
#include "motorencoder.h"
#include "usart.h"
#include "led.h"
#include "oled.h"
#include "key.h"
#include "SEGGER_RTT.h"
#include "pid.h"
#include "main.h"

int   TargetVelocity=100,  CurrentPosition, EncoderLeft,EncoderRight,PWM_left,PWM_right;  //目标速度、目标圈数(位置)、编码器读数、PWM控制变量

float TargetCircle=1;		//float类型转的花样多点，也不用去改那个脉冲

int   MortorRun = 0;  //允许电机控制标志位


float orignposition;

u16 flag = 0;

float angle;
int viewangle;

u16 times;

PidTypeDef position;
int seeAngleSet;
int seeAngleNow;
float AngleSet;
float AngleNow;
float AngleMax = 20;
float AngleMin = -20;
float k = 0.01;

PidTypeDef speed;
int seeSpeeSet;
int seeSpeedNow;
float SpeeSet;
float SpeedNow;

u8 key_value;
u8 usart_value;

motor_control motor[2];
/**************************************************************************
函数功能：主函数
入口参数：无
返回  值：无
**************************************************************************/
int main(void)
{
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //中断优先级分组
	 delay_init();                //延迟函数初始化
	 LED_Init();                  //LED灯初始化
	 KEY_Init();									//KEY按键初始化
   uart_init(9600);             //USART蓝牙串口初始化	 
   MotorEncoderLeft_Init();	    //编码器初始化 使用定时器4
	 MotorEncoderRight_Init();		//编码器初始化 使用定时器5
   TB6612_Init(7199, 0);        //电机驱动外设初始化 使用定时器3 
	 EncoderRead_TIM2(7199, 99);  //10ms读取一次编码器(即100HZ)，电机减速比为20，霍尔编码器精度13，AB双相组合得到4倍频，
	                              //则转1圈编码器读数为20*13*4=1040，电机转速=Encoder*100/1040r/s 使用定时器2
		PIDInit(&motor[LEFT].position.position_pid,420,0,600,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
	 PIDInit(&motor[RIGHT].position.position_pid,420,0,600,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
	 PIDInit(&motor[LEFT].speed.speed_pid,60,0.1,0.5,0,6500, 0, 250, 40 * 19, 30 * 19 * 2, 1350, SPEED);
	 PIDInit(&motor[RIGHT].speed.speed_pid,60,0.1,0.5,0,6500, 0, 250, 40 * 19, 30 * 19 * 2, 1350, SPEED);
	
	 delay_ms(200);              //延迟等待初始化完成
	 orignposition = TargetCircle;
//	 while(1)
		for(;;)
	  {
			key_value = KEY_Scan();
			
			if(key_value==1)
				LED2=1;
			if(key_value==2)
				LED2=0;
			
			switch(usart_value)
			{
				case 1:
					LED2=1;
					break;
				case 2:
					LED2=0;
					break;
				default:
					break;
			}
			//正常运行小灯
//			 GPIO_SetBits(GPIOD,GPIO_Pin_13);
			 seeAngleSet = AngleSet;
			 seeAngleNow = AngleNow;

			 
			 seeSpeeSet = SpeeSet;
			 seeSpeedNow = SpeedNow;

	  }
}

