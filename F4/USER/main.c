#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "timer.h"
#include "SEGGER_RTT.h"
#include "main.h"

int   TargetVelocity=100, EncoderLeft,EncoderRight,PWM_left,PWM_right;  //目标速度、目标圈数(位置)、编码器读数、PWM控制变量
motor_control motor[2];
int Res;
u8 usart_value;
int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);  //初始化延时函数
	uart_init(9600);//初始化串口波特率为115200
	LED_Init();			     //LED端口初始化
	
	
 	TIM2_PWM_Init(8399,0);	//84M/84=1Mhz的计数频率,重装载值500，所以PWM频率为 1M/500=2Khz.     
  TIM3_Encode_Init();
	TIM4_Encode_Init();
	EncoderRead_TIM1(8399, 99);
	 PIDInit(&motor[LEFT].speed.speed_pid,60,0.1,0.5,0,6500, 0, 250, 40 * 19, 30 * 19 * 2, 1350, SPEED);
	 PIDInit(&motor[RIGHT].speed.speed_pid,60,0.1,0.5,0,6500, 0, 250, 40 * 19, 30 * 19 * 2, 1350, SPEED);
  while(1) //实现比较值从0-300递增，到300后从300-0递减，循环
	{
 		
	}
}
