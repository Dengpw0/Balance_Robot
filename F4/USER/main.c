#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "timer.h"
#include "SEGGER_RTT.h"
#include "main.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 

int   TargetVelocity=100, EncoderLeft,EncoderRight,PWM_left,PWM_right;  //目标速度、目标圈数(位置)、编码器读数、PWM控制变量
motor_control motor[2];

float pitch,roll,yaw; 		//欧拉角							//定义三个实际参数	欧拉角
short aacx,aacy,aacz;		//加速度传感器原始数据	//定义三个实际参数	加速度
short gyrox,gyroy,gyroz;	//陀螺仪原始数据			//定义三个实际参数	陀螺仪角度
float pitch_med = -7;			//陀螺仪中值

int Res;
u8 usart_value;
int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);  //初始化延时函数
	uart_init(9600);//初始化串口波特率为115200
	LED_Init();			     //LED端口初始化
	MPU_Init();					        //初始化MPU6050
  while(mpu_dmp_init()!=0){}	//初始化成功是0，初始化不成功就循环，直到初始化成功，平放初始化容易成功。
	TIM2_PWM_Init(8399,0);	//84M/84=1Mhz的计数频率,重装载值500，所以PWM频率为 1M/500=2Khz.     
  TIM3_Encode_Init();
	TIM4_Encode_Init();
	EncoderRead_TIM1(8399, 99);//此定时器影响iic时序
	PIDInit(&motor[LEFT].position.imu_pid,-800,0,-0,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
	PIDInit(&motor[RIGHT].position.imu_pid,-800,0,-0,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
 PIDInit(&motor[LEFT].speed.speed_pid,60,0.1,0.5,0,6500, 0, 250, 40 * 19, 30 * 19 * 2, 1350, SPEED);
 PIDInit(&motor[RIGHT].speed.speed_pid,60,0.1,0.5,0,6500, 0, 250, 40 * 19, 30 * 19 * 2, 1350, SPEED);
 PIDInit(&motor[LEFT].position.position_pid,420,0,600,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
 PIDInit(&motor[RIGHT].position.position_pid,420,0,600,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
  while(1) //实现比较值从0-300递增，到300后从300-0递减，循环
	{
 		
	}
}
