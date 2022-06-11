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
#include "sys.h"
#include "usart.h"			
#include "TB6612.h"
#include "motorencoder.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "SEGGER_RTT.h"
#include "pid.h"
#include "main.h"
#include "oled.h"
#include "bmp.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 

int   TargetVelocity=100, EncoderLeft,EncoderRight,PWM_left,PWM_right;  //目标速度、目标圈数(位置)、编码器读数、PWM控制变量

float TargetCircle=1;		//float类型转的花样多点，也不用去改那个脉冲

int   MortorRun = 0;  //允许电机控制标志位


float orignposition;

u16 times;

u8 key_value;
u8 usart_value;

motor_control motor[2];

float pitch,roll,yaw; 		//欧拉角							//定义三个实际参数	欧拉角
short aacx,aacy,aacz;		//加速度传感器原始数据	//定义三个实际参数	加速度
short gyrox,gyroy,gyroz;	//陀螺仪原始数据			//定义三个实际参数	陀螺仪角度
//short temp;					//温度	    

unsigned char logo[16] = "*Balance_Robot*";
/***
* bref @name KEY_Porc
***/
void KEY_Porc(void)
{
	key_value = KEY_Scan();
			
	if(key_value==1)
		LED2=1;
	if(key_value==2)
		LED2=0;
}
/***
* bref @name BLUE_Porc
***/
void BLUE_Porc(void)
{
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
}
/***
* bref @name Adjustpara_Porc
***/
void Adjustpara_Porc(void)
{
	
}
/***
* bref @name IMUGetData_Porc
***/
void IMUGetData_Porc(void)
{
	while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}	//得到数据时，return 0，跳出循环，如果没得到数据，一直循环，直到得到dmp计算出的数据
	//temp=MPU_Get_Temperature();	//得到温度值，芯片温度
	MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据，获取三个实际参数的地址，传入函数
	MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
	//mpu6050_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//用自定义帧发送加速度和陀螺仪原始数据
	//usart1_report_imu(aacx,aacy,aacz,gyrox,gyroy,gyroz,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));	//发送DMP运算之后的数据，上位机发送			
	printf("roll:%d pitch: %d yaw:%d\r\n",(int)(roll*100),(int)(pitch*100),(int)(yaw*10));	//串口发送
}
/***
* bref @name TASK_Init
***/
void TASK_Init(void)
{
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //中断优先级分组
	 delay_init();                //延迟函数初始化
	 OLED_Init();									//显示屏初始化
	 LED_Init();                  //LED灯初始化
	 KEY_Init();									//KEY按键初始化
   uart_init(9600);             //USART蓝牙串口初始化	 
   MotorEncoderLeft_Init();	    //编码器初始化 使用定时器4
	 MotorEncoderRight_Init();		//编码器初始化 使用定时器5
   TB6612_Init(7199, 0);        //电机驱动外设初始化 使用定时器3 
	 EncoderRead_TIM2(7199, 99);  //10ms读取一次编码器(即100HZ)，电机减速比为20，霍尔编码器精度13，AB双相组合得到4倍频，
	                              //则转1圈编码器读数为20*13*4=1040，电机转速=Encoder*100/1040r/s 使用定时器2
	 OLED_ColorTurn(0);           //0正常显示，1 反色显示
   OLED_DisplayTurn(0);         //0正常显示 1 屏幕翻转显示
	 OLED_Refresh();
	 MPU_Init();					        //初始化MPU6050
	 while(mpu_dmp_init()!=0){}	//初始化成功是0，初始化不成功就循环，直到初始化成功，平放初始化容易成功。
}
/***
* bref @name PID_Init
***/
void PID_Init(void)
{
	 PIDInit(&motor[LEFT].position.position_pid,420,0,600,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
	 PIDInit(&motor[RIGHT].position.position_pid,420,0,600,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
	 PIDInit(&motor[LEFT].speed.speed_pid,60,0.1,0.5,0,6500, 0, 250, 40 * 19, 30 * 19 * 2, 1350, SPEED);
	 PIDInit(&motor[RIGHT].speed.speed_pid,60,0.1,0.5,0,6500, 0, 250, 40 * 19, 30 * 19 * 2, 1350, SPEED);
}
/***
* bref @name OLEDShow_Proc
***/
void OLEDShow_Proc(void)
{
	 //主页菜单栏
		OLED_ShowString(00, 00, logo, 16);
		
//		OLED_ShowChinese(0,18,0,16);
//		OLED_ShowChinese(18,18,1,16);
//		OLED_ShowChinese(36,18,2,16);
//		OLED_ShowChinese(00,36,3,16);
//		OLED_ShowChinese(18,36,4,16);
		
	
		OLED_Refresh();
}

/**************************************************************************
函数功能：主函数
入口参数：无
返回  值：无
**************************************************************************/
int main(void)
{
	 TASK_Init();
	 PID_Init();
	 delay_ms(200);              //延迟等待初始化完成
//	 while(1)
		for(;;)
	  {
			IMUGetData_Porc();
			KEY_Porc();
			BLUE_Porc();
			Adjustpara_Porc();
			OLEDShow_Proc();
	  }
}

