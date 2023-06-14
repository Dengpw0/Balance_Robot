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
#include "kalman.h"
//MOTOR
int   TargetVelocity=100, EncoderLeft,EncoderRight,PWM_left,PWM_right,Turn_out;  //目标速度、目标圈数(位置)、编码器读数、PWM控制变量
motor_control motor[2];
speed_difference speed_diff[2];
//MPU6050
float pitch,roll,yaw,pitch_kalman; 		//欧拉角							//定义三个实际参数	欧拉角
short aacx,aacy,aacz;		//加速度传感器原始数据	//定义三个实际参数	加速度
short gyrox,gyroy,gyroz;	//陀螺仪原始数据			//定义三个实际参数	陀螺仪角度
float pitch_med = -5.8;//-3.5;			//陀螺仪中值		-7是水平，却不是回中  -12 -3阈值		1.1-4.1
float groy_med = -37;
float targetspeed = 0;	//速度环期望（直立时为0.行走时给值）
int groz_med = 0;	//转向环期望（为0时走直线）
int pitchsee;
int Turn_Kp,gyroz_med,gyroz_set;
//my_value
int seepitch;
int Res;
u8 usart_value;
int speedleft,speedright;

int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);  //初始化延时函数
	uart_init(9600);//初始化串口波特率为115200
	LED_Init();			     //LED端口初始化
	MPU_Init();					        //初始化MPU6050
  while(mpu_dmp_init()!=0){}	//初始化成功是0，初始化不成功就循环，直到初始化成功，平放初始化容易成功。
//	TIM2_PWM_Init(8399,0);	//84M/84=1Mhz的计数频率,重装载值500，所以PWM频率为 1M/500=2Khz.     
//  TIM3_Encode_Init();
//	TIM4_Encode_Init();
//	EncoderRead_TIM1(8399, 99);//此定时器影响iic时序
//	KalmanCreate(&speedK, 1, 50);	//Q R	
		//跑一段但起不来
//	PIDInit(&motor[LEFT].position.imu_pid,-800,0,-4,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
//	PIDInit(&motor[RIGHT].position.imu_pid,-800,0,-4,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
		//试试加D，jiaD后虽然有所调高，但是太抖，还是起不来
//		PIDInit(&motor[LEFT].position.imu_pid,-800,0,-12,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
//	PIDInit(&motor[RIGHT].position.imu_pid,-800,0,-12,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
		//试试加P	，效果好了许多，并找到了更好的中值，但是泰斗
//		PIDInit(&motor[LEFT].position.imu_pid,-3000,0,-8,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
//	PIDInit(&motor[RIGHT].position.imu_pid,-3000,0,-8,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
		//试试减小D，P大D太小，来回晃，
//	PIDInit(&motor[LEFT].position.imu_pid,-2000,0,-4,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
//	PIDInit(&motor[RIGHT].position.imu_pid,-2000,0,-4,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
		//再增大D	抖啊抖	不太抖，但起不来，现在问题大了，轮子高度不同，速度不同，pitch轴一直动
//		PIDInit(&motor[LEFT].position.imu_pid,-1760,0,-2.4,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
//	PIDInit(&motor[RIGHT].position.imu_pid,-1760,0,-2.4,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
		//差不多了，晃晃悠悠可以立柱，但是还是抖，但参数大概这个左右了，等充满电回来加一些D
//	PIDInit(&motor[LEFT].position.imu_pid,-3800,0,-5,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
//	PIDInit(&motor[RIGHT].position.imu_pid,-3800,0,-5,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
//4.5抖			3300 4.5来回抖		3000 4.5好许多	2800 4.5起不来还有些抖	3000 9 抖两下，但是起不来了
//抖来抖去，勉强直立
// PIDInit(&motor[LEFT].position.imu_pid,-3500,0,-5,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
// PIDInit(&motor[RIGHT].position.imu_pid,-3500,0,-5,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);

/**************** 去掉差速 *******************/
//按道理是这么大调，但是发现差速不能加后，用回上面的那个，立柱了，就是超频很严重
// PIDInit(&motor[LEFT].position.imu_pid,-500,0,-0,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
// PIDInit(&motor[RIGHT].position.imu_pid,-500,0,-0,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
//立柱，但是抖来抖去（-3.5）
// PIDInit(&motor[LEFT].position.imu_pid,-3500,0,-5,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
// PIDInit(&motor[RIGHT].position.imu_pid,-3500,0,-5,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
//已经立柱了，还是有一点点抖，可能机械中值还有些偏 
// PIDInit(&motor[LEFT].position.imu_pid,-2800,0,-5,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
// PIDInit(&motor[RIGHT].position.imu_pid,-2800,0,-5,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
//效果不好
// PIDInit(&motor[LEFT].position.imu_pid,-1000,0,-0.8,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
// PIDInit(&motor[RIGHT].position.imu_pid,-1000,0,-0.8,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
//不行
// PIDInit(&motor[LEFT].position.imu_pid,-400,0,-0.5,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
// PIDInit(&motor[RIGHT].position.imu_pid,-400,0,-0.5,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
//稳住
 //直立环（陀螺仪）
 PIDInit(&motor[LEFT].position.imu_pid,-4000,0,-6.6,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
 PIDInit(&motor[RIGHT].position.imu_pid,-4000,0,-6.6,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
 //差速环，kp转向环（陀螺仪）
// PIDInit(&speed_diff[DOUBLE].diff_pid,6,0,0,0,6500, 0, 250, 40 * 19, 30 * 19 * 2, 1350, SPEED);
 Turn_Kp=4;
 //pi速度环（陀螺仪）
 PIDInit(&motor[DOUBLE].speed.speed_pid,0.03,0.00015,0,0,6500, 0, 250, 40 * 19, 30 * 19 * 2, 1350, SPEED);
 //pid速度环（编码器）
 PIDInit(&motor[LEFT].speed.speed_pid,60,0.1,0.5,0,6500, 0, 250, 40 * 19, 30 * 19 * 2, 1350, SPEED);
 PIDInit(&motor[RIGHT].speed.speed_pid,60,0.1,0.5,0,6500, 0, 250, 40 * 19, 30 * 19 * 2, 1350, SPEED);
 //pid位置环（编码器）
 PIDInit(&motor[LEFT].position.position_pid,420,0,600,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
 PIDInit(&motor[RIGHT].position.position_pid,420,0,600,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
  while(1) //实现比较值从0-300递增，到300后从300-0递减，循环
	{
		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
		{
		mpu_dmp_get_data(&pitch,&roll,&yaw);	//得到数据时，return 0，跳出循环，如果没得到数据，一直循环，直到得到dmp计算出的数据
     MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);  // 读取角速度
     MPU_Get_Accelerometer(&aacx,&aacy,&aacz); // 读取加速度
		}
		switch(usart_value)
		{
			case Return:
				targetspeed = 0;
			  groz_med = 0;
				break;
			case Go_font:
				targetspeed = 30;
			  break;
			case Go_back:
				targetspeed = -30;
			  break;
			case Go_left:
				groz_med = -90;
			  break;
			case Go_right:
				groz_med = 90;
			  break;
		}
		SEGGER_RTT_printf(0,"pitch_kalman %d\r\n",pitch_kalman);
//					if((lastspeed-viewspeed)>50||(lastspeed-viewspeed)<-50)
//					SEGGER_RTT_printf(0,"times %d\r\n",times++);
	}
}
