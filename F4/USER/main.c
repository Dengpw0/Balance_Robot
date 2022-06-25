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

int   TargetVelocity=100, EncoderLeft,EncoderRight,PWM_left,PWM_right;  //Ŀ���ٶȡ�Ŀ��Ȧ��(λ��)��������������PWM���Ʊ���
motor_control motor[2];

float pitch,roll,yaw; 		//ŷ����							//��������ʵ�ʲ���	ŷ����
short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����	//��������ʵ�ʲ���	���ٶ�
short gyrox,gyroy,gyroz;	//������ԭʼ����			//��������ʵ�ʲ���	�����ǽǶ�
float pitch_med = -7;			//��������ֵ

int Res;
u8 usart_value;
int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);  //��ʼ����ʱ����
	uart_init(9600);//��ʼ�����ڲ�����Ϊ115200
	LED_Init();			     //LED�˿ڳ�ʼ��
	MPU_Init();					        //��ʼ��MPU6050
  while(mpu_dmp_init()!=0){}	//��ʼ���ɹ���0����ʼ�����ɹ���ѭ����ֱ����ʼ���ɹ���ƽ�ų�ʼ�����׳ɹ���
	TIM2_PWM_Init(8399,0);	//84M/84=1Mhz�ļ���Ƶ��,��װ��ֵ500������PWMƵ��Ϊ 1M/500=2Khz.     
  TIM3_Encode_Init();
	TIM4_Encode_Init();
	EncoderRead_TIM1(8399, 99);//�˶�ʱ��Ӱ��iicʱ��
	PIDInit(&motor[LEFT].position.imu_pid,-800,0,-0,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
	PIDInit(&motor[RIGHT].position.imu_pid,-800,0,-0,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
 PIDInit(&motor[LEFT].speed.speed_pid,60,0.1,0.5,0,6500, 0, 250, 40 * 19, 30 * 19 * 2, 1350, SPEED);
 PIDInit(&motor[RIGHT].speed.speed_pid,60,0.1,0.5,0,6500, 0, 250, 40 * 19, 30 * 19 * 2, 1350, SPEED);
 PIDInit(&motor[LEFT].position.position_pid,420,0,600,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
 PIDInit(&motor[RIGHT].position.position_pid,420,0,600,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
  while(1) //ʵ�ֱȽ�ֵ��0-300��������300���300-0�ݼ���ѭ��
	{
 		
	}
}
