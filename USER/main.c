/***********************************************
��˾����ݸ��΢�����ܿƼ����޹�˾
Ʒ�ƣ�WHEELTEC
������wheeltec.net
�Ա����̣�shop114407458.taobao.com
�汾V1.0
�޸�ʱ�䣺2020-06-30
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

int   TargetVelocity=100,  CurrentPosition, EncoderLeft,EncoderRight,PWM_left,PWM_right;  //Ŀ���ٶȡ�Ŀ��Ȧ��(λ��)��������������PWM���Ʊ���

float TargetCircle=1;		//float����ת�Ļ�����㣬Ҳ����ȥ���Ǹ�����

int   MortorRun = 0;  //���������Ʊ�־λ


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
�������ܣ�������
��ڲ�������
����  ֵ����
**************************************************************************/
int main(void)
{
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //�ж����ȼ�����
	 delay_init();                //�ӳٺ�����ʼ��
	 LED_Init();                  //LED�Ƴ�ʼ��
	 KEY_Init();									//KEY������ʼ��
   uart_init(9600);             //USART�������ڳ�ʼ��	 
   MotorEncoderLeft_Init();	    //��������ʼ�� ʹ�ö�ʱ��4
	 MotorEncoderRight_Init();		//��������ʼ�� ʹ�ö�ʱ��5
   TB6612_Init(7199, 0);        //������������ʼ�� ʹ�ö�ʱ��3 
	 EncoderRead_TIM2(7199, 99);  //10ms��ȡһ�α�����(��100HZ)��������ٱ�Ϊ20����������������13��AB˫����ϵõ�4��Ƶ��
	                              //��ת1Ȧ����������Ϊ20*13*4=1040�����ת��=Encoder*100/1040r/s ʹ�ö�ʱ��2
		PIDInit(&motor[LEFT].position.position_pid,420,0,600,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
	 PIDInit(&motor[RIGHT].position.position_pid,420,0,600,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
	 PIDInit(&motor[LEFT].speed.speed_pid,60,0.1,0.5,0,6500, 0, 250, 40 * 19, 30 * 19 * 2, 1350, SPEED);
	 PIDInit(&motor[RIGHT].speed.speed_pid,60,0.1,0.5,0,6500, 0, 250, 40 * 19, 30 * 19 * 2, 1350, SPEED);
	
	 delay_ms(200);              //�ӳٵȴ���ʼ�����
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
			//��������С��
//			 GPIO_SetBits(GPIOD,GPIO_Pin_13);
			 seeAngleSet = AngleSet;
			 seeAngleNow = AngleNow;

			 
			 seeSpeeSet = SpeeSet;
			 seeSpeedNow = SpeedNow;

	  }
}

