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

int   TargetVelocity=300,  CurrentPosition, Encoder,SPEED_PWM,POSITION_PWM;  //Ŀ���ٶȡ�Ŀ��Ȧ��(λ��)��������������PWM���Ʊ���
float TargetCircle=1;		//float����ת�Ļ�����㣬Ҳ����ȥ���Ǹ�����
float Velcity_Kp=20,  Velcity_Ki=5,  Velcity_Kd; //����ٶ�PID����
//float Position_Kp=120, Position_Ki=0.1, Position_Kd=400; //���λ��PID����
float Position_Kp=120, Position_Ki=0.1, Position_Kd=400; //���λ��PID����
int   MortorRun = 0;  //���������Ʊ�־λ

u16 adc_value;
u16 adc_valueleft;
u16 adc_valueright;
u16 adc_sw;
float tmpvalue;
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
   //uart_init(9600);           //���ڳ�ʼ��	 
   MotorEncoder_Init();	        //��������ʼ�� ʹ�ö�ʱ��4
   TB6612_Init(7199, 0);        //������������ʼ�� ʹ�ö�ʱ��3 
	 EncoderRead_TIM2(7199, 99);  //10ms��ȡһ�α�����(��100HZ)��������ٱ�Ϊ20����������������13��AB˫����ϵõ�4��Ƶ��
	                              //��ת1Ȧ����������Ϊ20*13*4=1040�����ת��=Encoder*100/1040r/s ʹ�ö�ʱ��2
		PIDInit(&position,420,0,600,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
	 PIDInit(&speed,60,0.1,0.5,0,6500, 0, 250, 40 * 19, 30 * 19 * 2, 1350, SPEED);
	 delay_ms(200);              //�ӳٵȴ���ʼ�����
	 orignposition = TargetCircle;
//	 while(1)
		for(;;)
	  {
			 seeAngleSet = AngleSet;
			 seeAngleNow = AngleNow;

			 
			 seeSpeeSet = SpeeSet;
			 seeSpeedNow = SpeedNow;

	  }
}

