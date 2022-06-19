#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "timer.h"
#include "SEGGER_RTT.h"
#include "main.h"

//ALIENTEK ̽����STM32F407������ ʵ��9
//PWM���ʵ��-�⺯���汾
//����֧�֣�www.openedv.com
//�Ա����̣�http://eboard.taobao.com  
//������������ӿƼ����޹�˾  
//���ߣ�����ԭ�� @ALIENTEK
int   TargetVelocity=100, EncoderLeft,EncoderRight,PWM_left,PWM_right;  //Ŀ���ٶȡ�Ŀ��Ȧ��(λ��)��������������PWM���Ʊ���
motor_control motor[2];
int main(void)
{ 
	u16 led0pwmval=0;    
	u8 dir=1;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);  //��ʼ����ʱ����
	uart_init(115200);//��ʼ�����ڲ�����Ϊ115200
	LED_Init();			     //LED�˿ڳ�ʼ��
 	TIM14_PWM_Init(8399,0);	//84M/84=1Mhz�ļ���Ƶ��,��װ��ֵ500������PWMƵ��Ϊ 1M/500=2Khz.     
	TIM3_Encode_Init();
	EncoderRead_TIM1(8399, 99);
	 PIDInit(&motor[LEFT].speed.speed_pid,60,0.1,0.5,0,6500, 0, 250, 40 * 19, 30 * 19 * 2, 1350, SPEED);
	 PIDInit(&motor[RIGHT].speed.speed_pid,60,0.1,0.5,0,6500, 0, 250, 40 * 19, 30 * 19 * 2, 1350, SPEED);
  while(1) //ʵ�ֱȽ�ֵ��0-300��������300���300-0�ݼ���ѭ��
	{
 		
	}
}
