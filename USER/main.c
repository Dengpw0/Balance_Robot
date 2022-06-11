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

int   TargetVelocity=100, EncoderLeft,EncoderRight,PWM_left,PWM_right;  //Ŀ���ٶȡ�Ŀ��Ȧ��(λ��)��������������PWM���Ʊ���

float TargetCircle=1;		//float����ת�Ļ�����㣬Ҳ����ȥ���Ǹ�����

int   MortorRun = 0;  //���������Ʊ�־λ


float orignposition;

u16 times;

u8 key_value;
u8 usart_value;

motor_control motor[2];

float pitch,roll,yaw; 		//ŷ����							//��������ʵ�ʲ���	ŷ����
short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����	//��������ʵ�ʲ���	���ٶ�
short gyrox,gyroy,gyroz;	//������ԭʼ����			//��������ʵ�ʲ���	�����ǽǶ�
//short temp;					//�¶�	    

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
	while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}	//�õ�����ʱ��return 0������ѭ�������û�õ����ݣ�һֱѭ����ֱ���õ�dmp�����������
	//temp=MPU_Get_Temperature();	//�õ��¶�ֵ��оƬ�¶�
	MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ��������ݣ���ȡ����ʵ�ʲ����ĵ�ַ�����뺯��
	MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
	//mpu6050_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//���Զ���֡���ͼ��ٶȺ�������ԭʼ����
	//usart1_report_imu(aacx,aacy,aacz,gyrox,gyroy,gyroz,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));	//����DMP����֮������ݣ���λ������			
	printf("roll:%d pitch: %d yaw:%d\r\n",(int)(roll*100),(int)(pitch*100),(int)(yaw*10));	//���ڷ���
}
/***
* bref @name TASK_Init
***/
void TASK_Init(void)
{
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //�ж����ȼ�����
	 delay_init();                //�ӳٺ�����ʼ��
	 OLED_Init();									//��ʾ����ʼ��
	 LED_Init();                  //LED�Ƴ�ʼ��
	 KEY_Init();									//KEY������ʼ��
   uart_init(9600);             //USART�������ڳ�ʼ��	 
   MotorEncoderLeft_Init();	    //��������ʼ�� ʹ�ö�ʱ��4
	 MotorEncoderRight_Init();		//��������ʼ�� ʹ�ö�ʱ��5
   TB6612_Init(7199, 0);        //������������ʼ�� ʹ�ö�ʱ��3 
	 EncoderRead_TIM2(7199, 99);  //10ms��ȡһ�α�����(��100HZ)��������ٱ�Ϊ20����������������13��AB˫����ϵõ�4��Ƶ��
	                              //��ת1Ȧ����������Ϊ20*13*4=1040�����ת��=Encoder*100/1040r/s ʹ�ö�ʱ��2
	 OLED_ColorTurn(0);           //0������ʾ��1 ��ɫ��ʾ
   OLED_DisplayTurn(0);         //0������ʾ 1 ��Ļ��ת��ʾ
	 OLED_Refresh();
	 MPU_Init();					        //��ʼ��MPU6050
	 while(mpu_dmp_init()!=0){}	//��ʼ���ɹ���0����ʼ�����ɹ���ѭ����ֱ����ʼ���ɹ���ƽ�ų�ʼ�����׳ɹ���
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
	 //��ҳ�˵���
		OLED_ShowString(00, 00, logo, 16);
		
//		OLED_ShowChinese(0,18,0,16);
//		OLED_ShowChinese(18,18,1,16);
//		OLED_ShowChinese(36,18,2,16);
//		OLED_ShowChinese(00,36,3,16);
//		OLED_ShowChinese(18,36,4,16);
		
	
		OLED_Refresh();
}

/**************************************************************************
�������ܣ�������
��ڲ�������
����  ֵ����
**************************************************************************/
int main(void)
{
	 TASK_Init();
	 PID_Init();
	 delay_ms(200);              //�ӳٵȴ���ʼ�����
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

