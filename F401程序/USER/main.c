#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "kalman.h"
#include "main.h"
#include "time.h"
#include "pid.h"
#include "oled.h"
#include "usart_obser.h"

/************************************************************************************************MOTOR************************************************************************************************/
int TargetVelocity=100,EncoderLeft,EncoderRight,PWM_left,PWM_right,Turn_out;  //Ŀ���ٶȡ�Ŀ��Ȧ��(λ��)��������������PWM���Ʊ���
motor_control motor[3];					//��������ṹ��
speed_difference speed_diff[3];	//���ٽṹ��
int speedleft,speedright;				//���ҵ���ٶ����

/************************************************************************************************MPU6050************************************************************************************************/
float pitch,roll,yaw,pitch_kalman,yaw_kalman; 	//ŷ����		
short aacx,aacy,aacz;								//���ٶȴ�����ԭʼ����
short gyrox,gyroy,gyroz;						//������ԭʼ����

/************************************************************************************************Banlance************************************************************************************************/
//�ĸ�ƽ��ֵ
//����ÿ���ϵ磬��ͬ���򣬻�еpitch��е��ֵ����һ���������ϵ�ʱ����ƽ�⣬�ȴ��������Լ�У׼����е��ֵ0
float pitch_med = 0;			//pitch�ǻ�е��ֵ
float groy_med = -6;			//pitch����ٶ���ֵ
int groz_med = -39;				//z����ٶ���ֵ����֤����ֱ��
float targetspeed = 0;		//�ٶȻ�������ֱ��ʱΪ0.����ʱ��ֵ��
float Yaw_target = 0;					//ת���ٶ�����
//pid����ϵ��
float Turn_Kp = 0;							//ת��Kp
float Turn_Kd = 0;
float kp,kd;
float kp_speed,ki_speed;
//�����������������˲��ṹ��
extKalman_t MPU6050_Kalman;
extKalman_t MOTOR_Kalman;
float tq = 1,tr = 1.5;		//�����ǿ������˲�ϵ��
float tq_m = 1,tr_m = 50;	//����������������˲�ϵ��
//�������˲����ֵ
short gyroy_kal;
short gyroz_kal;
int EncoderLeft_kal;
int EncoderRight_kal;
//��ʱ����ʱ
int times;						//��ʱ��4��0.1ms����һ�Σ������+1

/**
* @name TASK_Move
* @brief	�ƶ�����
**/
void TASK_Move(void)
{
	task_flag_last = task_flag;	 //��¼��ǰ����״̬
	switch(task_flag)
	{
		case 0:
			times = 0;							 //�����ʱ
			break;
		case 1:
			targetspeed = 80;				//ǰ��
			if(times >= 500)				//5s��ص�ƽ��
				targetspeed = 0;			//ƽ��
			if(times >= 600)				//1s������ת��
				Yaw_target = 88;			//��ת
			if(times >= 900)				//3s���������ȡ���׶�
			{
				task_flag = 2;				//����ȡ���׶�
				times = 0;						//�ٴ������ʱ
			}
			break;
		case 2:
			if(times >= 500)				//3sȡ��
				task_flag = 3;				//ȡ����ɽ����ͻ��׶�
			break;
		case 3:
			Yaw_target = 0;		      //��ת
			if(times >= 700)				//2s��ת��ǰ��
				targetspeed = 65;			//ǰ��
			if(times >= 1000)				//3sǰ����ͣ��
				targetspeed = 0;			//ƽ��
			if(times >= 1100)				//����1��
				Yaw_target = 88;			//��ת
			if(times >= 1200)				//����1��
				task_flag = 4;				//����ж��״̬���������
			break;
		case 4:
			if(times >= 1400)				//ж��2�룬��֤OLED��ʾ2s
				task_flag = 0;				//���flag
			break;
		default:
			break;
	}
}	

/************************************************************************************************OLED************************************************************************************************/
//������������ʾ
unsigned char mpu_error[] = "mpu_error!";
unsigned char mpu_ok[] = "mpu_ok!";
unsigned char pitchangle[] = "pitch:";
unsigned char gyroyspeed[] = "gyroy:";
unsigned char gyrozspeed[] = "gyroz:";
unsigned char task_getgoods[] = "";
//������ʾ
int i,x;				//forѭ�����ñ���
/**
* @name OLED_Display
* @brief OLED��ʾ�л�
**/
void OLED_Display(void)
{
	if(task_flag != task_flag_last)	//������״̬�ı�ʱ
			OLED_Clear();								//�����ϴ�����
	if(!task_flag)//������ʱ��ʾ��ǰ����������
	{
		OLED_ShowString(0,0,pitchangle,16);
		OLED_ShowString(0,18,gyroyspeed,16);
		OLED_ShowString(0,36,gyrozspeed,16);
		OLED_Float(60,0,pitch,3);
		OLED_Float(60,18,gyroy_kal,3);
		OLED_Float(60,36,gyroz_kal,3);
	  OLED_Refresh();
	}
	else					//�յ����ڷ�����ָ����ʾ������Ϣ
	{
		switch(task_flag)
		{
			case 1:		//���ڵõ�����ʼ����
				//����ǰ��ȡ��
				for(i = 0,x=8;i < 6;i++)
				{
						OLED_ShowChinese(x,25,i,16);
						x += 20;
				}
				break;
			case 2:		//��ɵ�һ�������õ�
				//����ȡ��
				for(i = 6,x=28;i < 10;i++)
				{
						OLED_ShowChinese(x,25,i,16);
						x += 20;
				}
				break;
			case 3:		//��ɵڶ����õ�
				//����ǰ���ͻ�
				for(i = 10,x=8;i < 16;i++)
				{
						OLED_ShowChinese(x,25,i,16);
						x += 20;
				}
				break;
			case 4:		//��ɵ������õ�
				//�������
				for(i = 16,x=28;i < 20;i++)
				{
						OLED_ShowChinese(x,25,i,16);
						x += 20;
				}
				break;
			default:
				break;
		}
		OLED_Refresh();
	}
}

//USART Flag
int task_flag = 0;
int task_flag_last;
/**
* @name PID_Init
* @brief PID������ʼ��
**/
void PID_Init(void)
{
		/*Ч������*/
//	Turn_Kp=1;0
//	kp_speed = 0.00015;
//	ki_speed = 0.00001;
//	kp = 1850;
//  kd = 3.2;
	/*����Ч������*/
//	ֱ����
//	kp = 2400;
//  kd = 8.6;
//	ת��
//	Turn_Kp=5;
//	Turn_Kd=0.3;
//	�ٶȻ�
//	kp_speed = 80;
//	ki_speed = 0.09;
  //ֱ�����������ǣ�
	kp = 2400;
  kd = 8.6;
  PIDInit(&motor[LEFT].position.imu_pid,kp,0,kd);
  PIDInit(&motor[RIGHT].position.imu_pid,kp,0,kd);
  //pdת�򻷣������ǣ�
	Turn_Kp=5;
	Turn_Kd=0.3;
  //pid�ٶȻ�����������
	kp_speed = 80;
	ki_speed = 0.09;
	//pidλ�û�����������
  PIDInit(&motor[DOUBLE].speed.speed_pid,kp_speed,ki_speed,0);
	//��ʼ���������˲�
	KalmanCreate(&MPU6050_Kalman,tq,tr);
	KalmanCreate(&MOTOR_Kalman,tq_m,tr_m);
}

/**
* @name TASK_Init
* @brief �����ʼ��
**/
void TASK_Init(void)
{
	PID_Init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);		  //��ʼ����ʱ����
	uart_init(9600);      //��ʼ�����ڲ�����Ϊ9600����������ͨ��
	LED_Init();		        //��ʼ��LED�˿ڣ����ڲ���
	OLED_Init();					//��ʼ��OLED��ʾ��
	OLED_ColorTurn(0);    //0������ʾ��1 ��ɫ��ʾ
  OLED_DisplayTurn(0);  //0������ʾ 1 ��Ļ��ת��ʾ
  OLED_Refresh();				//ˢ����Ļ����ʾ
	MPU_IIC_Init();       //��ʼ��IIC����
	MPU_Init();					  //��ʼ��MPU6050
  while(mpu_dmp_init()!=0){//�����ǳ�ʼ��
	  OLED_ShowString(30,18,mpu_error,16);
		OLED_Refresh();     //��ʼ��ʧ�ܣ�OLED��ʾ mpu_error
	}
	OLED_Clear();
	//��ʼ���ɹ�OLED��ʾmpu_ok
	OLED_ShowString(40,0,mpu_ok,16);
	OLED_Refresh();
	TIM1_Encode_Init();		//��ʼ����ʱ��1����������
	TIM3_Encode_Init();		//��ʼ����ʱ��3����������
	TIM2_PWM_Init(8399,0);//��ʼ����ʱ��2PWM���
	TICK_TIM5(8399,99);		//10ms��ʱ�������ڵδ�ʱ��
	Control_TIM4(839, 9); //��ʼ����ʱ��4�������ݴ���0.1ms����һ���ж�
}
int main(void)
{ 
	TASK_Init();
//	delay_ms(3000);
	OLED_Clear();//�����ϴ�����
	LED = 0;
	while(1)
	{	oscillographyData[0] = EncoderLeft;
		oscillographyData[1] = EncoderLeft_kal;

		oscillographyData[2] = pitch;
		oscillographyData[3] = roll;
		oscillographyData[4] = yaw;
		oscillographyData[5] = gyroy_kal;
		oscillographyData[6] = gyroz_kal;
		oscillographyData[7] = 0;
		oscillographyData[8] = 0;
		oscillographyData[9] = 0;
		oscillography_sendData(oscillographyData, 10);
//		printf("task_flag:%d",task_flag);
		OLED_Display();	//OLED���ݱ�־λ��ʾ��ͬ����
		TASK_Move();		//������ִ��
	}
}
