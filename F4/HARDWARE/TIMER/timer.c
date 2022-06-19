#include "timer.h"
#include "led.h"
#include "usart.h"
 #include "main.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//��ʱ��PWM ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/4
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

//�ⲿ���� extern˵���ı������������ļ�����
extern int   Encoder, CurrentPosition; //��ǰ�ٶȡ���ǰλ��
extern int   TargetVelocity, CurrentPosition, EncoderLeft,EncoderRight,PWM_left,PWM_right;//Ŀ���ٶȡ�Ŀ��Ȧ����������������PWM���Ʊ���
extern float TargetCircle,AngleNow,AngleSet,SpeeSet,SpeedNow,AngleMax,k,AngleMin;
extern float Velcity_Kp,  Velcity_Ki,  Velcity_Kd; //����ٶ�PID����
extern float Position_Kp, Position_Ki, Position_Kd; //���λ��PID����
extern int   MortorRun;  //���������Ʊ�־λ

extern float pitch,roll,yaw,pitch_med; 		//ŷ����							//�����ĸ�ʵ�ʲ���	ŷ����	һ��pitch��ֵ������ƽ��
extern short gyrox,gyroy,gyroz;	//������ԭʼ����			//��������ʵ�ʲ���	�����ǽǶ�
extern short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����	//��������ʵ�ʲ���	���ٶ�

extern u16 flag;
extern PidTypeDef position;
extern PidTypeDef speed;
//TIM14 PWM���ֳ�ʼ�� 
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void TIM14_PWM_Init(u32 arr,u32 psc)
{		 					 
	//�˲������ֶ��޸�IO������
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  	//TIM14ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_TIM2); //GPIOF9����Ϊ��ʱ��14
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_Init(GPIOA,&GPIO_InitStructure);              //��ʼ��PF9
	
	GPIO_ResetBits(GPIOA, GPIO_Pin_1|GPIO_Pin_2); //��ʼ���������ŵ͵�ƽ
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;        //����
	GPIO_Init(GPIOA,&GPIO_InitStructure);              //��ʼ��PF9
	
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=0; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);//��ʼ����ʱ��14
	
	//��ʼ��TIM14 Channel1 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ե�
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 4OC4

	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);  //ʹ��TIM14��CCR4�ϵ�Ԥװ�ؼĴ���
 
  TIM_ARRPreloadConfig(TIM2,ENABLE);//ARPEʹ�� 	//����
	
	TIM_Cmd(TIM2, ENABLE);  //ʹ��TIM14

}  
void TIM3_Encode_Init(void)
{		 					 
	GPIO_InitTypeDef GPIO_InitStructure; //����һ�����ų�ʼ���Ľṹ��  
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;//����һ����ʱ����ʼ���Ľṹ��
  TIM_ICInitTypeDef TIM_ICInitStructure; //����һ����ʱ��������ģʽ��ʼ���Ľṹ��
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʹ��TIM5ʱ��
	RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //ʹ��CPIOAʱ��
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	//TIM5_CH2��TIM5_CH3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);	//����GPIO_InitStructure�Ĳ�����ʼ��GPIO
	
	 GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3); //GPIOF9����λ��ʱ��14
	 GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3); //GPIOF9����λ��ʱ��14

	TIM_TimeBaseStructure.TIM_Period = 0xffff; //�趨�������Զ���װֵ
	TIM_TimeBaseStructure.TIM_Prescaler = 0; // Ԥ��Ƶ�� 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //ѡ��ʱ�ӷ�Ƶ������Ƶ
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct�Ĳ�����ʼ����ʱ��TIM4
	
  TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //ʹ�ñ�����ģʽ3��CH1��CH2ͬʱ������Ϊ�ķ�Ƶ
  TIM_ICStructInit(&TIM_ICInitStructure); //��TIM_ICInitStruct �е�ÿһ��������ȱʡֵ����
	TIM_ICInitStructure.TIM_ICFilter = 10;  //�����˲�������
  TIM_ICInit(TIM3, &TIM_ICInitStructure); //��TIM_ICInitStructure������ʼ����ʱ��TIM4������ģʽ
	
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);//���TIM�ĸ��±�־λ
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); //�����ж�ʹ��
  TIM_SetCounter(TIM3,0); //��ʼ����ձ�������ֵ
	
	TIM_Cmd(TIM3, ENABLE); //ʹ�ܶ�ʱ��4

}  
/**************************************************************************
�������ܣ���ȡTIM4��������ֵ
��ڲ�������
����  ֵ����
**************************************************************************/
int Read_Encoder(int left_or_right)
{
	int Encoder_TIM;
	switch(left_or_right)
	{
		case 1:
			Encoder_TIM=TIM3->CNT; //��ȡ����
			if(Encoder_TIM>0xefff)Encoder_TIM=Encoder_TIM-0xffff; //ת������ֵΪ�з����ֵ������0��ת��С��0��ת��
																														//TIM4->CNT��ΧΪ0-0xffff����ֵΪ0��
			TIM3->CNT=0; //��ȡ����������
			return Encoder_TIM; //����ֵ
		case 0:
			Encoder_TIM=TIM5->CNT; //��ȡ����
			if(Encoder_TIM>0xefff)Encoder_TIM=Encoder_TIM-0xffff; //ת������ֵΪ�з����ֵ������0��ת��С��0��ת��
																														//TIM4->CNT��ΧΪ0-0xffff����ֵΪ0��
			TIM5->CNT=0; //��ȡ����������
			return Encoder_TIM; //����ֵ
	}
	return 0;
}
/**************************************************************************
�������ܣ�TIM4�жϷ�����
��ڲ�������
����  ֵ����
**************************************************************************/
void TIM3_IRQHandler(void)
{ 		    		  			    
	if(TIM3->SR&0X0001)//����ж�
	{    				   				     	    	
	}				   
	TIM3->SR&=~(1<<0);//����жϱ�־λ 	    
}
/**************************************************************************
�������ܣ�ͨ�ö�ʱ��2��ʼ��������
��ڲ������Զ���װ��ֵ Ԥ��Ƶϵ�� Ĭ�϶�ʱʱ��Ϊ72MHZʱ�����߹�ͬ������ʱ�ж�ʱ��
����  ֵ����
**************************************************************************/
void EncoderRead_TIM1(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue; //����һ����ʱ�жϵĽṹ��
	NVIC_InitTypeDef NVIC_InitStrue; //����һ���ж����ȼ���ʼ���Ľṹ��
	
  RCC_APB1PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //ʹ��ͨ�ö�ʱ��2ʱ��
	
	TIM_TimeBaseInitStrue.TIM_Period=arr; //����ģʽΪ���ϼ���ʱ����ʱ����0��ʼ����������������arrʱ������ʱ�жϷ�����
	TIM_TimeBaseInitStrue.TIM_Prescaler=psc; //Ԥ��Ƶϵ��������ÿһ��������ʱ��
	TIM_TimeBaseInitStrue.TIM_CounterMode=TIM_CounterMode_Up; //����ģʽ�����ϼ���
	TIM_TimeBaseInitStrue.TIM_ClockDivision=TIM_CKD_DIV1; //һ�㲻ʹ�ã�Ĭ��TIM_CKD_DIV1
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStrue); //����TIM_TimeBaseInitStrue�Ĳ�����ʼ����ʱ��TIM2
	
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); //ʹ��TIM2�жϣ��ж�ģʽΪ�����жϣ�TIM_IT_Update
	
	NVIC_InitStrue.NVIC_IRQChannel=TIM1_UP_TIM10_IRQn; //����TIM2�ж�
	NVIC_InitStrue.NVIC_IRQChannelCmd=ENABLE; //�ж�ʹ��
	NVIC_InitStrue.NVIC_IRQChannelPreemptionPriority=1; //��ռ���ȼ�Ϊ1����ֵԽС���ȼ�Խ�ߣ�0�����ȼ����
	NVIC_InitStrue.NVIC_IRQChannelSubPriority=1; //��Ӧ���ȼ�Ϊ1����ֵԽС���ȼ�Խ�ߣ�0�����ȼ����
	NVIC_Init(&NVIC_InitStrue); //����NVIC_InitStrue�Ĳ�����ʼ��VIC�Ĵ���������TIM2�ж�
	
	TIM_Cmd(TIM1, ENABLE); //ʹ�ܶ�ʱ��TIM2
}
#define SPEED 1
#define POSITION 0
void TIM1_UP_TIM10_IRQHandler()
{
  if(TIM_GetITStatus(TIM1, TIM_IT_Update)==1) //�������ж�ʱ״̬�Ĵ���(TIMx_SR)��bit0�ᱻӲ����1
	{
	   EncoderLeft=Read_Encoder(1);   //��ȡ��ǰ����������
		 EncoderRight=Read_Encoder(0);   //��ȡ��ǰ����������
		//�ٶȻ�
		 motor[LEFT].speed.SpeedSet = TargetVelocity;//motor[LEFT].position.PWM/76;	//
		 motor[RIGHT].speed.SpeedSet = TargetVelocity;//motor[RIGHT].position.PWM/76;	//
		
		 motor[LEFT].speed.SpeedNow = EncoderLeft*1000*200*0.000120830;
		 motor[RIGHT].speed.SpeedNow = EncoderRight*1000*200*0.000120830;
		 
		 motor[LEFT].speed.seeSpeedSet =  motor[LEFT].speed.SpeedSet;
		 motor[RIGHT].speed.seeSpeedSet =  motor[RIGHT].speed.SpeedSet;
		 
		 PID_Calc(&motor[LEFT].speed.speed_pid,motor[LEFT].speed.SpeedNow,motor[LEFT].speed.SpeedSet);
		 PID_Calc(&motor[RIGHT].speed.speed_pid,motor[RIGHT].speed.SpeedNow,motor[RIGHT].speed.SpeedSet);
		 
		 motor[LEFT].speed.PWM = motor[LEFT].speed.speed_pid.output;
		 motor[RIGHT].speed.PWM = motor[RIGHT].speed.speed_pid.output;
		 
#if (SPEED==1)
		SetPWM(motor[LEFT].speed.PWM, motor[RIGHT].speed.PWM); //����PWM
#endif
		
#if (POSITION==1)
		SetPWM(motor[LEFT].position.PWM, motor[RIGHT].position.PWM); //����PWM
#endif
			
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update); //״̬�Ĵ���(TIMx_SR)��bit0��0
	}
}
/**************************************************************************
�������ܣ�����TIM3ͨ��4PWMֵ
��ڲ�����PWMֵ
����  ֵ����
**************************************************************************/
void SetPWM(int pwmleft,int pwmright)
{
  if(pwmleft>=0)//pwm>=0 (BIN1, BIN2)=(0, 1) ��ת
  {
		PAout(1)=0; //BIN1=0
		PAout(2)=1; //BIN2=1
		TIM2->CCR4=pwmleft;
		TIM_SetCompare4(TIM2, pwmleft);
  }
  else if(pwmleft<0)//pwm<0 (BIN1, BIN2)=(1, 0) ��ת
  {
		PAout(1)=1; //BIN1=1
		PAout(2)=0; //BIN2=0
		TIM2->CCR4=-pwmleft;
		TIM_SetCompare4(TIM2, -pwmleft);
  }
	if(pwmright>=0)//pwm>=0 (BIN1, BIN2)=(0, 1) ��ת
  {
		PAout(15)=1; //BIN1=0
		PAout(14)=0; //BIN2=1
		TIM3->CCR3=pwmright;
		TIM_SetCompare3(TIM3, pwmright);
  }
  else if(pwmright<0)//pwm<0 (BIN1, BIN2)=(1, 0) ��ת
  {
		PAout(15)=0; //BIN1=1
		PAout(14)=1; //BIN2=0
		TIM3->CCR3=-pwmright;
		TIM_SetCompare3(TIM3, -pwmright);
  }
}
