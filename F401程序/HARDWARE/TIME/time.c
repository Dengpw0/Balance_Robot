#include "time.h"
#include "led.h"
#include "usart.h"
#include "main.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "kalman.h"



//TIM14 PWM���ֳ�ʼ�� 
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void TIM2_PWM_Init(u32 arr,u32 psc)
{		 					 
	//�˲������ֶ��޸�IO������
//	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  	//TIM14ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_TIM2); //GPIOF9����Ϊ��ʱ��14
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM2); //GPIOF9����Ϊ��ʱ��14
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_4|GPIO_Pin_5;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_Init(GPIOA,&GPIO_InitStructure);              //��ʼ��PF9
	
	GPIO_ResetBits(GPIOA, GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_4|GPIO_Pin_5); //��ʼ���������ŵ͵�ƽ
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_0;           //GPIOF9
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
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 4OC4

	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);  //ʹ��TIM14��CCR4�ϵ�Ԥװ�ؼĴ���
  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);  //ʹ��TIM14��CCR4�ϵ�Ԥװ�ؼĴ���
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
void TIM1_Encode_Init(void)
{		 					 
	GPIO_InitTypeDef GPIO_InitStructure; //����һ�����ų�ʼ���Ľṹ��  
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;//����һ����ʱ����ʼ���Ľṹ��
  TIM_ICInitTypeDef TIM_ICInitStructure; //����һ����ʱ��������ģʽ��ʼ���Ľṹ��
	
	RCC_APB1PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //ʹ��TIM5ʱ��
	RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //ʹ��CPIOAʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;	//TIM5_CH2��TIM5_CH3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);	//����GPIO_InitStructure�Ĳ�����ʼ��GPIO
	
	 GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_TIM1); //GPIOF9����λ��ʱ��14
	 GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_TIM1); //GPIOF9����λ��ʱ��14

	TIM_TimeBaseStructure.TIM_Period = 0xffff; //�趨�������Զ���װֵ
	TIM_TimeBaseStructure.TIM_Prescaler = 0; // Ԥ��Ƶ�� 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //ѡ��ʱ�ӷ�Ƶ������Ƶ
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct�Ĳ�����ʼ����ʱ��TIM4
	
  TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //ʹ�ñ�����ģʽ3��CH1��CH2ͬʱ������Ϊ�ķ�Ƶ
  TIM_ICStructInit(&TIM_ICInitStructure); //��TIM_ICInitStruct �е�ÿһ��������ȱʡֵ����
	TIM_ICInitStructure.TIM_ICFilter = 10;  //�����˲�������
  TIM_ICInit(TIM1, &TIM_ICInitStructure); //��TIM_ICInitStructure������ʼ����ʱ��TIM4������ģʽ
	
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);//���TIM�ĸ��±�־λ
  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); //�����ж�ʹ��
  TIM_SetCounter(TIM1,0); //��ʼ����ձ�������ֵ
	
	TIM_Cmd(TIM1, ENABLE); //ʹ�ܶ�ʱ��4

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
		case 0:
			Encoder_TIM=TIM3->CNT; //��ȡ����
			if(Encoder_TIM>0xefff)Encoder_TIM=Encoder_TIM-0xffff; //ת������ֵΪ�з����ֵ������0��ת��С��0��ת��
																														//TIM4->CNT��ΧΪ0-0xffff����ֵΪ0��
			TIM3->CNT=0; //��ȡ����������
			return Encoder_TIM; //����ֵ
		case 1:
			Encoder_TIM=TIM1->CNT; //��ȡ����
			if(Encoder_TIM>0xefff)Encoder_TIM=Encoder_TIM-0xffff; //ת������ֵΪ�з����ֵ������0��ת��С��0��ת��
																														//TIM4->CNT��ΧΪ0-0xffff����ֵΪ0��
			TIM1->CNT=0; //��ȡ����������
			return Encoder_TIM; //����ֵ
	}
	return 0;
}
/**************************************************************************
�������ܣ�ͨ�ö�ʱ��4��ʼ��������
��ڲ������Զ���װ��ֵ Ԥ��Ƶϵ�� Ĭ�϶�ʱʱ��Ϊ72MHZʱ�����߹�ͬ������ʱ�ж�ʱ��
����  ֵ����
**************************************************************************/
void Control_TIM4(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue; //����һ����ʱ�жϵĽṹ��
	NVIC_InitTypeDef NVIC_InitStrue; //����һ���ж����ȼ���ʼ���Ľṹ��
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //ʹ��ͨ�ö�ʱ��2ʱ��
	
	TIM_TimeBaseInitStrue.TIM_Period=arr; //����ģʽΪ���ϼ���ʱ����ʱ����0��ʼ����������������arrʱ������ʱ�жϷ�����
	TIM_TimeBaseInitStrue.TIM_Prescaler=psc; //Ԥ��Ƶϵ��������ÿһ��������ʱ��
	TIM_TimeBaseInitStrue.TIM_CounterMode=TIM_CounterMode_Up; //����ģʽ�����ϼ���
	TIM_TimeBaseInitStrue.TIM_ClockDivision=TIM_CKD_DIV1; //һ�㲻ʹ�ã�Ĭ��TIM_CKD_DIV1
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStrue); //����TIM_TimeBaseInitStrue�Ĳ�����ʼ����ʱ��TIM2
	
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); //ʹ��TIM2�жϣ��ж�ģʽΪ�����жϣ�TIM_IT_Update
	
	NVIC_InitStrue.NVIC_IRQChannel=TIM4_IRQn; //����TIM2�ж�
	NVIC_InitStrue.NVIC_IRQChannelCmd=ENABLE; //�ж�ʹ��
	NVIC_InitStrue.NVIC_IRQChannelPreemptionPriority=1; //��ռ���ȼ�Ϊ1����ֵԽС���ȼ�Խ�ߣ�0�����ȼ����
	NVIC_InitStrue.NVIC_IRQChannelSubPriority=1; //��Ӧ���ȼ�Ϊ1����ֵԽС���ȼ�Խ�ߣ�0�����ȼ����
	NVIC_Init(&NVIC_InitStrue); //����NVIC_InitStrue�Ĳ�����ʼ��VIC�Ĵ���������TIM2�ж�
	
	TIM_Cmd(TIM4, ENABLE); //ʹ�ܶ�ʱ��TIM2
}
/**************************************************************************
�������ܣ�ͨ�ö�ʱ��5��ʼ��������
��ڲ������Զ���װ��ֵ Ԥ��Ƶϵ�� Ĭ�϶�ʱʱ��Ϊ72MHZʱ�����߹�ͬ������ʱ�ж�ʱ��
����  ֵ����
**************************************************************************/
void TICK_TIM5(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue; //����һ����ʱ�жϵĽṹ��
	NVIC_InitTypeDef NVIC_InitStrue; //����һ���ж����ȼ���ʼ���Ľṹ��
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); //ʹ��ͨ�ö�ʱ��2ʱ��
	
	TIM_TimeBaseInitStrue.TIM_Period=arr; //����ģʽΪ���ϼ���ʱ����ʱ����0��ʼ����������������arrʱ������ʱ�жϷ�����
	TIM_TimeBaseInitStrue.TIM_Prescaler=psc; //Ԥ��Ƶϵ��������ÿһ��������ʱ��
	TIM_TimeBaseInitStrue.TIM_CounterMode=TIM_CounterMode_Up; //����ģʽ�����ϼ���
	TIM_TimeBaseInitStrue.TIM_ClockDivision=TIM_CKD_DIV1; //һ�㲻ʹ�ã�Ĭ��TIM_CKD_DIV1
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStrue); //����TIM_TimeBaseInitStrue�Ĳ�����ʼ����ʱ��TIM2
	
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE); //ʹ��TIM2�жϣ��ж�ģʽΪ�����жϣ�TIM_IT_Update
	
	NVIC_InitStrue.NVIC_IRQChannel=TIM5_IRQn; //����TIM2�ж�
	NVIC_InitStrue.NVIC_IRQChannelCmd=ENABLE; //�ж�ʹ��
	NVIC_InitStrue.NVIC_IRQChannelPreemptionPriority=1; //��ռ���ȼ�Ϊ1����ֵԽС���ȼ�Խ�ߣ�0�����ȼ����
	NVIC_InitStrue.NVIC_IRQChannelSubPriority=1; //��Ӧ���ȼ�Ϊ1����ֵԽС���ȼ�Խ�ߣ�0�����ȼ����
	NVIC_Init(&NVIC_InitStrue); //����NVIC_InitStrue�Ĳ�����ʼ��VIC�Ĵ���������TIM2�ж�
	
	TIM_Cmd(TIM5, ENABLE); //ʹ�ܶ�ʱ��TIM2
}
void TIM5_IRQHandler()
{
	if(TIM_GetITStatus(TIM5, TIM_IT_Update)==1) 
	{//TIM5�δ��ʱ
		times++;
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	}
}
#define SPEED 0
#define POSITION 1
void TIM4_IRQHandler()
{
  if(TIM_GetITStatus(TIM4, TIM_IT_Update)==1) //�������ж�ʱ״̬�Ĵ���(TIMx_SR)��bit0�ᱻӲ����1
	{
	  EncoderLeft=Read_Encoder(0);   //��ȡ��ǰ����������
		EncoderRight=Read_Encoder(1);   //��ȡ��ǰ����������
		EncoderLeft_kal = KalmanFilter(&MOTOR_Kalman,EncoderLeft);
		EncoderRight_kal = KalmanFilter(&MOTOR_Kalman,EncoderRight);
		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
		{
		  mpu_dmp_get_data(&pitch,&roll,&yaw);			// ��ȡ����Ƕ�
      MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);  // ��ȡ���ٶ�
			gyroy_kal = KalmanFilter(&MPU6050_Kalman,gyroy);
			gyroz_kal = KalmanFilter(&MPU6050_Kalman,gyroz);
      MPU_Get_Accelerometer(&aacx,&aacy,&aacz); // ��ȡ���ٶ�
		}
		 //������λ�û�
//		 motor[LEFT].position.PositionSet = 1;
//		 motor[RIGHT].position.PositionSet = 2;
//		
//		 motor[LEFT].position.PositionNow += EncoderLeft;
//		 motor[RIGHT].position.PositionNow += EncoderRight;
//		 
//		 PID_Calc(&motor[LEFT].speed.speed_pid,motor[LEFT].speed.SpeedNow,motor[LEFT].speed.SpeedSet);
//		 PID_Calc(&motor[RIGHT].speed.speed_pid,motor[RIGHT].speed.SpeedNow,motor[RIGHT].speed.SpeedSet);
//		 PID_Calc(&motor[LEFT].position.position_pid,motor[LEFT].position.PositionNow,motor[LEFT].position.PositionSet);
//		 PID_Calc(&motor[RIGHT].position.position_pid,motor[RIGHT].position.PositionNow,motor[RIGHT].position.PositionSet);
//		 
//		 motor[LEFT].position.PWM = motor[LEFT].position.position_pid.output;
//		 motor[RIGHT].position.PWM = motor[RIGHT].position.position_pid.output;
		//�������ٶȻ�
//		 motor[LEFT].speed.SpeedSet = TargetVelocity;//motor[LEFT].position.PWM/76;	//
//		 motor[RIGHT].speed.SpeedSet = TargetVelocity;//motor[RIGHT].position.PWM/76;	//
		
/***********Balance***********/
		//�ٶȻ�
		PidCalc_Encode(&motor[DOUBLE].speed.speed_pid,EncoderLeft_kal,EncoderRight_kal,targetspeed,ENCODE_SPEED);
		motor[DOUBLE].speed.PWM = motor[DOUBLE].speed.speed_pid.output;
		//ֱ����
		PidCalc_IMU(&motor[LEFT].position.imu_pid,pitch_med,pitch,gyroy_kal,IMU_POSITION);
		PidCalc_IMU(&motor[RIGHT].position.imu_pid,pitch_med,pitch,gyroy_kal,IMU_POSITION);
		motor[LEFT].position.PWM = motor[LEFT].position.imu_pid.output;
		motor[RIGHT].position.PWM = motor[LEFT].position.PWM;
		//ת��
		Turn_out=Turn_PID(gyroz_kal,yaw);
		//�����ں�
		speedleft = motor[LEFT].position.PWM+motor[DOUBLE].speed.PWM+Turn_out;//������	��-��������gyroz����0
		speedright = motor[RIGHT].position.PWM+motor[DOUBLE].speed.PWM-Turn_out;
		//PWM����޷�
		if(speedleft>7000)speedleft=7000;//���7200
		if(speedright<-7000)speedright=-7000;
		if(speedright>7000)speedright=7000;//���7200
		if(speedleft<-7000)speedleft=-7000;
			 
#if (SPEED)
//SetPWM(targetspeed,targetspeed); //����PWM
//SetPWM(motor[DOUBLE].speed.speed_pid.output,motor[DOUBLE].speed.speed_pid.output); //����PWM
SetPWM(motor[LEFT].speed.PWM,motor[RIGHT].speed.PWM); //����PWM
#endif
		
#if (POSITION)
//		SetPWM(motor[LEFT].position.PWM, motor[RIGHT].position.PWM); //����PWM
SetPWM(speedleft, speedright); //����PWM
#endif
			
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update); //״̬�Ĵ���(TIMx_SR)��bit0��0
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
		PAout(5)=0; //BIN1=0
		PAout(4)=1; //BIN2=1
		TIM2->CCR4=pwmleft;
		TIM_SetCompare4(TIM2, pwmleft);
  }
  else if(pwmleft<0)//pwm<0 (BIN1, BIN2)=(1, 0) ��ת
  {
		PAout(5)=1; //BIN1=1
		PAout(4)=0; //BIN2=0
		TIM2->CCR4=-pwmleft;
		TIM_SetCompare4(TIM2, -pwmleft);
  }
	if(pwmright>=0)//pwm>=0 (BIN1, BIN2)=(0, 1) ��ת
  {
		PAout(2)=1; //BIN1=0
		PAout(1)=0; //BIN2=1
		TIM2->CCR1=pwmright;
		TIM_SetCompare1(TIM2, pwmright);
  }
  else if(pwmright<0)//pwm<0 (BIN1, BIN2)=(1, 0) ��ת
  {
		PAout(2)=0; //BIN1=1
		PAout(1)=1; //BIN2=0
		TIM2->CCR1=-pwmright;
		TIM_SetCompare1(TIM2, -pwmright);
  }
}
