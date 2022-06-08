#include "motorencoder.h" 
#include "pid.h"
#include "led.h"

//�ⲿ���� extern˵���ı������������ļ�����
extern int   Encoder, CurrentPosition; //��ǰ�ٶȡ���ǰλ��
extern int   TargetVelocity, CurrentPosition, Encoder,SPEED_PWM,POSITION_PWM;//Ŀ���ٶȡ�Ŀ��Ȧ����������������PWM���Ʊ���
extern float TargetCircle,AngleNow,AngleSet,SpeeSet,SpeedNow,AngleMax,k,AngleMin;
extern float Velcity_Kp,  Velcity_Ki,  Velcity_Kd; //����ٶ�PID����
extern float Position_Kp, Position_Ki, Position_Kd; //���λ��PID����
extern int   MortorRun;  //���������Ʊ�־λ

extern u16 flag;
extern PidTypeDef position;
extern PidTypeDef speed;

extern u16 times;
/**************************************************************************
�������ܣ���������ʼ������
��ڲ�������
����  ֵ����
**************************************************************************/
void MotorEncoder_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; //����һ�����ų�ʼ���Ľṹ��  
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;//����һ����ʱ����ʼ���Ľṹ��
  TIM_ICInitTypeDef TIM_ICInitStructure; //����һ����ʱ��������ģʽ��ʼ���Ľṹ��
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //ʹ��TIM4ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //ʹ��CPIOBʱ��
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	//TIM4_CH1��TIM4_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //��������
	GPIO_Init(GPIOB, &GPIO_InitStructure);	//����GPIO_InitStructure�Ĳ�����ʼ��GPIO

	TIM_TimeBaseStructure.TIM_Period = 0xffff; //�趨�������Զ���װֵ
	TIM_TimeBaseStructure.TIM_Prescaler = 0; // Ԥ��Ƶ�� 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //ѡ��ʱ�ӷ�Ƶ������Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct�Ĳ�����ʼ����ʱ��TIM4
	
  TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //ʹ�ñ�����ģʽ3��CH1��CH2ͬʱ������Ϊ�ķ�Ƶ
  TIM_ICStructInit(&TIM_ICInitStructure); //��TIM_ICInitStruct �е�ÿһ��������ȱʡֵ����
	TIM_ICInitStructure.TIM_ICFilter = 10;  //�����˲�������
  TIM_ICInit(TIM4, &TIM_ICInitStructure); //��TIM_ICInitStructure������ʼ����ʱ��TIM4������ģʽ
	
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);//���TIM�ĸ��±�־λ
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); //�����ж�ʹ��
  TIM_SetCounter(TIM4,0); //��ʼ����ձ�������ֵ
	
	TIM_Cmd(TIM4, ENABLE); //ʹ�ܶ�ʱ��4
}

/**************************************************************************
�������ܣ���ȡTIM4��������ֵ
��ڲ�������
����  ֵ����
**************************************************************************/
int Read_Encoder(void)
{
	int Encoder_TIM;
	Encoder_TIM=TIM4->CNT; //��ȡ����
	if(Encoder_TIM>0xefff)Encoder_TIM=Encoder_TIM-0xffff; //ת������ֵΪ�з����ֵ������0��ת��С��0��ת��
	                                                      //TIM4->CNT��ΧΪ0-0xffff����ֵΪ0��
	TIM4->CNT=0; //��ȡ����������
	return Encoder_TIM; //����ֵ
}

/**************************************************************************
�������ܣ�TIM4�жϷ�����
��ڲ�������
����  ֵ����
**************************************************************************/
void TIM4_IRQHandler(void)
{ 		    		  			    
	if(TIM4->SR&0X0001)//����ж�
	{    				   				     	    	
	}				   
	TIM4->SR&=~(1<<0);//����жϱ�־λ 	    
}

/**************************************************************************
�������ܣ�ͨ�ö�ʱ��2��ʼ��������
��ڲ������Զ���װ��ֵ Ԥ��Ƶϵ�� Ĭ�϶�ʱʱ��Ϊ72MHZʱ�����߹�ͬ������ʱ�ж�ʱ��
����  ֵ����
**************************************************************************/
void EncoderRead_TIM2(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue; //����һ����ʱ�жϵĽṹ��
	NVIC_InitTypeDef NVIC_InitStrue; //����һ���ж����ȼ���ʼ���Ľṹ��
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //ʹ��ͨ�ö�ʱ��2ʱ��
	
	TIM_TimeBaseInitStrue.TIM_Period=arr; //����ģʽΪ���ϼ���ʱ����ʱ����0��ʼ����������������arrʱ������ʱ�жϷ�����
	TIM_TimeBaseInitStrue.TIM_Prescaler=psc; //Ԥ��Ƶϵ��������ÿһ��������ʱ��
	TIM_TimeBaseInitStrue.TIM_CounterMode=TIM_CounterMode_Up; //����ģʽ�����ϼ���
	TIM_TimeBaseInitStrue.TIM_ClockDivision=TIM_CKD_DIV1; //һ�㲻ʹ�ã�Ĭ��TIM_CKD_DIV1
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStrue); //����TIM_TimeBaseInitStrue�Ĳ�����ʼ����ʱ��TIM2
	
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); //ʹ��TIM2�жϣ��ж�ģʽΪ�����жϣ�TIM_IT_Update
	
	NVIC_InitStrue.NVIC_IRQChannel=TIM2_IRQn; //����TIM2�ж�
	NVIC_InitStrue.NVIC_IRQChannelCmd=ENABLE; //�ж�ʹ��
	NVIC_InitStrue.NVIC_IRQChannelPreemptionPriority=1; //��ռ���ȼ�Ϊ1����ֵԽС���ȼ�Խ�ߣ�0�����ȼ����
	NVIC_InitStrue.NVIC_IRQChannelSubPriority=1; //��Ӧ���ȼ�Ϊ1����ֵԽС���ȼ�Խ�ߣ�0�����ȼ����
	NVIC_Init(&NVIC_InitStrue); //����NVIC_InitStrue�Ĳ�����ʼ��VIC�Ĵ���������TIM2�ж�
	
	TIM_Cmd(TIM2, ENABLE); //ʹ�ܶ�ʱ��TIM2
}

/**************************************************************************
�������ܣ�TIM2�жϷ����� ��ʱ��ȡ��������ֵ������λ�ñջ����� 10ms����һ��
��ڲ�������
����  ֵ����
**************************************************************************/
#define SPEED 0
#define POSITION 1
void TIM2_IRQHandler()
{
  if(TIM_GetITStatus(TIM2, TIM_IT_Update)==1) //�������ж�ʱ״̬�Ĵ���(TIMx_SR)��bit0�ᱻӲ����1
	{
	  Encoder=Read_Encoder();   //��ȡ��ǰ���������������ٶ�

//		//�ٶȻ�
//		SpeeSet = TargetVelocity; //PWMֵת��Ϊ�ٶ�ֵ 76Ϊת������;
//		SpeedNow = Encoder*1000*200*0.000120830;
//		PID_Calc(&speed,SpeedNow,SpeeSet);
//		SPEED_PWM = speed.output;		
//		//λ�û�
//		TargetCircle = SPEED_PWM/2000;	//���������
//		AngleSet = TargetCircle*1560*1.04/2;
//		AngleNow += Encoder;
//		PID_Calc(&position,AngleNow,AngleSet);
//		POSITION_PWM = position.output;
		//������������
		times ++;
		if((times - 30)>0)
		{
			times -= 30;
			LED1=!LED1;
		}
		
		//λ�û�
		AngleSet = TargetCircle*1560*1.04/2;
		AngleNow += Encoder;
		PID_Calc(&position,AngleNow,AngleSet);
		POSITION_PWM = position.output;
		
		//�ٶȻ�
		SpeeSet = POSITION_PWM/76; //PWMֵת��Ϊ�ٶ�ֵ 76Ϊת������;
		SpeedNow = Encoder*1000*200*0.000120830;
		PID_Calc(&speed,SpeedNow,SpeeSet);
		SPEED_PWM = speed.output;		

#if (SPEED==1)
		SetPWM(SPEED_PWM); //����PWM
#endif
		
#if (POSITION==1)
		SetPWM(POSITION_PWM); //����PWM
#endif
			
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update); //״̬�Ĵ���(TIMx_SR)��bit0��0
	}
}

/**************************************************************************
�������ܣ��ٶȱջ�PID����(ʵ��ΪPI����)
��ڲ�����Ŀ���ٶ� ��ǰ�ٶ�
����  ֵ���ٶȿ���ֵ
��������ʽ��ɢPID��ʽ 
ControlVelocity+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
ControlVelocity�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
ControlVelocity+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/
//int Velocity_FeedbackControl(int TargetVelocity, int CurrentVelocity)
//{
//		int Bias;  //������ر���
//		static int ControlVelocity, Last_bias; //��̬�������������ý�������ֵ��Ȼ����
//		
//		Bias=TargetVelocity-CurrentVelocity; //���ٶ�ƫ��
//		
//		ControlVelocity+=Velcity_Kp*(Bias-Last_bias)+Velcity_Ki*Bias;  //����ʽPI������
//                                                                   //Velcity_Kp*(Bias-Last_bias) ����Ϊ���Ƽ��ٶ�
//	                                                                 //Velcity_Ki*Bias             �ٶȿ���ֵ��Bias���ϻ��ֵõ� ƫ��Խ����ٶ�Խ��
//		Last_bias=Bias;	
//		return ControlVelocity; //�����ٶȿ���ֵ
//}

/**************************************************************************
�������ܣ�λ��ʽPID������
��ڲ�����Ŀ��Ȧ��(λ��) ��ǰλ��
����  ֵ���ٶȿ���ֵ
����λ��ʽ��ɢPID��ʽ 
pwm=Kp*e(k)+Ki*��e(k)+Kd[e��k��-e(k-1)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  
��e(k)����e(k)�Լ�֮ǰ��ƫ����ۻ���;����kΪ1,2,,k;
pwm�������
**************************************************************************/
//int Position_FeedbackControl(float Circle, int CurrentPosition)
//{
//		float TargetPosition,Bias, ControlVelocity;     //������ر���
//		static float Last_bias, Integral_Bias;          //��̬�������������ý�������ֵ��Ȼ����
//		
//	  TargetPosition=Circle*1560*1.04; //Ŀ��λ��=Ŀ��Ȧ��*1040
//	                                   //10ms��ȡһ�α�����(��100HZ)��������ٱ�Ϊ20����������������13��AB˫����ϵõ�4��Ƶ��
//	                                   //��ת1Ȧ����������Ϊ20*13*4=1040�����ת��=Encoder*100/1040r/s ʹ�ö�ʱ��2
//	                                   //1.04�����ϵ�����������������ɸ���ʵ�����������ϵ������߿��ƾ���
//		Bias=TargetPosition-CurrentPosition; //��λ��ƫ��
//	  Integral_Bias+=Bias;
//    if(Integral_Bias> 970) Integral_Bias= 970;	//�����޷� ��ֹ����Ŀ��λ�ú����
//	  if(Integral_Bias<-970) Integral_Bias=-970;	//�����޷� ��ֹ����Ŀ��λ�ú����
//	
//		ControlVelocity=Position_Kp*Bias+Position_Ki*Integral_Bias+Position_Kd*(Bias-Last_bias);  //����ʽPI������
//	                                                                                            //Position_Kp*Bias ƫ��Խ���ٶ�Խ��
//	                                                                                            //Position_Ki*Integral_Bias ��С��̬���
//	                                                                                            //Position_Kd*(Bias-Last_bias) �����ٶ�

//		Last_bias=Bias;	
//		return ControlVelocity;    //�����ٶȿ���ֵ 
//}


