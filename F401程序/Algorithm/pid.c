#include "pid.h"
#include "main.h"
/**
  * @name 	PID_Init()
  * @brief 	PID������ʼ��
  * @param	pid:��Ҫ��ʼ���Ľṹ��
  * @param	kp:ϵ��p
  * @param	ki:ϵ��i
  * @param	kd:ϵ��d
  */
void PIDInit(PidTypeDef *pid,double kp,double ki,double kd)
{
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
}

/**
* @name PidCalc_IMU
* @use 	ֱ����
**/
void PidCalc_IMU(PidTypeDef *pid, float med, float pitch, float gyroy,float type)
{
	float PWM_out;
  PWM_out = pid->Kp*(pitch-med)+pid->Kd*(gyroy-groy_med);
	pid->output = PWM_out;
}
/**
* @name PidCalc_Encode
* @use 	�ٶȻ�
**/
void PidCalc_Encode(PidTypeDef *pid, float left, float right, float targetspeed, float type)
{
	static float PWM_out;
	static float Encoder_S;
	static float Encoder_Least,Encoder_Least_last;
	static float Err_Lowout,Err_Lowout_last;	//��ͨ�˲�
  static float a=0.7;
	// 1.�����ٶ�ƫ��
  // ��ȥ���--�ҵ���⣺�ܹ����ٶ�Ϊ"0"�ĽǶȣ����ǻ�е��ֵ��
	Encoder_Least = (left+right) - targetspeed;
	// 2.���ٶ�ƫ����е�ͨ�˲�
  // low_out = (1-a)*Ek+a*low_out_last
  Err_Lowout = (1-a)*Encoder_Least + a*Encoder_Least_last; // ʹ�ò��θ���ƽ�����˳���Ƶ���ţ������ٶ�ͻ��
  Err_Lowout_last = Err_Lowout;   // ��ֹ�ٶȹ���Ӱ��ֱ��������������
  // 3.���ٶ�ƫ����ֳ�λ��
  Encoder_S+=Err_Lowout;
  // 4.�����޷�
  Encoder_S=Encoder_S>10000?10000:(Encoder_S<(-10000)?(-10000):Encoder_S);
  
  // 5.�ٶȻ��������
  PWM_out = pid->Kp*Err_Lowout+pid->Ki*Encoder_S;
  
  pid->output = PWM_out;	
}
/**
* @name Turn_PID
* @use 	ת��
**/
int Turn_PID(int gyro_Z,int yaw)
{
	int PWM_out;    
	float Bias;	  
	Bias=yaw - Yaw_target;
	PWM_out = Turn_Kd*(gyro_Z-groz_med) + Bias*Turn_Kp;
	return PWM_out;
}
