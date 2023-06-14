#include "pid.h"
#include "main.h"
/**
  * @name 	PID_Init()
  * @brief 	PID参数初始化
  * @param	pid:所要初始化的结构体
  * @param	kp:系数p
  * @param	ki:系数i
  * @param	kd:系数d
  */
void PIDInit(PidTypeDef *pid,double kp,double ki,double kd)
{
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
}

/**
* @name PidCalc_IMU
* @use 	直立环
**/
void PidCalc_IMU(PidTypeDef *pid, float med, float pitch, float gyroy,float type)
{
	float PWM_out;
  PWM_out = pid->Kp*(pitch-med)+pid->Kd*(gyroy-groy_med);
	pid->output = PWM_out;
}
/**
* @name PidCalc_Encode
* @use 	速度环
**/
void PidCalc_Encode(PidTypeDef *pid, float left, float right, float targetspeed, float type)
{
	static float PWM_out;
	static float Encoder_S;
	static float Encoder_Least,Encoder_Least_last;
	static float Err_Lowout,Err_Lowout_last;	//低通滤波
  static float a=0.7;
	// 1.计算速度偏差
  // 舍去误差--我的理解：能够让速度为"0"的角度，就是机械中值。
	Encoder_Least = (left+right) - targetspeed;
	// 2.对速度偏差进行低通滤波
  // low_out = (1-a)*Ek+a*low_out_last
  Err_Lowout = (1-a)*Encoder_Least + a*Encoder_Least_last; // 使得波形更加平滑，滤除高频干扰，放置速度突变
  Err_Lowout_last = Err_Lowout;   // 防止速度过大影响直立环的正常工作
  // 3.对速度偏差积分出位移
  Encoder_S+=Err_Lowout;
  // 4.积分限幅
  Encoder_S=Encoder_S>10000?10000:(Encoder_S<(-10000)?(-10000):Encoder_S);
  
  // 5.速度环控制输出
  PWM_out = pid->Kp*Err_Lowout+pid->Ki*Encoder_S;
  
  pid->output = PWM_out;	
}
/**
* @name Turn_PID
* @use 	转向环
**/
int Turn_PID(int gyro_Z,int yaw)
{
	int PWM_out;    
	float Bias;	  
	Bias=yaw - Yaw_target;
	PWM_out = Turn_Kd*(gyro_Z-groz_med) + Bias*Turn_Kp;
	return PWM_out;
}
