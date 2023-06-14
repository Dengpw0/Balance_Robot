/**************************************************************************
 * @ file     	PID.h
 * @ brief    	PID算法
 * @ writer		宋立栋
 * @ Q Q		2296054658
 **************************************************************************/
#ifndef _PID_H
#define _PID_H

#include "stm32f4xx.h"

typedef struct
{
    //PID 三参数
	float Kp;			//比例系数，主要控制相应速度，大了容易超调
	float Ki;			//积分系数，用于消除稳态误差，增大小误差范围内的反馈立
	float Kd;			//微分系数，实现具有预测功能，减小超调
	
	//PID输出值
	float output;		//输出
	float P_output;		//P输出
	float I_output;		//I输出
	float D_output;		//D输出
	
} PidTypeDef;

void PIDInit(PidTypeDef *pid,double kp,double ki,double kd);
void PidCalc_IMU(PidTypeDef *pid, float med, float pitch, float gyroy,float type);
void PidCalc_Encode(PidTypeDef *pid, float left, float right, float targetspeed, float type);
int Turn_PID(int gyro_Z,int yaw);
#endif
