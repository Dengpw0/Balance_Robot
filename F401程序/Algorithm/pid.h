/**************************************************************************
 * @ file     	PID.h
 * @ brief    	PID�㷨
 * @ writer		������
 * @ Q Q		2296054658
 **************************************************************************/
#ifndef _PID_H
#define _PID_H

#include "stm32f4xx.h"

typedef struct
{
    //PID ������
	float Kp;			//����ϵ������Ҫ������Ӧ�ٶȣ��������׳���
	float Ki;			//����ϵ��������������̬������С��Χ�ڵķ�����
	float Kd;			//΢��ϵ����ʵ�־���Ԥ�⹦�ܣ���С����
	
	//PID���ֵ
	float output;		//���
	float P_output;		//P���
	float I_output;		//I���
	float D_output;		//D���
	
} PidTypeDef;

void PIDInit(PidTypeDef *pid,double kp,double ki,double kd);
void PidCalc_IMU(PidTypeDef *pid, float med, float pitch, float gyroy,float type);
void PidCalc_Encode(PidTypeDef *pid, float left, float right, float targetspeed, float type);
int Turn_PID(int gyro_Z,int yaw);
#endif
