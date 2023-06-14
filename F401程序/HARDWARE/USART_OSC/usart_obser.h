 /**************************************************************************
 * @file     	oscillography.h/.c
 * @brief    	ͨ������2������λ����ʾ���ε���ʹ��
 **************************************************************************/
#ifndef OSCILLOGRAPHY_H
#define OSCILLOGRAPHY_H
 
#include "stm32f4xx.h"
void upComputer_dataRecieve(s16 roll_p, s16 roll_i, s16 roll_d, s16 pitch_p, s16 pitch_i, s16 pitch_d, s16 yaw_p, s16 yaw_i, s16 yaw_d);
 void oscillography_sendData(s16 *dataSend, u8 len);

extern s16 oscillographyData[10];	//������ʾ����

#endif
