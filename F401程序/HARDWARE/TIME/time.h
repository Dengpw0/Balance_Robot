#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//��ʱ�� ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/6/16
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

void TIM2_PWM_Init(u32 arr,u32 psc);
void TIM3_Encode_Init(void);
void TIM1_Encode_Init(void);
int Read_Encoder(int left_or_right);
void Control_TIM4(u16 arr, u16 psc);
void TICK_TIM5(u16 arr, u16 psc);
void SetPWM(int pwmleft,int pwmright);
#endif
