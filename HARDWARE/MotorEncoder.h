#ifndef __MOTORENCODER_H 
#define __MOTORENCODER_H 
#include "sys.h"
#include "TB6612.h"

void MotorEncoderLeft_Init(void); 
void MotorEncoderRight_Init(void); 
int Read_Encoder(int left_or_right);
void EncoderRead_TIM2(u16 arr, u16 psc);
int Position_FeedbackControl(float Circle, int CurrentVelocity);
int Velocity_FeedbackControl(int TargetVelocity, int CurrentVelocity);
int PWM_Restrict(int PWM_P, int TargetVelocity);
#endif
