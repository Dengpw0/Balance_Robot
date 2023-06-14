#ifndef MAIN_H
#define MAIN_H
#include "pid.h"


#define LEFT 1
#define RIGHT 0
#define DOUBLE 3

#define Return 0
#define Go_font 1
#define Go_back 2
#define Go_left 3
#define Go_right 4


enum{
IMU_POSITION = 0,
ENCODE_POSITION,
ENCODE_SPEED
};


typedef struct{
	PidTypeDef speed_pid;
	int seeSpeedSet;
	int seeSpeedNow;
	float SpeedSet;
	float SpeedNow;
	float PWM;
}speed_control;
typedef struct{
	PidTypeDef position_pid;
	PidTypeDef imu_pid;
	int seePositionSet;
	int seePositionNow;
	float PositionSet;
	float PositionNow;
	float PWM;
}position_control;
typedef struct{
	PidTypeDef diff_pid;
}speed_difference;
typedef struct{
	speed_control speed;
	position_control position;
	
}motor_control;


extern motor_control motor[2];
extern u8 usart_value;
extern speed_difference speed_diff[2];
#endif
