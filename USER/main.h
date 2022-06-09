#ifndef MAIN_H
#define MAIN_H

#define LEFT 1
#define RIGHT 0
#include "pid.h"

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
	int seePositionSet;
	int seePositionNow;
	float PositionSet;
	float PositionNow;
	float PWM;
}position_control;

typedef struct{
	speed_control speed;
	position_control position;
}motor_control;


extern motor_control motor[2];

#endif
