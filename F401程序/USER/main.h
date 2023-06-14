#ifndef MAIN_H
#define MAIN_H
#include "pid.h"
#include "kalman.h"

#define LEFT 1
#define RIGHT 0
#define DOUBLE 2

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


//外部变量 extern说明改变量已在其它文件定义
extern int   Encoder, CurrentPosition;
extern int   TargetVelocity, CurrentPosition, EncoderLeft,EncoderRight,PWM_left,PWM_right,Turn_out;
extern int speedleft,speedright;
extern float targetspeed;	
extern int groz_med;
extern float pitch,roll,yaw,pitch_med,groy_med,pitch_kalman,yaw_kalman;
extern int pitchsee;
extern short gyrox,gyroy,gyroz;
extern short aacx,aacy,aacz;
extern int gyroz_med,gyroz_set;
extern float Turn_Kp,Turn_Kd;
extern u16 flag;
extern PidTypeDef position;
extern PidTypeDef speed;
extern short gyroy_kal;
extern short gyroz_kal;
extern int EncoderLeft_kal;
extern int EncoderRight_kal;
extern extKalman_t MPU6050_Kalman;
extern extKalman_t MOTOR_Kalman;
extern float groy_X;
extern int task_flag;
extern int task_flag_last;
extern int times;
extern float Yaw_target;

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


extern motor_control motor[3];
extern u8 usart_value;
extern speed_difference speed_diff[3];
#endif
