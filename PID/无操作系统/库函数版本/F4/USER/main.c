#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "timer.h"
#include "SEGGER_RTT.h"
#include "main.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "kalman.h"
//MOTOR
int   TargetVelocity=100, EncoderLeft,EncoderRight,PWM_left,PWM_right,Turn_out;  //Ŀ���ٶȡ�Ŀ��Ȧ��(λ��)��������������PWM���Ʊ���
motor_control motor[2];
speed_difference speed_diff[2];
//MPU6050
float pitch,roll,yaw,pitch_kalman; 		//ŷ����							//��������ʵ�ʲ���	ŷ����
short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����	//��������ʵ�ʲ���	���ٶ�
short gyrox,gyroy,gyroz;	//������ԭʼ����			//��������ʵ�ʲ���	�����ǽǶ�
float pitch_med = -5.8;//-3.5;			//��������ֵ		-7��ˮƽ��ȴ���ǻ���  -12 -3��ֵ		1.1-4.1
float groy_med = -37;
float targetspeed = 0;	//�ٶȻ�������ֱ��ʱΪ0.����ʱ��ֵ��
int groz_med = 0;	//ת��������Ϊ0ʱ��ֱ�ߣ�
int pitchsee;
int Turn_Kp,gyroz_med,gyroz_set;
//my_value
int seepitch;
int Res;
u8 usart_value;
int speedleft,speedright;

int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);  //��ʼ����ʱ����
	uart_init(9600);//��ʼ�����ڲ�����Ϊ115200
	LED_Init();			     //LED�˿ڳ�ʼ��
	MPU_Init();					        //��ʼ��MPU6050
  while(mpu_dmp_init()!=0){}	//��ʼ���ɹ���0����ʼ�����ɹ���ѭ����ֱ����ʼ���ɹ���ƽ�ų�ʼ�����׳ɹ���
//	TIM2_PWM_Init(8399,0);	//84M/84=1Mhz�ļ���Ƶ��,��װ��ֵ500������PWMƵ��Ϊ 1M/500=2Khz.     
//  TIM3_Encode_Init();
//	TIM4_Encode_Init();
//	EncoderRead_TIM1(8399, 99);//�˶�ʱ��Ӱ��iicʱ��
//	KalmanCreate(&speedK, 1, 50);	//Q R	
		//��һ�ε�����
//	PIDInit(&motor[LEFT].position.imu_pid,-800,0,-4,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
//	PIDInit(&motor[RIGHT].position.imu_pid,-800,0,-4,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
		//���Լ�D��jiaD����Ȼ�������ߣ�����̫������������
//		PIDInit(&motor[LEFT].position.imu_pid,-800,0,-12,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
//	PIDInit(&motor[RIGHT].position.imu_pid,-800,0,-12,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
		//���Լ�P	��Ч��������࣬���ҵ��˸��õ���ֵ������̩��
//		PIDInit(&motor[LEFT].position.imu_pid,-3000,0,-8,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
//	PIDInit(&motor[RIGHT].position.imu_pid,-3000,0,-8,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
		//���Լ�СD��P��D̫С�����ػΣ�
//	PIDInit(&motor[LEFT].position.imu_pid,-2000,0,-4,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
//	PIDInit(&motor[RIGHT].position.imu_pid,-2000,0,-4,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
		//������D	������	��̫����������������������ˣ����Ӹ߶Ȳ�ͬ���ٶȲ�ͬ��pitch��һֱ��
//		PIDInit(&motor[LEFT].position.imu_pid,-1760,0,-2.4,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
//	PIDInit(&motor[RIGHT].position.imu_pid,-1760,0,-2.4,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
		//����ˣ��λ����ƿ������������ǻ��Ƕ��������������������ˣ��ȳ����������һЩD
//	PIDInit(&motor[LEFT].position.imu_pid,-3800,0,-5,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
//	PIDInit(&motor[RIGHT].position.imu_pid,-3800,0,-5,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
//4.5��			3300 4.5���ض�		3000 4.5�����	2800 4.5��������Щ��	3000 9 �����£�����������
//������ȥ����ǿֱ��
// PIDInit(&motor[LEFT].position.imu_pid,-3500,0,-5,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
// PIDInit(&motor[RIGHT].position.imu_pid,-3500,0,-5,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);

/**************** ȥ������ *******************/
//����������ô��������Ƿ��ֲ��ٲ��ܼӺ��û�������Ǹ��������ˣ����ǳ�Ƶ������
// PIDInit(&motor[LEFT].position.imu_pid,-500,0,-0,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
// PIDInit(&motor[RIGHT].position.imu_pid,-500,0,-0,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
//���������Ƕ�����ȥ��-3.5��
// PIDInit(&motor[LEFT].position.imu_pid,-3500,0,-5,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
// PIDInit(&motor[RIGHT].position.imu_pid,-3500,0,-5,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
//�Ѿ������ˣ�������һ��㶶�����ܻ�е��ֵ����Щƫ 
// PIDInit(&motor[LEFT].position.imu_pid,-2800,0,-5,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
// PIDInit(&motor[RIGHT].position.imu_pid,-2800,0,-5,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
//Ч������
// PIDInit(&motor[LEFT].position.imu_pid,-1000,0,-0.8,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
// PIDInit(&motor[RIGHT].position.imu_pid,-1000,0,-0.8,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
//����
// PIDInit(&motor[LEFT].position.imu_pid,-400,0,-0.5,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
// PIDInit(&motor[RIGHT].position.imu_pid,-400,0,-0.5,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
//��ס
 //ֱ�����������ǣ�
 PIDInit(&motor[LEFT].position.imu_pid,-4000,0,-6.6,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
 PIDInit(&motor[RIGHT].position.imu_pid,-4000,0,-6.6,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
 //���ٻ���kpת�򻷣������ǣ�
// PIDInit(&speed_diff[DOUBLE].diff_pid,6,0,0,0,6500, 0, 250, 40 * 19, 30 * 19 * 2, 1350, SPEED);
 Turn_Kp=4;
 //pi�ٶȻ��������ǣ�
 PIDInit(&motor[DOUBLE].speed.speed_pid,0.03,0.00015,0,0,6500, 0, 250, 40 * 19, 30 * 19 * 2, 1350, SPEED);
 //pid�ٶȻ�����������
 PIDInit(&motor[LEFT].speed.speed_pid,60,0.1,0.5,0,6500, 0, 250, 40 * 19, 30 * 19 * 2, 1350, SPEED);
 PIDInit(&motor[RIGHT].speed.speed_pid,60,0.1,0.5,0,6500, 0, 250, 40 * 19, 30 * 19 * 2, 1350, SPEED);
 //pidλ�û�����������
 PIDInit(&motor[LEFT].position.position_pid,420,0,600,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
 PIDInit(&motor[RIGHT].position.position_pid,420,0,600,0,6000, 0, 20, 3600, 30 * 19 * 4, 970, POSITION_360);
  while(1) //ʵ�ֱȽ�ֵ��0-300��������300���300-0�ݼ���ѭ��
	{
		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
		{
		mpu_dmp_get_data(&pitch,&roll,&yaw);	//�õ�����ʱ��return 0������ѭ�������û�õ����ݣ�һֱѭ����ֱ���õ�dmp�����������
     MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);  // ��ȡ���ٶ�
     MPU_Get_Accelerometer(&aacx,&aacy,&aacz); // ��ȡ���ٶ�
		}
		switch(usart_value)
		{
			case Return:
				targetspeed = 0;
			  groz_med = 0;
				break;
			case Go_font:
				targetspeed = 30;
			  break;
			case Go_back:
				targetspeed = -30;
			  break;
			case Go_left:
				groz_med = -90;
			  break;
			case Go_right:
				groz_med = 90;
			  break;
		}
		SEGGER_RTT_printf(0,"pitch_kalman %d\r\n",pitch_kalman);
//					if((lastspeed-viewspeed)>50||(lastspeed-viewspeed)<-50)
//					SEGGER_RTT_printf(0,"times %d\r\n",times++);
	}
}
