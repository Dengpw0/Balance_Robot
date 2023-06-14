/**************************************************************************
 * @ file     	PID.c
 * @ brief    	PID�㷨
 * @ writer		������
 * @ Q Q		2296054658
 **************************************************************************/
#include "pid.h"

/**
	* @name f_abs()
	* @brief ����ֵ
**/
float f_abs(float num)
{
	if(num>=0)
		return num;
	else 
		return -num;	
}

/**
  * @name 	PID_Init()
  * @brief 	PID������ʼ��
  * @param	pid:��Ҫ��ʼ���Ľṹ��
  * @param	kp:ϵ��p
  * @param	ki:ϵ��i
  * @param	kd:ϵ��d
  * @param	ka:ϵ��a
  * @param	max_out:������
  * @param	dead_band:�������ڴ˷�Χ�ڲ�����PID����ֱ�����0
  * @param	i_band:�������䣬�ڴ˷�Χ�ڽ��л���
  * @param	max_input:����ֵ���������
  * @param	e_max:����ʱ��������
  * @param	i_maxout:i��������
  * @return None
  */
void PIDInit(PidTypeDef *pid,double kp,double ki,double kd,double ka,double max_out,double dead_band,double i_band,double max_input, double e_max, double i_maxout, pid_mode_e model)
{
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
	
	if(ka < 0 || ka >= 1)
		pid->Ka = 0;
	else
		pid->Ka = ka;
	
	if(max_out < 0)
		pid->max_out = 65535/2 - 1;
	else
		pid->max_out = max_out;
	
	if(max_input < 0)
		pid->max_input = 65535/2-1;
	else
		pid->max_input = max_input;
	
	if(i_maxout < 0)
		pid->intergral_maxOut = pid->max_out;
	else
		pid->intergral_maxOut = i_maxout;
	
	if(dead_band < 0)
		pid->dead_band = 0;
	else
		pid->dead_band = dead_band;
	
	if(i_band < 0)
		pid->intergral_band = 2*pid->max_input;
	else
		pid->intergral_band = i_band;
	
	pid->model = model;
	pid->err_max = e_max;
}

/**
  * @name 	PIDOutputClear()
  * @brief 	���PID�ṹ������
  * @param	pid:��Ҫ��յĽṹ��
  * @return None
  */
void PIDOutputClear(PidTypeDef *pid){
	pid->output = 0;
	pid->I_output = 0;
}

/**
  * @name 	PID_Calc()
  * @brief 	PID���㺯������������������pid->output������
  * @param	pid:����PID��������Ľṹ��
  * @param	rel_val:PID�����ʵ��ֵֵ
  * @param	set_val:PID���������ֵ
  * @return None
  */
void PID_Calc(PidTypeDef *pid, float rel_val, float set_val)
{
	if(set_val > pid->max_input)
		set_val = pid->max_input;
	else if(set_val < -(pid->max_input))
		set_val = -(pid->max_input);

	pid->err = set_val - rel_val; //��ǰ���
	
	/*360ת��PIDλ�ü����0����*/
	if(pid->model == POSITION_360)
	{
		if(pid->err>(pid->max_input/2))
		{
			pid->err=pid->err- pid->max_input;
		}
		else if((-pid->err)>(pid->max_input/2))
		{
			pid->err=pid->max_input + pid->err;
		}
	}
	else if(pid->model == POSITION_180)
	{
		if(pid->err>(pid->max_input))
		{
			pid->err=pid->err- 2 * pid->max_input;
		}
		else if((-pid->err)>(pid->max_input))
		{
			pid->err=pid->max_input * 2 + pid->err;
		}
	}
	
	//��������С
	if(pid->err > pid->err_max)
		pid->err = pid->err_max;
	else if(pid->err < -pid->err_max)
		pid->err = -pid->err_max;
	
	if(f_abs(pid->err) > pid->dead_band)
	{ //�ж��Ƿ����������û�н��������PID���㣬���������0
		
		//�ж��Ƿ��ڻ��������ڣ�����ھͽ��л��֣����ھ�ʹ�û����𽥹�0
		if(f_abs(pid->err) <= pid->intergral_band)
			pid->I_output = pid->I_output + (pid->Ki) * (pid->err);
		else
			pid->I_output = pid->I_output*0.99f;
		
		//���ƻ������ֵ
		if(pid->I_output > pid->intergral_maxOut)
			pid->I_output = pid->intergral_maxOut;
		else if(pid->I_output < -pid->intergral_maxOut)
			pid->I_output= -pid->intergral_maxOut;
		
		//����PID���
		pid->P_output = pid->Kp * (pid->err);
		pid->D_output = pid->Kd * (1-pid->Ka)*(pid->err - pid->err_last) + (pid->Ka)*(pid->d_last);
		pid->d_last = pid->D_output;
		pid->output = pid->P_output+pid->I_output+pid->D_output;
		
		//�Ƿ񳬳�������
		if(pid->output > pid->max_out)
			pid->output = pid->max_out;
		if(pid->output < -(pid->max_out))
			pid->output = -(pid->max_out);
			
	}
	else
	{
		pid->output *= 0.99f;
		pid->I_output *= 0.99f;
	}
	
	pid->err_last = pid->err;
}
