#include "usart_obser.h"
#include "stm32f4xx.h"
#include "usart.h"
#include "main.h"

typedef union
{
	float fd;
	s16 data[2];
} float_tt;

s16 oscillographyData[10];
//上位机发送数据给单片机时的回调函数
void upComputer_dataRecieve(s16 roll_p, s16 roll_i, s16 roll_d, s16 pitch_p, s16 pitch_i, s16 pitch_d, s16 yaw_p, s16 yaw_i, s16 yaw_d)
{
}
//向上位机发送一组数据，用于显示
void oscillography_sendData(s16 *dataSend, u8 len)
{
	u8 i = 0;
	u8 data[20];
	if (len > 10)
		len = 10;
	for (i = 0; i < len; i++)
	{
		data[2 * i] = dataSend[i];
		data[2 * i + 1] = (u8)(dataSend[i] >> 8);
	}
	Send_data8(data, len * 2, 1);
}

extKalman_t usr_look;

