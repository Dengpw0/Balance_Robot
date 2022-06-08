#include "key.h"
#include "delay.h"
//由于"key.h"中已引用"stm32f10x.h" ，则此处无须重复引用
extern u8 key_value;
void KEY_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure; //定义一个引脚初始化的类
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU; //引脚输入输出模式为上拉输入模式，默认为高电平，外接地后为低电平
	GPIO_Init(GPIOC, &GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU; //引脚输入输出模式为上拉输入模式，默认为高电平，外接地后为低电平
	GPIO_Init(GPIOE, &GPIO_InitStructure); 
}

u8 KEY_Scan()
{
	static u8 flag_key=1;//按键松开标志，static使flag_key在函数执行完后依然存在，值依然不变
	if(flag_key==1&&(KEY1==0||KEY2==0)) //flag_key==1代表按键状态为松开，KEY==0代表检测到按键处于按下状态，两者结合代表一次按下动作
	{                      //若flag_key==0&&KEY==0，则表示按键一直处于按下、没有松开的状态
		delay_ms(10); //防抖
		if(KEY1==0)
	  {
			flag_key=0; //确认按键按下，按键松开标志置0
	    return 1;	  //按键按下，函数返回值1
		}
		else if(KEY2==0)
		{
			flag_key=0; //确认按键按下，按键松开标志置0
	    return 2;	  //按键按下，函数返回值2
		}
	}
  else if(KEY1==1&&KEY2==1) flag_key=1; //检测到按键松开，按键松开标志置1
	return 0;//无按键按下，函数返回值0
}
