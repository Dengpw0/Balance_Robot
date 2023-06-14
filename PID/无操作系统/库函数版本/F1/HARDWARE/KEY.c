#include "key.h"
#include "delay.h"
//����"key.h"��������"stm32f10x.h" ����˴������ظ�����
extern u8 key_value;
void KEY_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure; //����һ�����ų�ʼ������
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU; //�����������ģʽΪ��������ģʽ��Ĭ��Ϊ�ߵ�ƽ����ӵغ�Ϊ�͵�ƽ
	GPIO_Init(GPIOC, &GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU; //�����������ģʽΪ��������ģʽ��Ĭ��Ϊ�ߵ�ƽ����ӵغ�Ϊ�͵�ƽ
	GPIO_Init(GPIOE, &GPIO_InitStructure); 
}

u8 KEY_Scan()
{
	static u8 flag_key=1;//�����ɿ���־��staticʹflag_key�ں���ִ�������Ȼ���ڣ�ֵ��Ȼ����
	if(flag_key==1&&(KEY1==0||KEY2==0)) //flag_key==1������״̬Ϊ�ɿ���KEY==0�����⵽�������ڰ���״̬�����߽�ϴ���һ�ΰ��¶���
	{                      //��flag_key==0&&KEY==0�����ʾ����һֱ���ڰ��¡�û���ɿ���״̬
		delay_ms(10); //����
		if(KEY1==0)
	  {
			flag_key=0; //ȷ�ϰ������£������ɿ���־��0
	    return 1;	  //�������£���������ֵ1
		}
		else if(KEY2==0)
		{
			flag_key=0; //ȷ�ϰ������£������ɿ���־��0
	    return 2;	  //�������£���������ֵ2
		}
	}
  else if(KEY1==1&&KEY2==1) flag_key=1; //��⵽�����ɿ��������ɿ���־��1
	return 0;//�ް������£���������ֵ0
}
